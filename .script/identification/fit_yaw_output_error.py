#!/usr/bin/env python3
"""Constrained continuous-rollout output-error fit of J*w_dot + b*w = torque.

Requires only NumPy. Each half is initialized once from measured velocity;
no subsequent measured state is fed back into the simulation. Original 1 kHz
samples and their recorded time intervals are retained. Torque is zero-order held.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def simulate(t, torque, initial_velocity, parameters):
    inertia, damping = parameters
    dt = np.diff(t)
    decay = np.exp(-damping * dt / inertia)
    gain = -np.expm1(-damping * dt / inertia) / damping if damping else dt / inertia
    predicted = np.empty(len(t))
    predicted[0] = initial_velocity
    for i in range(len(dt)):
        predicted[i + 1] = decay[i] * predicted[i] + gain[i] * torque[i]
    return predicted


def nelder_mead(objective, start, bounds, iterations=180):
    """Bounded two-parameter Nelder-Mead in log parameter coordinates."""
    clip = lambda x: np.clip(x, bounds[:, 0], bounds[:, 1])
    points = np.array([start, clip(start + [0.3, 0]), clip(start + [0, 0.3])])
    values = np.array([objective(p) for p in points])
    converged = False
    for iteration in range(iterations):
        order = np.argsort(values)
        points, values = points[order], values[order]
        if np.max(np.abs(points - points[0])) < 1e-6:
            converged = True
            break
        center = points[:2].mean(axis=0)
        reflected = clip(2 * center - points[2]); fr = objective(reflected)
        if fr < values[0]:
            expanded = clip(center + 2 * (reflected - center)); fe = objective(expanded)
            points[2], values[2] = (expanded, fe) if fe < fr else (reflected, fr)
        elif fr < values[1]:
            points[2], values[2] = reflected, fr
        else:
            outside = fr < values[2]
            contracted = clip(center + 0.5 * ((reflected if outside else points[2]) - center))
            fc = objective(contracted)
            if fc < (fr if outside else values[2]):
                points[2], values[2] = contracted, fc
            else:
                for i in (1, 2):
                    points[i] = clip(points[0] + 0.5 * (points[i] - points[0]))
                    values[i] = objective(points[i])
    best = np.argmin(values)
    return points[best], float(values[best]), converged, iteration + 1


def fit(t, torque, velocity):
    # Numerical search bounds, not prior measurements of this robot.
    bounds = np.log(np.array([[1e-5, 10.0], [1e-6, 100.0]]))
    objective = lambda q: float(np.mean((simulate(t, torque, velocity[0], np.exp(q)) - velocity)**2))
    starts = [np.log([j, b]) for j in (0.001, 0.03, 0.3, 3.0) for b in (0.1, 3.0, 10.0)]
    # Rank initial candidates, then independently refine the best four.
    starts.sort(key=objective)
    runs = [nelder_mead(objective, s, bounds) for s in starts[:4]]
    q, value, converged, iterations = min(runs, key=lambda r: r[1])
    return np.exp(q), {
        'converged': converged, 'iterations': iterations, 'mse': value,
        'J_bounds': [1e-5, 10.0], 'b_bounds': [1e-6, 100.0],
        'at_search_boundary': bool(np.any(np.minimum(q-bounds[:,0], bounds[:,1]-q) < 1e-3)),
        'multistart_parameters': [np.exp(r[0]).tolist() for r in runs],
    }


def metrics(actual, predicted):
    mse = np.mean((actual-predicted)**2)
    variance = np.mean((actual-actual.mean())**2)
    return {'rmse_rad_s': float(np.sqrt(mse)),
            'r2': float(1-mse/variance),
            'fit_percent': float(100*(1-np.sqrt(mse/variance)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with args.csv_path.open() as f:
        reader = csv.DictReader(f); data=[]; skipped=0
        keys = ['/gimbal/sample_time_s', '/gimbal/yaw/torque', '/gimbal/yaw/velocity_imu',
                '/gimbal/yaw/excitation/state', '/gimbal/yaw/angle']
        for row in reader:
            try: values = [float(row[k]) for k in keys]
            except (TypeError, ValueError): skipped+=1; continue
            if values[3] == 2:
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('Nonfinite sample within excitation; split explicitly before fitting')
                data.append(values)
    a=np.array(data)
    if len(a)<20 or np.any(np.diff(a[:,0])<=0) or np.max(np.diff(a[:,0]))>0.01:
        raise ValueError('Need one contiguous excitation segment with increasing timestamps')
    split=np.searchsorted(a[:,0], (a[0,0]+a[-1,0])/2)
    train, validation=a[:split],a[split:]
    t,u,w=train[:,:3].T
    parameters, diagnostics=fit(t,u,w)
    report={'source':str(args.csv_path),'sha256':hashlib.sha256(args.csv_path.read_bytes()).hexdigest(),
            'model':'J*domega/dt + b*omega = measured_torque',
            'input':'current-derived output-shaft torque, zero-order hold',
            'output':'IMU yaw velocity', 'skipped_incomplete_rows':skipped,
            'J_kg_m2':float(parameters[0]),'b_Nm_s_rad':float(parameters[1]),
            'optimizer':diagnostics,'segments':{}}
    output=[]
    for name,segment in [('train',train),('validation',validation)]:
        t,u,w=segment[:,:3].T
        prediction=simulate(t,u,w[0],parameters)
        report['segments'][name]={'samples':len(t),'start_s':float(t[0]),'end_s':float(t[-1]),
                                  **metrics(w,prediction)}
        output.extend(zip([name]*len(t),t,u,w,prediction))
    # Also fit the complete sweep; this is a descriptive fit, not independent validation.
    full_parameters, full_diagnostics=fit(a[:,0],a[:,1],a[:,2])
    report['full_sweep_fit']={'J_kg_m2':float(full_parameters[0]),'b_Nm_s_rad':float(full_parameters[1]),
                             **metrics(a[:,2],simulate(a[:,0],a[:,1],a[0,2],full_parameters)),
                             'optimizer':full_diagnostics}
    (args.output/'fit.json').write_text(json.dumps(report,indent=2)+'\n')
    with (args.output/'prediction.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['segment','time_s','torque_Nm','measured_velocity_rad_s','predicted_velocity_rad_s']);writer.writerows(output)
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
