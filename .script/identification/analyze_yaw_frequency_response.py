#!/usr/bin/env python3
"""Local chirp harmonic fits; approximate frequency response, not steady sine tests."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def harmonic(signal, phase, time):
    # A local sinusoid plus offset and linear drift. Hann weights suppress edges.
    weight=np.sin(np.linspace(0,np.pi,len(time)))**2
    x=np.column_stack([np.cos(phase),np.sin(phase),np.ones(len(time)),time-time.mean()])
    coef=np.linalg.lstsq(x*np.sqrt(weight[:,None]),signal*np.sqrt(weight),rcond=None)[0]
    amplitude=float(np.hypot(coef[0],coef[1]))
    angle=float(np.arctan2(-coef[1],coef[0]))
    residual=signal-x@coef
    return amplitude,angle,float(np.sqrt(np.sum(weight*residual**2)/weight.sum()))


def lag(reference,actual):
    return float(np.degrees(np.angle(np.exp(1j*(reference-actual)))))


def analyze(path):
    with path.open() as f:
        reader=csv.DictReader(f);data=[]
        names=['/gimbal/sample_time_s']+['/gimbal/yaw/'+k for k in [
            'excitation/state','excitation/elapsed_s','excitation/frequency_hz',
            'excitation/velocity_rad_s','velocity_imu','control_torque','torque','control_angle_error']]
        for row in reader:
            try:v=[float(row[k]) for k in names]
            except (TypeError,ValueError):continue
            if v[1]==2 and np.isfinite(v).all():data.append(v)
    a=np.array(data);t=a[:,0];f=a[:,3];elapsed=a[:,2]
    # The logged linear chirp frequency determines phase up to an irrelevant constant.
    slope,intercept=np.polyfit(elapsed,f,1)
    phase=2*np.pi*(intercept*elapsed+slope*elapsed**2/2)
    results=[]
    # Current profile fades during its first/last 3 seconds: use only the interior.
    interior=(elapsed>=8)&(elapsed<=42)
    for low in np.arange(.5,4.5,.5):
        high=low+.5;m=interior&(f>=low)&(f<high)
        if m.sum()<100:continue
        tt=t[m];pp=phase[m]
        reference=harmonic(a[m,4],pp,tt);actual=harmonic(a[m,5],pp,tt)
        command=harmonic(a[m,6],pp,tt);feedback=harmonic(a[m,7],pp,tt)
        cycles=(pp[-1]-pp[0])/(2*np.pi)
        center=(low+high)/2
        # Approximate proportional cascade only; ignores discrete integral action/delays.
        s=1j*2*np.pi*center
        predicted=130/(.15732625601555403*s*s+(13+.8226911525281183)*s+130)
        results.append(dict(source=path.name,frequency_low_hz=float(low),frequency_high_hz=float(high),
            cycles=float(cycles),reference_velocity_amplitude=reference[0],actual_velocity_amplitude=actual[0],
            velocity_gain=actual[0]/reference[0],velocity_lag_deg=lag(reference[1],actual[1]),
            velocity_harmonic_residual_rms=actual[2],
            torque_gain=feedback[0]/command[0],torque_lag_deg=lag(command[1],feedback[1]),
            command_torque_peak=float(np.max(np.abs(a[m,6]))),feedback_torque_peak=float(np.max(np.abs(a[m,7]))),
            torque_difference_rms=float(np.sqrt(np.mean((a[m,6]-a[m,7])**2))),
            angle_error_rms_deg=float(np.degrees(np.sqrt(np.mean(a[m,8]**2)))),
            proportional_cascade_gain=float(abs(predicted)),proportional_cascade_lag_deg=float(-np.degrees(np.angle(predicted)))))
    return results


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv_paths',nargs='+',type=Path)
    p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rows=[r for path in args.csv_paths for r in analyze(path)]
    with (args.output/'frequency_response.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (args.output/'frequency_response.json').write_text(json.dumps(rows,indent=2)+'\n')
    for r in rows:
        print(r['source'],f"{r['frequency_low_hz']:.1f}-{r['frequency_high_hz']:.1f}",
              f"v_gain={r['velocity_gain']:.3f} lag={r['velocity_lag_deg']:.1f}deg",
              f"torque_gain={r['torque_gain']:.3f} lag={r['torque_lag_deg']:.1f}deg",
              f"P_cascade_gain={r['proportional_cascade_gain']:.3f} lag={r['proportional_cascade_lag_deg']:.1f}")

if __name__=='__main__':main()
