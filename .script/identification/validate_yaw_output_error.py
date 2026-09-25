#!/usr/bin/env python3
"""Validate frozen yaw model parameters on enabled segments of another CSV."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from fit_yaw_output_error import simulate, metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv_path',type=Path)
    p.add_argument('--fit',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--excitation-only',action='store_true',help='Validate only state=2 sweep samples')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    fit=json.loads(args.fit.read_text())
    models={'sweep_first_half':[fit['J_kg_m2'],fit['b_Nm_s_rad']],
            'sweep_full':[fit['full_sweep_fit']['J_kg_m2'],fit['full_sweep_fit']['b_Nm_s_rad']]}
    fields=['/gimbal/sample_time_s','/gimbal/yaw/torque','/gimbal/yaw/velocity_imu',
            '/gimbal/yaw/control_torque','/gimbal/yaw/velocity']
    if args.excitation_only: fields.append('/gimbal/yaw/excitation/state')
    rows=[];skipped=0
    with args.csv_path.open() as f:
        for row in csv.DictReader(f):
            try: rows.append([float(row[k]) for k in fields])
            except (TypeError,ValueError): skipped+=1
    a=np.array(rows);valid=np.isfinite(a[:,:4]).all(axis=1)
    if args.excitation_only: valid &= a[:,-1] == 2
    # Split on enable transitions or acquisition gaps; no measured-state resets inside a segment.
    starts=np.flatnonzero(valid & np.r_[True,(~valid[:-1]) | (np.diff(a[:,0])>0.01)])
    ends=np.flatnonzero(valid & np.r_[(~valid[1:]) | (np.diff(a[:,0])>0.01),True])+1
    report={'source':str(args.csv_path),'sha256':hashlib.sha256(args.csv_path.read_bytes()).hexdigest(),
            'fit_source':str(args.fit),'models':models,'skipped_rows':skipped,'segments':[],
            'method':'Frozen parameters; continuous rollout initialized once per enabled segment; measured current-derived torque input. No refitting.'}
    all_actual=[];all_pred={k:[] for k in models}
    with (args.output/'prediction.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['segment','time_s','torque_Nm','measured_velocity_rad_s',*models])
        for n,(start,end) in enumerate(zip(starts,ends),1):
            b=a[start:end]
            if len(b)<2:continue
            t,u,w=b[:,:3].T
            if np.any(np.diff(t)<=0):raise ValueError('Non-increasing time')
            predictions={name:simulate(t,u,w[0],parameters) for name,parameters in models.items()}
            item={'segment':n,'samples':len(t),'start_s':float(t[0]),'end_s':float(t[-1]),
                  'actual_velocity_range_rad_s':[float(w.min()),float(w.max())],
                  'torque_range_Nm':[float(u.min()),float(u.max())],
                  'motor_imu_velocity_rmse_rad_s':float(np.sqrt(np.mean((b[:,4]-w)**2))),
                  'models':{name:metrics(w,prediction) for name,prediction in predictions.items()}}
            report['segments'].append(item);all_actual.append(w)
            for name, prediction in predictions.items():all_pred[name].append(prediction)
            writer.writerows(zip([n]*len(t),t,u,w,*predictions.values()))
    if not all_actual:raise ValueError('No usable enabled segments')
    report['aggregate']={name:metrics(np.concatenate(all_actual),np.concatenate(values)) for name,values in all_pred.items()}
    (args.output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
