#!/usr/bin/env python3
"""Reproducible three-model benchmark. Writes only to --output (default /tmp).
Requires numpy, scipy, matplotlib, and g++. Never connects to the robot.
"""
import argparse,csv,ctypes,hashlib,json,subprocess,tempfile
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.signal import lfilter

ROOT=Path(__file__).resolve().parents[2]
DT=.01
WARMUP=20 # common 0.2 s omitted from metrics after each segment initialization
EPS=.05


def metrics(actual,predicted):
    mse=np.mean((actual-predicted)**2);variance=np.var(actual)
    return {'rmse_rad_s':float(np.sqrt(mse)), 'r2':float(1-mse/variance) if variance>1e-12 else None,
            'bias_rad_s':float(np.mean(predicted-actual)),'samples':len(actual)}


def native():
    directory=Path(tempfile.mkdtemp(prefix='rmcs_three_models_'))
    cpp=directory/'blackbox.cpp'
    cpp.write_text('''#include <cmath>
extern "C" void blackbox(int n,const double* u,const double* y,int na,int nb,int delay,
                         const double* c,double* out) {
 int warm=na>nb+delay?na:nb+delay;
 for(int k=0;k<n;++k){
  if(k<warm){out[k]=y[k];continue;}
  double v=c[na+nb];
  for(int j=0;j<na;++j)v+=c[j]*out[k-1-j];
  for(int j=0;j<nb;++j)v+=c[na+j]*u[k-1-delay-j];
  out[k]=v;
 }
}
''')
    lib=directory/'rollout.so'
    subprocess.run(['g++','-O3','-shared','-fPIC',str(cpp),str(Path(__file__).with_name('yaw_friction_rollout.cpp')),'-o',str(lib)],check=True)
    dll=ctypes.CDLL(str(lib));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
    dll.rollout.argtypes=[ctypes.c_int,ptr,ptr,ctypes.c_double,ptr,ctypes.c_double,ctypes.c_int,ptr];dll.rollout.restype=None
    dll.blackbox.argtypes=[ctypes.c_int,ptr,ptr,ctypes.c_int,ctypes.c_int,ctypes.c_int,ptr,ptr];dll.blackbox.restype=None
    return dll


def load(path):
    prefix='/gimbal/yaw/excitation/'
    fields=['/gimbal/sample_time_s','/gimbal/yaw/torque','/gimbal/yaw/velocity_imu',
            '/gimbal/yaw/control_torque',prefix+'mode',prefix+'stage',prefix+'session_id',prefix+'pitch_actual_up_deg']
    rows=[];bad=0
    with path.open() as f:
        for row in csv.DictReader(f):
            try:rows.append([float(row[k]) for k in fields])
            except (ValueError,TypeError):bad+=1
    a=np.array(rows);valid=np.isfinite(a[:,:4]).all(axis=1)
    standard='标准测试' in path.name
    if standard:valid &= (a[:,4]==1)&np.isin(a[:,5],[3,4,5,6])
    else:valid &= (a[:,5]==9)|((a[:,4]==0)&(a[:,5]==0))
    dt=np.diff(a[:,0])
    changes=(dt>.0025)|(dt<=0)|np.any(a[1:,4:7]!=a[:-1,4:7],axis=1)
    starts=np.flatnonzero(valid&np.r_[True,(~valid[:-1])|changes]);ends=np.flatnonzero(valid&np.r_[(~valid[1:])|changes,True])+1
    segments=[];discarded=0
    for index,(i,j) in enumerate(zip(starts,ends)):
        b=a[i:j];t,u,y=b[:,:3].T
        if t[-1]-t[0]<2:discarded+=len(b);continue
        # Anti-alias speed with a causal one-pole filter; same kernel on torque.
        # Cutoff 20 Hz, above the 5 Hz excitation; common filter phase cancels
        # for a linear plant. Nonlinear filtering approximation is documented.
        alpha=np.exp(-2*np.pi*20*np.median(np.diff(t)))
        uf=lfilter([1-alpha],[1,-alpha],u,zi=[alpha*u[0]])[0]
        yf=lfilter([1-alpha],[1,-alpha],y,zi=[alpha*y[0]])[0]
        grid=np.arange(t[0],t[-1]-DT/2,DT)
        # Input is mean filtered torque over the next grid interval (ZOH equivalent).
        cumulative=np.r_[0,np.cumsum((uf[1:]+uf[:-1])*.5*np.diff(t))]
        integrated=np.interp(grid,t,cumulative)
        ug=np.diff(integrated)/DT
        ug=np.r_[ug,ug[-1]]
        yg=np.interp(grid,t,yf)
        segments.append({'file':path.name,'stage':int(b[0,5]),'chunk':index,
                         't':np.ascontiguousarray(grid),'u':np.ascontiguousarray(ug),'y':np.ascontiguousarray(yg),
                         'pitch':np.interp(grid,t,b[:,7])})
    quality={'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
             'raw_rows':len(a),'incomplete_rows':bad,'gap_count_over_2_5ms':int(np.sum(dt>.0025)),
             'discarded_short_segment_rows':discarded,'usable_segments':len(segments)}
    return segments,quality


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/tmp/rmcs_three_models_20260924'))
    args=parser.parse_args();out=args.output.resolve()
    if out.is_relative_to(ROOT/'docs'):raise ValueError('docs output prohibited without explicit request')
    out.mkdir(parents=True,exist_ok=True)
    lib=native()
    standard=sorted((ROOT/'data').glob('yaw标准测试_*.csv'))
    manual=sorted((ROOT/'data').glob('yaw手动*.csv'))
    if len(standard)!=5:raise ValueError('Expected frozen five standard-test files')
    segments=[];quality=[]
    for path in standard+manual:
        ss,q=load(path);quality.append(q)
        for s in ss:
            if path in standard:
                run=standard.index(path)+1
                s['group']='mixed_validation' if s['stage']==6 else 'train' if run<=3 else 'standard_validation'
                s['run']=run
            else:s['group']='manual_fixed' if '固定pitch' in path.name and '非固定' not in path.name else 'manual_variable';s['run']=0
            segments.append(s)
    train=[s for s in segments if s['group']=='train'];innertrain=[s for s in train if s['run']<=2];innercheck=[s for s in train if s['run']==3]
    print('Data ready',[(q['file'],q['usable_segments']) for q in quality],flush=True)
    def gray(s,p,steps=5):
        pred=np.empty(len(s['t']));lib.rollout(len(pred),s['t'],s['u'],float(s['y'][0]),np.ascontiguousarray(p),EPS,steps,pred);return pred
    def bb(s,c,na,nb,delay):
        pred=np.empty(len(s['t']));lib.blackbox(len(pred),s['u'],s['y'],na,nb,delay,np.ascontiguousarray(c),pred);return pred
    def loss_on(ss,simulate):
        total=0.;count=0
        for s in ss:
            residual=simulate(s)[WARMUP:]-s['y'][WARMUP:]
            if not np.isfinite(residual).all() or np.max(abs(residual))>1e6:return 1e12
            total+=float(residual@residual);count+=len(residual)
        return total/count
    report={'data_quality':quality,'dt_s':DT,'warmup_s':WARMUP*DT,
            'preprocess':'split active mode/stage/session and >2.5ms gaps; discard <2s chunks; 20Hz causal lowpass on torque and velocity; 100Hz grid with interval-mean torque',
            'split':{'train':[p.name for p in standard[:3]],'standard_validation':[p.name for p in standard[3:]],
                     'reserved':'stage6 mixed trajectories of every run; both manual files are validation only'},
            'input':'motor current-derived feedback torque','output':'IMU body-z yaw rate; no pitch correction or pitch input',
            'models':{}}
    predictors={}
    for name,n in [('simple_second_order',2),('extended_second_order',4)]:
        bounds=[(np.log(.005),np.log(3)),(np.log(.0001),np.log(20)),(0,4),(-3,3)][:n]
        def decode(q):
            p=np.zeros(4);p[:2]=np.exp(q[:2]);p[2:n]=q[2:];return p
        objective=lambda q:loss_on(train,lambda s:gray(s,decode(q)))
        seeds=[np.array([np.log(j),np.log(b),fc,bias])[:n] for j,b,fc,bias in [(.16,.8,0,0),(.16,.05,.7,-.03),(.3,.3,.4,0)]]
        runs=[minimize(objective,seed,method='Nelder-Mead',bounds=bounds,options={'maxiter':500,'xatol':2e-5,'fatol':1e-8}) for seed in seeds]
        best=min(runs,key=lambda r:r.fun);p=decode(best.x)
        report['models'][name]={'parameters':dict(zip(['J','b','Fc','bias'],p.tolist())),
            'epsilon':EPS if n==4 else None,'train_mse':float(best.fun),'success':bool(best.success),
            'bounds':bounds,'multistart':[{'mse':float(r.fun),'parameters':decode(r.x).tolist(),'success':bool(r.success)} for r in runs]}
        predictors[name]=lambda s,p=p:gray(s,p)
        print(name,report['models'][name],flush=True)
        (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    # Discrete black box: low-order ARX starts; selection uses only run3,
    # then output-error refinement + all first-three runs for final parameters.
    def design(ss,na,nb,delay):
        xs=[];ys=[]
        for s in ss:
            warm=max(na,nb+delay,WARMUP);y=s['y'];u=s['u'];N=len(y)
            x=np.column_stack([y[warm-1-i:N-1-i] for i in range(na)]+[u[warm-1-delay-i:N-1-delay-i] for i in range(nb)]+[np.ones(N-warm)])
            xs.append(x);ys.append(y[warm:])
        return np.vstack(xs),np.concatenate(ys)
    def stable(c,na):return np.max(np.abs(np.roots(np.r_[1,-c[:na]])))<.99999
    candidates=[]
    for na in [1,2]:
        for nb in [1,2,4]:
            for delay in [0,1,3,5]:
                X,Y=design(innertrain,na,nb,delay);scale=np.sqrt(np.mean(X*X,axis=0));scale=np.maximum(scale,1e-8);Xs=X/scale
                for ridge in [1e-8,1e-4]:
                    c=np.linalg.solve(Xs.T@Xs+ridge*len(Y)*np.eye(X.shape[1]),Xs.T@Y)/scale
                    if not stable(c,na):continue
                    score=loss_on(innercheck,lambda s:bb(s,c,na,nb,delay))
                    candidates.append({'na':na,'nb':nb,'delay':delay,'ridge':ridge,'c':c,'selection_mse':score})
    if not candidates:raise RuntimeError('No stable black-box candidate')
    candidates.sort(key=lambda x:x['selection_mse'])
    def to_q(c,na):
        if na==1:return np.r_[c[0],c[na:]]
        return np.r_[c[0]/(1-c[1]),c[1],c[na:]]
    def from_q(q,na):
        return np.r_[q[0]*(1-q[1]),q[1],q[na:]] if na==2 else q.copy()
    # Schur stability for AR(2): a2=k2, a1=k1*(1-k2), |ki|<1.
    refined=[]
    for candidate in candidates[:3]:
        na,nb,delay=[candidate[k] for k in ['na','nb','delay']]
        bounds=[(-.9999,.9999)]*na+[(-20,20)]*nb+[(-3,3)]
        f=lambda q:loss_on(innertrain,lambda s:bb(s,from_q(q,na),na,nb,delay))
        r=minimize(f,to_q(candidate['c'],na),method='L-BFGS-B',bounds=bounds,options={'maxiter':250,'ftol':1e-11})
        c=from_q(r.x,na);score=loss_on(innercheck,lambda s:bb(s,c,na,nb,delay))
        refined.append({**candidate,'c':c,'selection_mse':score,'refine_success':bool(r.success)})
    chosen=min(refined,key=lambda x:x['selection_mse']);na,nb,delay=[chosen[k] for k in ['na','nb','delay']]
    bounds=[(-.9999,.9999)]*na+[(-20,20)]*nb+[(-3,3)]
    f=lambda q:loss_on(train,lambda s:bb(s,from_q(q,na),na,nb,delay))
    best=minimize(f,to_q(chosen['c'],na),method='L-BFGS-B',bounds=bounds,options={'maxiter':400,'ftol':1e-12})
    c=from_q(best.x,na)
    report['models']['linear_black_box']={'na':na,'nb':nb,'delay_samples':delay,'dt_s':DT,'coefficients':c.tolist(),
        'equation':'y[k]=sum(a_i*y[k-i])+sum(b_j*u[k-1-delay-j])+constant',
        'poles':[[float(z.real),float(z.imag)] for z in np.roots(np.r_[1,-c[:na]])],
        'train_mse':float(best.fun),'success':bool(best.success),'selection_mse':float(chosen['selection_mse']),
        'candidate_count':len(candidates),'selection':'fit runs1-2, choose structure using run3 recursive error, refit runs1-3',
        'candidate_results':[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in x.items()} for x in refined]}
    predictors['linear_black_box']=lambda s:bb(s,c,na,nb,delay)
    print('linear_black_box',report['models']['linear_black_box'],flush=True)
    # Numerical consistency checks: gray linear limit and nonlinear step refinement.
    test=train[0];J,b=.18,.7;constant={**test,'u':np.ones(len(test['u'])),'y':np.zeros(len(test['y']))}
    linear=gray(constant,np.array([J,b,0.,0.]));analytic=(1-np.exp(-b*(test['t']-test['t'][0])/J))/b
    assert np.max(abs(linear-analytic))<1e-9
    extended=np.array(list(report['models']['extended_second_order']['parameters'].values()))
    report['numerical_checks']={'linear_analytic_max_error':float(np.max(abs(linear-analytic))),
       'extended_refinement_max_segment_rmse':float(max(np.sqrt(np.mean((gray(s,extended,5)-gray(s,extended,10))**2)) for s in segments))}
    grouped={};by_file={};by_stage={};prediction_fields=['file','group','stage','chunk','time_s','pitch_deg','torque_Nm','measured_rad_s',*predictors]
    with (out/'predictions.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(prediction_fields)
        for s in segments:
            pred={name:predict(s) for name,predict in predictors.items()}
            for name,yhat in pred.items():
                pair=(s['y'][WARMUP:],yhat[WARMUP:])
                grouped.setdefault((s['group'],name),[]).append(pair)
                by_file.setdefault((s['file'],s['group'],name),[]).append(pair)
                by_stage.setdefault((s['group'],s['stage'],name),[]).append(pair)
            for i in range(WARMUP,len(s['t'])):
                writer.writerow([s['file'],s['group'],s['stage'],s['chunk'],s['t'][i],s['pitch'][i],s['u'][i],s['y'][i],*[v[i] for v in pred.values()]])
    aggregate=lambda pairs:metrics(np.concatenate([p[0] for p in pairs]),np.concatenate([p[1] for p in pairs]))
    report['metrics']={g:{name:aggregate(pairs) for (group,name),pairs in grouped.items() if group==g} for g in sorted({k[0] for k in grouped})}
    report['per_file']=[{'file':file,'group':group,'model':name,**aggregate(pairs)} for (file,group,name),pairs in by_file.items()]
    report['per_stage']=[{'group':group,'stage':stage,'model':name,**aggregate(pairs)} for (group,stage,name),pairs in by_stage.items()]
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    with (out/'metrics.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['group','model','rmse_rad_s','r2','bias_rad_s','samples'])
        for group,models in report['metrics'].items():
            for name,m in models.items():writer.writerow([group,name,*m.values()])
    print('FINAL',json.dumps(report['metrics']),flush=True)

if __name__=='__main__':main()
