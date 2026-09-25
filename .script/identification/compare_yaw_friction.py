#!/usr/bin/env python3
"""Compare linear, Coulomb and bias models with disjoint training/validation segments."""
import ctypes,csv,hashlib,json,subprocess,tempfile
from pathlib import Path
import numpy as np
from fit_yaw_output_error import metrics,simulate
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/yaw_identification/friction_comparison_2026-09-22'
OUT.mkdir(parents=True,exist_ok=True)
source=Path(__file__).with_name('yaw_friction_rollout.cpp')
libpath=Path(tempfile.mkdtemp(prefix='rmcs_friction_'))/'rollout.so'
subprocess.run(['g++','-O3','-shared','-fPIC',str(source),'-o',str(libpath)],check=True)
lib=ctypes.CDLL(str(libpath)); ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
lib.rollout.argtypes=[ctypes.c_int,ptr,ptr,ctypes.c_double,ptr,ctypes.c_double,ctypes.c_int,ptr]
lib.rollout.restype=None
EPS=.05

def roll(s,p,substeps=1):
 t,u,w=s[:,:3].T;t=np.ascontiguousarray(t);u=np.ascontiguousarray(u)
 out=np.empty(len(t));lib.rollout(len(t),t,u,w[0],np.ascontiguousarray(p,dtype=float),EPS,substeps,out)
 return out

def load(name,sweep):
 path=ROOT/'data'/name
 keys=['/gimbal/sample_time_s','/gimbal/yaw/torque','/gimbal/yaw/velocity_imu','/gimbal/yaw/control_torque','/gimbal/yaw/excitation/state']
 rows=[]
 with path.open() as f:
  for r in csv.DictReader(f):
   try:rows.append([float(r[k]) for k in keys])
   except (TypeError,ValueError):continue
 a=np.array(rows);valid=np.isfinite(a[:,:4]).all(axis=1)
 if sweep:valid &= a[:,4]==2
 starts=np.flatnonzero(valid&np.r_[True,~valid[:-1]|(np.diff(a[:,0])>.01)])
 ends=np.flatnonzero(valid&np.r_[~valid[1:]|(np.diff(a[:,0])>.01),True])+1
 return [a[i:j] for i,j in zip(starts,ends) if j-i>100]

def optimize(fun,start,bounds,maxiter=420):
 n=len(start);clip=lambda q:np.clip(q,bounds[:,0],bounds[:,1])
 x=np.array([start]+[clip(start+np.eye(n)[i]*.2) for i in range(n)])
 y=np.array([fun(q) for q in x]);converged=False
 for it in range(maxiter):
  order=np.argsort(y);x,y=x[order],y[order]
  if np.max(abs(x-x[0]))<2e-5:converged=True;break
  center=x[:-1].mean(axis=0);r=clip(2*center-x[-1]);fr=fun(r)
  if fr<y[0]:
   e=clip(center+2*(r-center));fe=fun(e);x[-1],y[-1]=(e,fe) if fe<fr else (r,fr)
  elif fr<y[-2]:x[-1],y[-1]=r,fr
  else:
   outside=fr<y[-1];c=clip(center+.5*((r if outside else x[-1])-center));fc=fun(c)
   if fc<(fr if outside else y[-1]):x[-1],y[-1]=c,fc
   else:
    for i in range(1,n+1):x[i]=clip(x[0]+.5*(x[i]-x[0]));y[i]=fun(x[i])
 i=int(np.argmin(y));return x[i],float(y[i]),converged,it+1

sweep1=load('yaw扫频测试_原代码_2026-09-22_09-00-27.csv',True)[0]
sweep2=load('yaw扫频测试_原代码_2026-09-22_09-01-51.csv',True)[0]
pitch5=load('yaw扫频抬头5度测试_原代码_2026-09-22_13-20-27.csv',True)[0]
manual=load('yaw手动验证测试_原代码_2026-09-22_08-34-53.csv',False)
segments={'train_sweep1':sweep1,'train_manual1':manual[0],
          'validation_manual2':manual[1],'validation_sweep2':sweep2,'validation_pitch5':pitch5}
train=[sweep1,manual[0]]
# Verify linear numerical path and step-refinement error before fitting.
p0=np.array([.158,.886,0.,0.]);assert np.max(abs(roll(sweep1,p0)-simulate(sweep1[:,0],sweep1[:,1],sweep1[0,2],p0[:2])))<1e-8
report={'model':'J*w_dot+b*w+Fc*tanh(w/eps)+bias=feedback_torque','epsilon_rad_s':EPS,
 'training':['train_sweep1','train_manual1'],'validation':['validation_manual2','validation_sweep2','validation_pitch5'],
 'training_objective':'equal weight per segment, mean velocity squared error; continuous rollout',
 'models':{}}
for name,n in [('linear',2),('friction',3),('friction_bias',4)]:
 bounds=np.array([[np.log(.005),np.log(3)],[np.log(.0001),np.log(20)],[0,4],[-3,3]])[:n]
 def decode(q):
  p=np.zeros(4);p[:2]=np.exp(q[:2]);p[2:n]=q[2:];return p
 def objective(q):
  p=decode(q);return float(np.mean([np.mean((roll(s,p)-s[:,2])**2) for s in train]))
 starts=[np.array([np.log(j),np.log(b),fc,bias])[:n] for j,b,fc,bias in [(.16,.8,.2,0),(.16,.15,.7,0),(.3,.3,.5,-.2)]]
 runs=[optimize(objective,q,bounds) for q in starts]
 q,loss,ok,it=min(runs,key=lambda r:r[1]);p=decode(q)
 result={'J':float(p[0]),'b':float(p[1]),'Fc':float(p[2]),'bias':float(p[3]),'loss':loss,'converged':ok,'iterations':it,
 'bounds':bounds.tolist(),'at_boundary':bool(np.any(np.minimum(q-bounds[:,0],bounds[:,1]-q)<1e-3)),
 'multistart':[{'parameters':decode(r[0]).tolist(),'loss':r[1],'converged':r[2]} for r in runs], 'segments':{}}
 for label,s in segments.items():
  pred=roll(s,p);refined=roll(s,p,2)
  result['segments'][label]={**metrics(s[:,2],pred),'step_refinement_rmse':float(np.sqrt(np.mean((pred-refined)**2)))}
  with (OUT/f'{name}_{label}.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['time_s','torque_Nm','velocity_rad_s','predicted_rad_s']);w.writerows(zip(s[:,0],s[:,1],s[:,2],pred))
 report['models'][name]=result
 (OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
 print(name,json.dumps(result),flush=True)
# Classifications use smoothed measurements only for diagnostics, never to reset predictions.
old=np.array([.158191719,.886434043,0,0]);diag=[]
for number,s in enumerate(manual,1):
 t,u,v=s[:,:3].T;dt=float(np.median(np.diff(t)));window=max(3,int(round(.1/dt))|1)
 smooth=np.convolve(np.pad(v,window//2,mode='edge'),np.ones(window)/window,mode='valid')
 acc=np.gradient(smooth,t);reversal=np.zeros(len(t),dtype=bool)
 crossings=np.flatnonzero((smooth[:-1]*smooth[1:]<0))
 for k in crossings:
  lo=max(0,k-int(.25/dt));hi=min(len(t),k+int(.25/dt))
  if smooth[lo:hi].min()<-.15 and smooth[lo:hi].max()>.15:reversal[lo:hi]=True
 labels=np.full(len(t),'acceleration',dtype=object)
 labels[np.abs(smooth)<.2]='low_speed';labels[(abs(smooth)>=.2)&(abs(acc)<.5)]='steady';labels[reversal]='reversal'
 models={'old_sweep_model':old,**{n:np.array([r[k] for k in ['J','b','Fc','bias']]) for n,r in report['models'].items()}}
 for name,p in models.items():
  error=roll(s,p)-v
  for label in ['steady','acceleration','reversal','low_speed']:
   m=labels==label
   if m.any():diag.append({'manual_segment':number,'model':name,'motion':label,'samples':int(m.sum()),'rmse':float(np.sqrt(np.mean(error[m]**2))),'bias':float(error[m].mean())})
with (OUT/'residual_by_motion.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(diag[0]));w.writeheader();w.writerows(diag)
print('residual diagnostics',json.dumps(diag),flush=True)
report['source_files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [
 ROOT/'data/yaw扫频测试_原代码_2026-09-22_09-00-27.csv',
 ROOT/'data/yaw扫频测试_原代码_2026-09-22_09-01-51.csv',
 ROOT/'data/yaw扫频抬头5度测试_原代码_2026-09-22_13-20-27.csv',
 ROOT/'data/yaw手动验证测试_原代码_2026-09-22_08-34-53.csv']}
report['segment_times']={label:[float(s[0,0]),float(s[-1,0]),len(s)] for label,s in segments.items()}
report['classification']={'smoothing_s':.1,'steady_acceleration_threshold_rad_s2':.5,
                         'low_speed_threshold_rad_s':.2,'reversal_half_window_s':.25}
(OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
