import csv, sys, numpy as np
f=sys.argv[1]; t0,t1=float(sys.argv[2]),float(sys.argv[3])
r=csv.reader(open(f),delimiter=';'); h=next(r); X=np.array([[float(v) if v else np.nan for v in row] for row in r])
def C(p,suf=None):
  idx=[(i,k) for i,k in enumerate(h) if k.startswith(p+'_')]
  if suf: return X[:,[h.index(p+'_'+s) for s in suf]]
  idx.sort(key=lambda x:int(x[1].rsplit('_',1)[1])); return X[:,[i for i,_ in idx]]
T=X[:,h.index('t')]; dt=0.0025
J=["lhp","lhr","lhy","lk","lap","lar","rhp","rhr","rhy","rk","rap","rar","wy","wr","wp","lsp","lsr","lsy","le","lwr","lwp","lwy","rsp","rsr","rsy","re","rwr","rwp","rwy"]
ref=C('G1RLQPController_qddRefFull')[:,6:]; out=C('alphaDOut'); vin=C('alphaIn')
real=np.gradient(vin,dt,axis=0)
fo=C('ffOut_acceleration',['wx','wy','wz','vx','vy','vz']); bid=C('G1RLQPController_qddBaseID')
fv=C('ffIn_velocity',['wx','wy','wz','vx','vy','vz']); fr=np.gradient(fv,dt,axis=0)
m=(T>t0)&(T<t1)
def rms(a): return np.sqrt(np.nanmean(a[m]**2,0))
print("joint | rms ref | rms QP-ref | corr(ref,QP) | rms real-QP | mean(QP-ref)")
for j in range(29):
  a,b,c=ref[m,j],out[m,j],real[m,j]
  print(f"{J[j]:4s} {rms(ref)[j]:7.1f} {np.sqrt(np.mean((b-a)**2)):7.1f} {np.corrcoef(a,b)[0,1]:6.2f} {np.sqrt(np.mean((c-b)**2)):7.1f} {np.mean(b-a):7.2f}")
e=out[m]-ref[m]; print("all joints: rms(QP-ref)/rms(ref) =", round(np.sqrt((e**2).mean())/np.sqrt((ref[m]**2).mean()),3))
names=['wx','wy','wz','vx','vy','vz']
print("base | mean QP | mean ID | rms(QP-ID) | rms ID | mean real(dv/dt)")
for k in range(6):
  print(f"{names[k]} {np.nanmean(fo[m,k]):8.2f} {np.nanmean(bid[m,k]):8.2f} {np.sqrt(np.nanmean((fo[m,k]-bid[m,k])**2)):8.2f} {rms(bid)[k]:8.2f} {np.nanmean(fr[m,k]):8.3f}")
tau=C('tauOut'); print("max |tau| per leg joint:", np.nanmax(np.abs(tau[m][:,:12]),0).round(0))
