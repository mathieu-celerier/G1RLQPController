"""Per-segment command tracking, pushes and falls of a sim_test run (CSV from mc_bin_utils)."""
import csv, sys, numpy as np
r=csv.reader(open(sys.argv[1]),delimiter=';'); h=next(r); X=np.array([[float(v) if v else np.nan for v in row] for row in r])
c=lambda n: X[:,h.index(n)]
T=c('t'); z=c('FloatingBase_position_z'); x=c('FloatingBase_position_x'); y=c('FloatingBase_position_y')
qw,qx,qy,qz=[c('FloatingBase_orientation_'+k) for k in 'wxyz']
qx,qy,qz=-qx,-qy,-qz  # mc_rtc logs the world-to-body (sva) orientation: conjugate it
yaw=np.unwrap(np.arctan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2)))
vxw,vyw=np.gradient(x,T),np.gradient(y,T); vb=np.cos(yaw)*vxw+np.sin(yaw)*vyw; vl=-np.sin(yaw)*vxw+np.cos(yaw)*vyw; wz=np.gradient(yaw,T)
cmd=np.stack([c(f'G1RLQPController_velCmd_{i}') for i in 'xyz'],1)
fall=T[np.argmax(z<0.5)] if (z<0.5).any() else None
print(f"sim {T[-1]:.1f}s  fall: {('%.1fs' % fall) if fall else 'none'}")
pf=np.linalg.norm(np.stack([c(f'G1RLQPController_pushForce_{i}') for i in 'xyz'],1),axis=1)
starts=np.where((pf[1:]>0)&(pf[:-1]==0))[0]+1
if len(starts): print(f"pushes: {len(starts)}, peak force {pf.max():.0f} N, impulses (N.s) median {np.median([pf[s:s+200][pf[s:s+200]>0].sum()*0.0025 for s in starts]):.0f}")
ch=np.where(np.any(np.diff(cmd,axis=0)!=0,axis=1))[0]+1; edges=[0,*ch,len(T)]
print(" segment        cmd (vx vy wz)        measured (last 4 s)")
for a,b in zip(edges[:-1],edges[1:]):
  m=(T>=max(T[a],T[b-1]-4))&(T<=T[b-1])&(T<(fall or 1e9))
  if m.sum()<10: continue
  print(f" {T[a]:5.1f}-{T[b-1]:5.1f}  {cmd[a,0]:+.2f} {cmd[a,1]:+.2f} {cmd[a,2]:+.2f}   {vb[m].mean():+.2f} {vl[m].mean():+.2f} {wz[m].mean():+.2f}")
