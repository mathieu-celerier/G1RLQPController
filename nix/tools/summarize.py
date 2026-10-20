import csv, math, sys, numpy as np
r=list(csv.DictReader(open(sys.argv[1]),delimiter=';'))
T=np.array([float(d['t']) for d in r]); z=np.array([float(d['FloatingBase_position_z']) for d in r])
x=np.array([float(d['FloatingBase_position_x']) for d in r]); y=np.array([float(d['FloatingBase_position_y']) for d in r])
q=np.array([[float(d['FloatingBase_orientation_'+c]) for c in 'wxyz'] for d in r])
q[:,1:]*=-1  # mc_rtc logs the world-to-body (sva) orientation: conjugate it
yaw=np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))
fall=T[np.argmax(z<0.5)] if (z<0.5).any() else None
end=fall if fall else T[-1]; m=(T>2)&(T<min(end-1,20))
sp=np.hypot(np.gradient(x,T),np.gradient(y,T))
print(f"sim {T[-1]:.1f}s  fall {('%.1fs'%fall) if fall else 'none'}  speed {sp[m].mean():.2f} m/s  yaw rate {math.degrees(np.polyfit(T[m],yaw[m],1)[0]):+.2f} deg/s  yaw@20s {math.degrees(yaw[np.argmin(abs(T-20))]):+.0f}")
for j,n in ((4,'L ankle pitch'),(10,'R ankle pitch')):
  dd=np.array([float(d[f'alphaDOut_{j}'])-float(d[f'G1RLQPController_qddRefFull_{j+6}']) for d in r])[m]
  print(f"  {n}: QP deviates {100*(abs(dd)>1).mean():.1f}% of steps, rms {np.sqrt((dd**2).mean()):.1f}")
