"""Pooled fall rate over sim_test runs: falls / (sim time until fall or end), per minute."""
import csv, sys, numpy as np
falls=0; total=0.0
for f in sys.argv[1:]:
  r=csv.reader(open(f),delimiter=';'); h=next(r); X=np.array([[float(v) if v else np.nan for v in row] for row in r])
  T=X[:,h.index('t')]; z=X[:,h.index('FloatingBase_position_z')]
  end=T[np.argmax(z<0.5)] if (z<0.5).any() else T[-1]
  falls+=int((z<0.5).any()); total+=end
  print(f"  {f}: {'fall at %.1f s' % end if (z<0.5).any() else 'no fall in %.1f s' % end}")
print(f"falls {falls} in {total:.0f} s -> {60*falls/total:.2f} per minute")
