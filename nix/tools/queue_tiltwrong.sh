#!/usr/bin/env bash
# Tilt FT (policy 6) vs JointDR 17998 (policy 3) where the estimate is wrong (2026-10-26), all with the
# real Tilt observer and 4x MEMS IMU noise (n4):
#  - vb*: constant bias on the policy's base_lin_vel observation (sim_test.obs_vel_bias, x, m/s);
#  - mu*: slippery feet (foot geoms friction mu with priority 1, as mjlab; the training DR is 0.3-1.2),
#    the anchor slips so Tilt's static-contact assumption is violated.
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export DROP_BIN=1
rt() { if [ -s $1_test.csv ]; then echo "== $1"; python3 analyze_test.py $1_test.csv; else ./run_test.sh "$@"; fi; }
{
for p in tft jdr; do
  export CTL_BASE=run/ctl_override_${p}_tilt.yaml
  cp mj_g1_imu_n4.yaml $MJ
  for b in 0.4 0.2; do
    rt vb${b}_${p}_straight vb${b}_jdr_straight.yaml 60
    rt vb${b}_${p}_turn vb${b}_jdr_test_cmd_turn.yaml 60
    rt vb${b}_${p}_cmd_sched vb${b}_jdr_test_cmd_sched.yaml 100
    for s in 1 2 3; do rt vb${b}_${p}_push300_s$s vb${b}_test_push300_s$s.yaml 120; done
    echo "push300 vb$b $p: $(python3 fall_rate.py vb${b}_${p}_push300_s*_test.csv | tail -1)"
  done
  for mu in 0.3 0.2; do
    cp mj_g1_imu_n4_mu$mu.yaml $MJ
    rt mu${mu}_${p}_straight jdr_straight.yaml 60
    rt mu${mu}_${p}_turn jdr_test_cmd_turn.yaml 60
    rt mu${mu}_${p}_cmd_sched jdr_test_cmd_sched.yaml 100
    for s in 1 2 3; do rt mu${mu}_${p}_push300_s$s test_push300_s$s.yaml 120; done
    echo "push300 mu$mu $p: $(python3 fall_rate.py mu${mu}_${p}_push300_s*_test.csv | tail -1)"
  done
done
for f in vb*_*.log mu*_*.log; do echo "${f%.log}: $(grep -c 'QLD fail' "$f") QP failures"; done
echo QUEUE_TILTWRONG_DONE
} > tests_tiltwrong.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
