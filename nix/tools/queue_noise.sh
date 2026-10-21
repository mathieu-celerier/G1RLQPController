#!/usr/bin/env bash
# model_17998, Tilt-observer pipeline, IMU noise/bias in mc_mujoco: n1 = nominal MEMS (gyro 0.005 rad/s,
# acc 0.03 m/s^2, bias std 0.005 / 0.05), n4 = 4x. Bias seed = push seed.
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export CTL_BASE=run/ctl_override_jdr_tilt.yaml
{
for lvl in n1 n4; do
  cp mj_g1_imu_$lvl.yaml $MJ
  ./run_test.sh ${lvl}_jdr_turn jdr_test_cmd_turn.yaml 60
  ./run_test.sh ${lvl}_jdr_cmd_sched jdr_test_cmd_sched.yaml 100
  for s in 1 2 3 4 5 6; do
    sed "s/seed: 0}/seed: $s}/" mj_g1_imu_$lvl.yaml > $MJ
    ./run_test.sh ${lvl}_jdr_push300_s$s test_push300_s$s.yaml 120
  done
  echo "push300 $lvl: $(python3 fall_rate.py ${lvl}_jdr_push300_s*_test.csv | tail -1)"
done
echo QUEUE_NOISE_DONE
} > tests_noise.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
