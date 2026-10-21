#!/usr/bin/env bash
# model_17998 with IMU-based base estimation (Tilt observer + knee-torque anchor) on the matched model with IMU sensors.
cd ~/devel/mc_rtc_nix_ws/run
cp mj_g1_mjlab_imu.yaml ~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export CTL_BASE=run/ctl_override_jdr_tilt.yaml
{
./run_test.sh tilt_jdr_turn jdr_test_cmd_turn.yaml 60
./run_test.sh tilt_jdr_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3; do ./run_test.sh tilt_jdr_push150_s$s test_push150_s$s.yaml 120; done
for s in 1 2 3 4 5 6; do ./run_test.sh tilt_jdr_push300_s$s test_push300_s$s.yaml 120; done
echo "push150 tilt: $(python3 fall_rate.py tilt_jdr_push150_s*_test.csv | tail -1)"
echo "push300 tilt: $(python3 fall_rate.py tilt_jdr_push300_s*_test.csv | tail -1)"
echo QUEUE_TILT_DONE
} > tests_tilt.txt 2>&1
cp mj_g1_mjlab.yaml ~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
