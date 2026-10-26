#!/usr/bin/env bash
# Seed 43 in mc_mujoco (2026-10-26): ArmWrenchRamp s43 model_14999 (index 7, velocity-bias-fragile in mjlab:
# 9.98 falls/min at 0.4 m/s) vs its Tilt fine-tune s43 model_17998 (index 8, 0 in mjlab).
# Tilt observer + 4x IMU noise; policy base_lin_vel biased by 0 / 0.2 / 0.4 m/s (sim_test.obs_vel_bias).
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export DROP_BIN=1
rt() { if [ -s $1_test.csv ]; then echo "== $1"; python3 analyze_test.py $1_test.csv; else ./run_test.sh "$@"; fi; }
{
cp mj_g1_imu_n4.yaml $MJ
for p in s43 tft_s43; do
  export CTL_BASE=run/ctl_override_${p}_tilt.yaml
  rt b0_${p}_straight jdr_straight.yaml 60
  rt b0_${p}_turn jdr_test_cmd_turn.yaml 60
  rt b0_${p}_cmd_sched jdr_test_cmd_sched.yaml 100
  for b in 0.2 0.4; do
    rt vb${b}_${p}_straight vb${b}_jdr_straight.yaml 60
    rt vb${b}_${p}_turn vb${b}_jdr_test_cmd_turn.yaml 60
    rt vb${b}_${p}_cmd_sched vb${b}_jdr_test_cmd_sched.yaml 100
    for s in 1 2 3; do rt vb${b}_${p}_push300_s$s vb${b}_test_push300_s$s.yaml 120; done
  done
done
for f in b0_s43_*.log b0_tft_s43_*.log vb*_s43_*.log vb*_tft_s43_*.log; do echo "${f%.log}: $(grep -c 'QLD fail' "$f") QP failures"; done
echo QUEUE_S43BIAS_DONE
} > tests_s43bias.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
