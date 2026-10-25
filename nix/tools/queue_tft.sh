#!/usr/bin/env bash
# ArmWrenchRamp-Tilt fine-tune model_17998 (policy index 6, 2026-10-26) through the battery used for model_17998:
# ground truth (stock models straight, matched model turns/schedule/150 N/300 N at CoM),
# Tilt observer with a perfect IMU, Tilt observer with 4x MEMS IMU noise (n4).
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export DROP_BIN=1  # raw logs are ~8.5 GB per 150 s run; keep only the converted CSVs
# Skip a test whose CSV already exists (resume after an interruption).
rt() { if [ -s $1_test.csv ]; then echo "== $1"; python3 analyze_test.py $1_test.csv; else ./run_test.sh "$@"; fi; }
{
export CTL_BASE=run/ctl_override_tft.yaml
cp ~/devel/mc_rtc_nix_ws/install/share/mc_mujoco/g1_29dof.yaml $MJ; rt tft_stock1ms_straight jdr_straight.yaml 150
cp mj_g1_rt.yaml $MJ; rt tft_stockrt_straight jdr_straight.yaml 150
cp mj_g1_mjlab.yaml $MJ
rt tft_mjlab_turn jdr_test_cmd_turn.yaml 60
rt tft_mjlab_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3; do rt tft_mjlab_push150_s$s test_push150_s$s.yaml 120; done
for s in 1 2 3 4 5 6; do rt tft_mjlab_push300_s$s test_push300_s$s.yaml 120; done
echo "push150 tft: $(python3 fall_rate.py tft_mjlab_push150_s*_test.csv | tail -1)"
echo "push300 tft: $(python3 fall_rate.py tft_mjlab_push300_s*_test.csv | tail -1)"
export CTL_BASE=run/ctl_override_tft_tilt.yaml
cp mj_g1_mjlab_imu.yaml $MJ
rt tilt_tft_turn jdr_test_cmd_turn.yaml 60
rt tilt_tft_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3 4 5 6; do rt tilt_tft_push300_s$s test_push300_s$s.yaml 120; done
echo "push300 tilt tft: $(python3 fall_rate.py tilt_tft_push300_s*_test.csv | tail -1)"
cp mj_g1_imu_n4.yaml $MJ
rt n4_tft_turn jdr_test_cmd_turn.yaml 60
rt n4_tft_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3 4 5 6; do
  sed "s/seed: 0}/seed: $s}/" mj_g1_imu_n4.yaml > $MJ
  rt n4_tft_push300_s$s test_push300_s$s.yaml 120
done
echo "push300 n4 tft: $(python3 fall_rate.py n4_tft_push300_s*_test.csv | tail -1)"
for f in tft_*.log tilt_tft_*.log n4_tft_*.log; do echo "${f%.log}: $(grep -c 'QLD fail' "$f") QP failures"; done
echo QUEUE_TFT_DONE
} > tests_tft.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
