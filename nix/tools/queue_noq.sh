#!/usr/bin/env bash
# ArmWrenchRamp-NoQacc model_14999 (policy index 5) through the battery used for model_17998:
# ground truth (stock models straight, matched model turns/schedule/150 N/300 N at CoM),
# Tilt observer with a perfect IMU, Tilt observer with 4x MEMS IMU noise (n4).
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export DROP_BIN=1  # raw logs are ~8.5 GB per 150 s run; keep only the converted CSVs
# Skip a test whose CSV already exists (resume after an interruption).
rt() { if [ -s $1_test.csv ]; then echo "== $1"; python3 analyze_test.py $1_test.csv; else ./run_test.sh "$@"; fi; }
{
export CTL_BASE=run/ctl_override_noq.yaml
cp ~/devel/mc_rtc_nix_ws/install/share/mc_mujoco/g1_29dof.yaml $MJ; rt noq_stock1ms_straight jdr_straight.yaml 150
cp mj_g1_rt.yaml $MJ; rt noq_stockrt_straight jdr_straight.yaml 150
cp mj_g1_mjlab.yaml $MJ
rt noq_mjlab_turn jdr_test_cmd_turn.yaml 60
rt noq_mjlab_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3; do rt noq_mjlab_push150_s$s test_push150_s$s.yaml 120; done
for s in 1 2 3 4 5 6; do rt noq_mjlab_push300_s$s test_push300_s$s.yaml 120; done
echo "push150 noq: $(python3 fall_rate.py noq_mjlab_push150_s*_test.csv | tail -1)"
echo "push300 noq: $(python3 fall_rate.py noq_mjlab_push300_s*_test.csv | tail -1)"
export CTL_BASE=run/ctl_override_noq_tilt.yaml
cp mj_g1_mjlab_imu.yaml $MJ
rt tilt_noq_turn jdr_test_cmd_turn.yaml 60
rt tilt_noq_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3 4 5 6; do rt tilt_noq_push300_s$s test_push300_s$s.yaml 120; done
echo "push300 tilt noq: $(python3 fall_rate.py tilt_noq_push300_s*_test.csv | tail -1)"
cp mj_g1_imu_n4.yaml $MJ
rt n4_noq_turn jdr_test_cmd_turn.yaml 60
rt n4_noq_cmd_sched jdr_test_cmd_sched.yaml 100
for s in 1 2 3 4 5 6; do
  sed "s/seed: 0}/seed: $s}/" mj_g1_imu_n4.yaml > $MJ
  rt n4_noq_push300_s$s test_push300_s$s.yaml 120
done
echo "push300 n4 noq: $(python3 fall_rate.py n4_noq_push300_s*_test.csv | tail -1)"
for f in noq_*.log tilt_noq_*.log n4_noq_*.log; do echo "${f%.log}: $(grep -c 'QLD fail' "$f") QP failures"; done
echo QUEUE_B0813_DONE
} > tests_noq.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
