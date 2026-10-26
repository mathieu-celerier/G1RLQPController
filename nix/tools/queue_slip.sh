#!/usr/bin/env bash
# Slippery feet, Tilt + n4 (2026-10-26): Tilt FT (6) vs JointDR (3) at foot friction 0.5 and 0.4
# (0.3 fails within 2 s for both; ground truth at 0.3 lasts 68 s). Each run ends at the first fall/QP failure.
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
export DROP_BIN=1
rt() { if [ -s $1_test.csv ]; then echo "== $1"; python3 analyze_test.py $1_test.csv; else ./run_test.sh "$@"; fi; }
{
for p in tft jdr; do
  export CTL_BASE=run/ctl_override_${p}_tilt.yaml
  for mu in 0.5 0.4; do
    cp mj_g1_imu_n4_mu$mu.yaml $MJ
    rt slip${mu}_${p}_turn jdr_test_cmd_turn.yaml 60
    rt slip${mu}_${p}_cmd_sched jdr_test_cmd_sched.yaml 100
    for s in 1 2 3; do rt slip${mu}_${p}_push150_s$s test_push150_s$s.yaml 120; done
    echo "push150 mu$mu $p: $(python3 fall_rate.py slip${mu}_${p}_push150_s*_test.csv | tail -1)"
  done
done
for f in slip*.log; do echo "${f%.log}: $(grep -c 'QLD fail' "$f") QP failures"; done
echo QUEUE_SLIP_DONE
} > tests_slip.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
