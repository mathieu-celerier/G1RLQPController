#!/usr/bin/env bash
# After the mc_mujoco FloatingBase angular-velocity frame fix (2026-10-21): model_17998 (jdr) and
# model_14999 (arm_ramp) on the stock models (straight 0.5) and the matched model (turns, schedule, pushes).
cd ~/devel/mc_rtc_nix_ws/run
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
battery() {  # $1 tag, $2 test-file prefix ("jdr_test_" or "test_"), $3 straight yaml
  cp ~/devel/mc_rtc_nix_ws/install/share/mc_mujoco/g1_29dof.yaml $MJ; ./run_test.sh fix_$1_stock1ms_straight $3 150
  cp mj_g1_rt.yaml $MJ; ./run_test.sh fix_$1_stockrt_straight $3 150
  cp mj_g1_mjlab.yaml $MJ
  ./run_test.sh fix_$1_mjlab_turn jdr_test_cmd_turn.yaml 60
  ./run_test.sh fix_$1_mjlab_cmd_sched jdr_test_cmd_sched.yaml 100
  for s in 1 2 3; do ./run_test.sh fix_$1_mjlab_push150_s$s ${2}push150_s$s.yaml 120; done
  for s in 1 2 3; do ./run_test.sh fix_$1_mjlab_push300_s$s ${2}push300_s$s.yaml 120; done
  echo "push150 $1: $(python3 fall_rate.py fix_$1_mjlab_push150_s*_test.csv | tail -1)"
  echo "push300 $1: $(python3 fall_rate.py fix_$1_mjlab_push300_s*_test.csv | tail -1)"
}
{
CTL_BASE=run/ctl_override_jdr.yaml battery jdr jdr_test_ jdr_straight.yaml
CTL_BASE=run/ctl_override_momentum.yaml battery arm test_ jdr_straight.yaml
for n in fix_*_straight fix_*_turn fix_*_cmd_sched; do [ -f $n.log ] && echo "$n: $(grep -c 'QLD fail' $n.log) QP failures"; done
} > tests_fix.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
echo QUEUE_FIX_DONE >> tests_fix.txt
