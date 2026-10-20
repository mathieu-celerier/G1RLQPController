#!/usr/bin/env bash
# ArmWrenchJointDR model_17998 in mc_mujoco: stock models (straight 0.5), matched model (turn, schedule, pushes).
cd ~/devel/mc_rtc_nix_ws/run
export CTL_BASE=run/ctl_override_jdr.yaml
MJ=~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
{
cp ~/devel/mc_rtc_nix_ws/install/share/mc_mujoco/g1_29dof.yaml $MJ; ./run_test.sh jdr_stock1ms_straight jdr_straight.yaml 150
cp mj_g1_rt.yaml $MJ; ./run_test.sh jdr_stockrt_straight jdr_straight.yaml 150
cp mj_g1_mjlab.yaml $MJ
./run_test.sh jdr_mjlab_straight jdr_straight.yaml 150
./run_test.sh jdr_mjlab_turn jdr_test_cmd_turn.yaml 200
./run_test.sh jdr_mjlab_cmd_sched jdr_test_cmd_sched.yaml 200
for s in 1 2 3; do ./run_test.sh jdr_mjlab_push150_s$s jdr_test_push150_s$s.yaml 120; done
for s in 1 2 3; do ./run_test.sh jdr_mjlab_push300_s$s jdr_test_push300_s$s.yaml 60; done
python3 fall_rate.py jdr_mjlab_push150_s*_test.csv | tail -1
python3 fall_rate.py jdr_mjlab_push300_s*_test.csv | tail -1
for n in jdr_stock1ms_straight jdr_stockrt_straight jdr_mjlab_straight jdr_mjlab_turn jdr_mjlab_cmd_sched; do echo "$n: $(grep -c 'QLD fail' $n.log) QP failures; $(grep -m1 'policy\|Observation size' $n.log | cut -c1-120)"; done
} > tests_jdr.txt 2>&1
cp mj_g1_mjlab.yaml $MJ
