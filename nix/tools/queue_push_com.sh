#!/usr/bin/env bash
# 300 N pelvis pushes applied at the pelvis CoM (pushes.at_com, as mjlab's xfrc_applied), 6 seeds per policy.
cd ~/devel/mc_rtc_nix_ws/run
cp mj_g1_mjlab.yaml ~/.config/mc_rtc/mc_mujoco/g1_29dof.yaml
{
for s in 1 2 3 4 5 6; do CTL_BASE=run/ctl_override_jdr.yaml ./run_test.sh com_jdr_push300_s$s test_push300_s$s.yaml 120; done
for s in 1 2 3 4 5 6; do CTL_BASE=run/ctl_override_momentum.yaml ./run_test.sh com_arm_push300_s$s test_push300_s$s.yaml 120; done
echo "push300 at CoM jdr: $(python3 fall_rate.py com_jdr_push300_s*_test.csv | tail -1)"
echo "push300 at CoM arm: $(python3 fall_rate.py com_arm_push300_s*_test.csv | tail -1)"
echo QUEUE_COM_DONE
} > tests_push_com.txt 2>&1
