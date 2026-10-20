#!/usr/bin/env bash
# run_case.sh NAME [extra yaml file appended to the controller override] [wall seconds]
# Runs mc_mujoco headless, links the log as logs/NAME.bin and prints a walking summary.
set -u
WS=$HOME/devel/mc_rtc_nix_ws; cd $WS
NAME=$1; EXTRA=${2:-}; WALL=${3:-180}
OV=$HOME/.config/mc_rtc/controllers/G1RLQPController.yaml
cp run/ctl_override_momentum.yaml $OV
[ -n "$EXTRA" ] && cat "$EXTRA" >> $OV
rm -f run/logs/mc-control-G1RLQPController-latest.bin  # a crash must not reuse the previous log
timeout $((WALL+60)) bash -c "source run/env.sh >/dev/null; timeout -s INT $WALL mc_mujoco -f run/mc_rtc.yaml --without-visualization --without-mc-rtc-gui" > run/$NAME.log 2>&1
cp run/ctl_override_momentum.yaml $OV
L=$(readlink -f run/logs/mc-control-G1RLQPController-latest.bin); ln -sf $L run/logs/$NAME.bin
bash -c "source run/env.sh >/dev/null 2>&1; mc_bin_utils convert --in $L --out run/$NAME.csv --entries t FloatingBase_position FloatingBase_orientation alphaDOut G1RLQPController_qddRefFull >/dev/null 2>&1"
python3 run/summarize.py run/$NAME.csv
