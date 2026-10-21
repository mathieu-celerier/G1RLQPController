#!/usr/bin/env bash
# run_test.sh NAME TEST_YAML [wall seconds]: run_case.sh with the sim_test block, then per-segment analysis.
WS=$HOME/devel/mc_rtc_nix_ws; cd $WS/run
./run_case.sh "$1" "$(readlink -f "$2")" "${3:-150}" > /dev/null 2>&1
bash -c "source env.sh >/dev/null 2>&1; mc_bin_utils convert --in logs/$1.bin --out $1_test.csv --entries t FloatingBase_position FloatingBase_orientation G1RLQPController_velCmd G1RLQPController_pushForce >/dev/null 2>&1"
echo "== $1"; python3 analyze_test.py $1_test.csv
# DROP_BIN=1: delete the raw mc_rtc log once converted (8.5 GB per 150 s run) and the summary CSV.
if [ -n "${DROP_BIN:-}" ] && [ -s $1_test.csv ]; then rm -f "$(readlink -f logs/$1.bin)" logs/$1.bin $1.csv; fi
