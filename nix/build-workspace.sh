#!/usr/bin/env bash
# Build the source components of the G1RLQPController workspace inside the nix shell:
#   nix develop ./nix -c ./nix/build-workspace.sh [component...]
# Sources live in $G1_WS/src (pinned commits: deploy/pinned-commits.txt), install in $G1_WS/install.
set -euo pipefail
WS=${G1_WS:-$HOME/devel/mc_rtc_nix_ws}
CTRL=$(cd "$(dirname "$0")/.." && pwd)
PREFIX=$WS/install
mkdir -p "$PREFIX"
# Raphaël built everything into one prefix; the controller relies on it for mc_joystick_plugin headers.
export CPATH=$PREFIX/include${CPATH:+:$CPATH}
build() {
  local name=$1 src=$2; shift 2
  echo "=== $name"
  cmake -S "$src" -B "$WS/build/$name" -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_PREFIX_PATH="$PREFIX" -DMC_RTC_HONOR_INSTALL_PREFIX=ON \
    -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
  cmake --build "$WS/build/$name" -j"$(nproc)"
  cmake --install "$WS/build/$name"
}
ALL=(mc_joystick_plugin mc_external_forces_observer mc_mujoco g1_mj_description G1RLQPController)
for c in "${@:-${ALL[@]}}"; do
  case $c in
    G1RLQPController) build "$c" "$CTRL" ;;
    *) build "$c" "$WS/src/$c" ;;
  esac
done
