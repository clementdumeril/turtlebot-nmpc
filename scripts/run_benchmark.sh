#!/usr/bin/env bash
# Every scenario with every controller, each run in a fresh simulation.
#   bash scripts/run_benchmark.sh results/sim_2026-10-30
#   SCENARIOS="crossing" CONTROLLERS="nmpc mppi" bash scripts/run_benchmark.sh results/test
# About 2 min per run (Webots start + one 51 s lap); 4 x 4 x 2 runs in about an hour.
cd "$(dirname "$0")/.."
source scripts/env.sh   # before set -u: the ROS setup files use unset variables
set -u
OUT="${1:-results/sim}"
SCENARIOS="${SCENARIOS:-clear static crossing crossing_fast}"
CONTROLLERS="${CONTROLLERS:-nmpc nmpc_nopred mppi rpp}"
REPEATS="${REPEATS:-2}"
STACK="ros2 launch tb_nmpc_sim|nmpc_node|nav2_follow|controller_server|lifecycle_manager|webots_controller|ros2_supervisor|robot_state_publisher|static_transform_publisher"

webots_running() { tasklist.exe 2>/dev/null | grep -qai "webots"; }

stop_sim() {
  # SIGTERM, not SIGINT: a job started with & in a script ignores SIGINT.
  pkill -TERM -f "ros2 launch tb_nmpc_sim"
  for _ in $(seq 1 15); do (pgrep -f "$STACK" > /dev/null || webots_running) || return 0; sleep 2; done
  pkill -KILL -f "$STACK"
  taskkill.exe /IM webots.exe /F > /dev/null 2>&1
  sleep 3
}

mkdir -p "$OUT"
for i in $(seq 1 "$REPEATS"); do
for s in $SCENARIOS; do
  for c in $CONTROLLERS; do
    run="$OUT/${s}_${c}_$i"
    if [ -f "$run/meta.json" ]; then echo "skip $run"; continue; fi
    stop_sim
    echo "=== $s / $c"
    mkdir -p "$run"
    bash scripts/sim.sh controller:="$c" scenario:="$s" > "$run/sim.log" 2>&1 &
    timeout 400 python3 scripts/benchmark.py --controller "$c" --scenario "$s" --out "$run"
    stop_sim
  done
done
done
python3 scripts/metrics.py "$OUT"
