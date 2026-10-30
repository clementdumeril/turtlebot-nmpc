#!/usr/bin/env bash
# One-time setup in WSL2 (Ubuntu 24.04, ROS 2 Jazzy): workspace, casadi link, build.
#   bash scripts/setup_ws.sh
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p ~/tb_ws
[ -e ~/tb_ws/src ] || ln -s "$REPO/src" ~/tb_ws/src
# casadi from a venv that has it (pip install casadi), linked so that ROS's python finds it.
mkdir -p ~/.cache/tb_nmpc/pylib
CASADI=$(python3 -c "import casadi, os; print(os.path.dirname(casadi.__file__))" 2>/dev/null || true)
[ -z "$CASADI" ] && CASADI=$(ls -d ~/venvs/*/lib/python3*/site-packages/casadi 2>/dev/null | head -1)
[ -n "$CASADI" ] && ln -sfn "$CASADI" ~/.cache/tb_nmpc/pylib/casadi
source "$REPO/scripts/env.sh"
cd ~/tb_ws && colcon build --symlink-install
echo "built; now: source scripts/env.sh"
