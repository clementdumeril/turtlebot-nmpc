#!/usr/bin/env bash
# Webots (Windows) + the stack (WSL2).  bash scripts/sim.sh controller:=nmpc scenario:=crossing
source "$(dirname "$0")/env.sh"
exec ros2 launch tb_nmpc_sim sim.launch.py "$@"
