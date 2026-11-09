# Source in every WSL terminal:  source scripts/env.sh
# ROS 2, the optional user-space Nav2 (scripts/install_nav2_userspace.sh), acados, the workspace.
# Keep an environment that is already set up (a lab .bashrc may already source the TurtleBot3 workspace).
[ -n "$AMENT_PREFIX_PATH" ] || source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
N2=$HOME/nav2_local/root
if [ -d "$N2/opt/ros/jazzy" ] && [ ! -d /opt/ros/jazzy/share/nav2_mppi_controller ]; then
  export AMENT_PREFIX_PATH=$N2/opt/ros/jazzy:$AMENT_PREFIX_PATH
  export LD_LIBRARY_PATH=$N2/opt/ros/jazzy/lib:$N2/usr/lib/llvm-18/lib:$LD_LIBRARY_PATH
  export PYTHONPATH=$N2/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
  export PATH=$N2/opt/ros/jazzy/bin:$PATH
fi
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
# acados_template and casadi only; numpy and scipy stay the system ones that rclpy uses.
export PYTHONPATH=$PYTHONPATH:$ACADOS_SOURCE_DIR/interfaces/acados_template:$HOME/.cache/tb_nmpc/pylib
[ -f "$HOME/tb_ws/install/setup.bash" ] && source "$HOME/tb_ws/install/setup.bash"
# Webots installed per user on Windows; webots_ros2 starts it from WSL.
export WEBOTS_HOME="${WEBOTS_HOME:-/mnt/c/Users/$(cmd.exe /c "echo %USERNAME%" 2>/dev/null | tr -d '\r')/AppData/Local/Programs/Webots}"
