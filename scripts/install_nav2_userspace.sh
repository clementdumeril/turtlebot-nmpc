#!/usr/bin/env bash
# Nav2 controller_server, MPPI, RPP and costmap without root: download the .deb files from the
# ROS apt repository and unpack them into ~/nav2_local (scripts/env.sh picks it up).
# With sudo, this is simply:
#   sudo apt install ros-jazzy-nav2-controller ros-jazzy-nav2-mppi-controller \
#        ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager
set -e
mkdir -p ~/nav2_local/debs && cd ~/nav2_local/debs
want="ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-controller ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager"
missing=""
for p in $(apt-cache depends --recurse --no-recommends --no-suggests --no-conflicts --no-breaks --no-replaces --no-enhances $want | grep -E '^[a-z0-9]' | sort -u); do
  case "$p" in rti-connext*|ros-jazzy-rmw-connext*|ros-jazzy-rti-*|*cyclonedds*|*iceoryx*) continue ;; esac
  dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"
done
apt-get download $missing
rm -rf ../root && mkdir ../root
for d in ros-jazzy-nav2-*.deb ros-jazzy-nav-2d-*.deb ros-jazzy-bond_*.deb ros-jazzy-bondcpp_*.deb ros-jazzy-smclib_*.deb ros-jazzy-behaviortree-cpp_*.deb libomp5-18_*.deb; do
  [ -f "$d" ] && dpkg -x "$d" ../root
done
echo "Nav2 unpacked in ~/nav2_local/root"
