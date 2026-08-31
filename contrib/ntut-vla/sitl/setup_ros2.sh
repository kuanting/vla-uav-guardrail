#!/usr/bin/env bash
# ROS 2 Jazzy + MAVROS install for WSL Ubuntu 24.04 — run as ROOT.
# Idempotent.
set -e
export DEBIAN_FRONTEND=noninteractive

echo "=== [1/4] ROS 2 apt source ==="
apt-get install -y -qq software-properties-common curl gnupg lsb-release > /dev/null
add-apt-repository -y universe > /dev/null
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  > /etc/apt/sources.list.d/ros2.list
apt-get update -qq

echo "=== [2/4] ROS 2 Jazzy base + MAVROS (big download) ==="
apt-get install -y -qq ros-jazzy-ros-base ros-jazzy-mavros ros-jazzy-mavros-extras > /dev/null
echo "ros packages installed"

echo "=== [3/4] GeographicLib datasets (MAVROS requirement) ==="
bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh > /dev/null 2>&1 || \
  echo "(datasets script warning ignored if already installed)"

echo "=== [4/4] python venv with ROS access ==="
sudo -u nathan bash -c '
  if [ ! -d ~/venv-ros ]; then python3 -m venv --system-site-packages ~/venv-ros; fi
  ~/venv-ros/bin/pip install -q "pydantic>=2,<3" pyyaml shapely matplotlib
'
echo "=== DONE ==="
ls /opt/ros/jazzy/setup.bash
