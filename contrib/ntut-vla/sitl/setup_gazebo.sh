#!/usr/bin/env bash
# Gazebo Harmonic + ardupilot_gazebo plugin for WSL Ubuntu 24.04. Run as ROOT.
# Idempotent.
set -e
export DEBIAN_FRONTEND=noninteractive

echo "=== [1/3] Gazebo Harmonic apt ==="
apt-get install -y -qq curl gnupg lsb-release > /dev/null
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
  curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update -qq
apt-get install -y -qq gz-harmonic libgz-sim8-dev rapidjson-dev cmake build-essential > /dev/null
echo "gz-harmonic installed"

echo "=== [2/3] ardupilot_gazebo plugin ==="
sudo -u nathan bash -c '
  set -e
  cd ~
  if [ ! -d ardupilot_gazebo ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git
  fi
  cd ardupilot_gazebo
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo > /dev/null
  make -j"$(nproc)" > /dev/null
'
echo "plugin built"

echo "=== [3/3] verify ==="
ls /home/nathan/ardupilot_gazebo/build/*.so
gz sim --version | head -1
echo "=== DONE ==="
