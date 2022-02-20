#!/usr/bin/env bash

# ROS2 install from source
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html

echo ">> setup ROS2 apt repository"
apt update
apt upgrade -y
apt install --no-install-recommends -y curl ca-certificates gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $VERSION_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ">> install basic build dependencies"
apt update
apt install --no-install-recommends -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-rosinstall-generator \
  python3-setuptools \
  python3-vcstool \
  wget \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev

python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

echo ">> download source"
SCRIPT_PATH=$(dirname $(realpath $0))
mkdir -p $SCRIPT_PATH/src
cd $SCRIPT_PATH
rosinstall_generator ros_base image_common vision_opencv demo_nodes_cpp --rosdistro galactic --deps --format=repos > base.repos
vcs import src < base.repos

echo ">> install build dependencies"
rosdep init
rosdep update
source /etc/os-release
rosdep install --from-paths src --ignore-src -y --os=debian:$VERSION_CODENAME --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers python3-ifcfg"

# echo ">> build workspace"
# cd $SCRIPT_PATH
# colcon build --merge-install
