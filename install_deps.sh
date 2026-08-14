#!/usr/bin/env bash
#
# Install dependencies for building SUPER and FAST-LIO (ROS 2 Humble / Ubuntu 22.04).
#
# What it installs:
#   1. System libraries: Eigen, PCL, yaml-cpp, QHull, FLANN, fmt, glfw/glew, ncurses, dw
#   2. ROS 2 (Humble) packages: mavros-msgs, pcl-ros, tf2, livox driver, rosfmt ...
#   3. Python packages used by analysis / plotting scripts
#   4. cmd_record dependencies (python3-tk for its live matplotlib plot)
#
# fmt and flann are provided by the system packages libfmt-dev / libflann-dev;
# no Conan is required.
#
# Usage:
#   ./utils/install_deps.sh
#
# Idempotent — safe to run multiple times.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# sudo helper (no-op when already root)
SUDO=""
if [[ $EUID -ne 0 ]]; then
  SUDO="sudo"
fi

section() { echo; echo "==> $*"; }

# ---------------------------------------------------------------------------
# 1. System libraries
# ---------------------------------------------------------------------------
section "1/4 Installing system libraries"
$SUDO apt-get update
# SUPER uses fmt header-only (FMT_HEADER_ONLY); libfmt-dev provides the headers.
# libflann-dev provides the FLANN library that PCL depends on.
$SUDO apt-get install -y \
    libeigen3-dev \
    libdw-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    libqhull-dev \
    libflann-dev \
    libfmt-dev \
    libglfw3-dev \
    libglew-dev \
    libncurses5-dev \
    libncursesw5-dev \
    python3-pip

# Eigen compatibility symlink (SUPER includes <Eigen/...> from /usr/include)
if [[ ! -e /usr/include/Eigen ]]; then
  section "Creating Eigen symlink /usr/include/Eigen -> eigen3/Eigen"
  $SUDO ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
fi

# ---------------------------------------------------------------------------
# 2. ROS 2 packages
# ---------------------------------------------------------------------------
section "2/4 Installing ROS ${ROS_DISTRO} packages"

# Core packages that must exist (fail the script if any of these is missing).
$SUDO apt-get install -y \
    "ros-${ROS_DISTRO}-mavros-msgs" \
    "ros-${ROS_DISTRO}-pcl-ros" \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-vision-msgs" \
    "ros-${ROS_DISTRO}-rosidl-default-generators"

# Best-effort packages. They may not be resolvable from the configured apt
# repos on every system, so install them separately and warn instead of failing.
# $SUDO apt-get install -y \
#     "ros-${ROS_DISTRO}-rosfmt" \
#     "ros-${ROS_DISTRO}-livox-ros-driver2" \
#     || echo "NOTE: rosfmt / livox-ros-driver2 not available via apt. FAST-LIO "
#        "requires livox_ros_driver2 - build it from source if missing:"
#        "  git clone https://github.com/Livox-SDK/livox_ros_driver2.git"

# ---------------------------------------------------------------------------
# 3. Python packages
# ---------------------------------------------------------------------------
section "3/3 Installing Python packages"
# Keep numpy on 1.x: the system matplotlib (apt, e.g. 3.5.1 on Ubuntu 22.04) is
# compiled against the NumPy 1.x ABI and crashes under numpy>=2
# ("_ARRAY_API not found" / "numpy.core.multiarray failed to import").
# pandas 2.x supports numpy 1.26, so this does not break it.
python3 -m pip install --user --quiet 'numpy<2' pandas matplotlib || true
section "3/4 Installing Python packages"
# numpy/matplotlib (used by cmd_record's recorder + plot_csv and by analysis
# scripts); pandas is optional for custom CSV analysis.
python3 -m pip install --user --quiet numpy pandas matplotlib || true

# ---------------------------------------------------------------------------
# 4. cmd_record (goal-triggered recorder + live matplotlib plot)
# ---------------------------------------------------------------------------
section "4/4 Installing cmd_record dependencies"
# cmd_record is a pure-Python ROS 2 package. Its ROS deps (rclpy,
# geometry_msgs, nav_msgs) ship with the base ROS install; mars_quadrotor_msgs
# is built with colcon. numpy + matplotlib are installed in step 3; the live
# sliding-window plot uses matplotlib's TkAgg backend, which needs the system
# Tk libraries (python3-tk), installed here.
$SUDO apt-get install -y python3-tk

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo
echo "Dependency installation finished."
echo "Next steps:"
echo "  1. source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  2. colcon build --symlink-install"
