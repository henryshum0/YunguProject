#!/usr/bin/env bash
#
# YunguProject 编译脚本（不需要 root）
#
#   bash build.sh            # 全量编译
#   bash build.sh <pkg>...   # 只编指定包
#
# 前置：先跑完 setup_env.sh。

set -euo pipefail

PROJ="/home/zhugb/Projects/GCI_projects/YunguProject"
cd "${PROJ}"

set +u
source /opt/ros/humble/setup.bash
set -u

echo "ROS_DISTRO=${ROS_DISTRO}  ROS_VERSION=${ROS_VERSION}"

ARGS=(--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
if [[ $# -gt 0 ]]; then
  colcon build --packages-select "$@" "${ARGS[@]}"
else
  colcon build "${ARGS[@]}"
fi

echo
echo "编译完成。使用前先 source："
echo "  source ${PROJ}/install/setup.bash"
