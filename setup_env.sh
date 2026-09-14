#!/usr/bin/env bash
#
# YunguProject 环境安装脚本（需要 sudo 密码）
#
#   bash setup_env.sh
#
# 执行内容：
#   1. PX4 官方 ubuntu.sh --no-nuttx  -> PX4 编译工具链 + Gazebo Harmonic + PX4 Python 依赖
#   2. ros-humble-ros-gzharmonic      -> Harmonic 版 ros_gz bridge（apt 默认版是 Fortress，不能用）
#   3. install_deps.sh                -> SUPER / FAST-LIO 的系统库 + ROS 包 + Python 包
#   4. libapr1-dev / xterm            -> livox 驱动可选依赖 + PX4 SITL 窗口
#   5. MicroXRCEAgent 安装            -> 已编译好，只做 make install
#
# 幂等，可重复运行。编译由 build.sh 单独执行（不需要 root）。

set -euo pipefail

PROJ="/home/zhugb/Projects/GCI_projects/YunguProject"
AGENT_BUILD="/home/zhugb/Projects/GCI_projects/Micro-XRCE-DDS-Agent/build"

section() { echo; echo "############ $* ############"; echo; }

# 先要一次 sudo 授权，后续步骤复用同一个 tty 的时间戳缓存
sudo -v

section "1/5  PX4 工具链 + Gazebo Harmonic (ubuntu.sh --no-nuttx)"
bash "${PROJ}/VisionFlow-PX4/Tools/setup/ubuntu.sh" --no-nuttx

section "2/5  ros_gz (Harmonic 版)"
sudo apt-get install -y ros-humble-ros-gzharmonic

section "3/5  仓库自带依赖 (install_deps.sh)"
cd "${PROJ}" && ./install_deps.sh

section "4/5  补充依赖"
sudo apt-get install -y libapr1-dev xterm

section "5/5  安装 MicroXRCEAgent"
if [[ -d "${AGENT_BUILD}" ]]; then
  cd "${AGENT_BUILD}"
  sudo make install
  sudo ldconfig /usr/local/lib/
else
  echo "ERROR: ${AGENT_BUILD} 不存在，Agent 还没编译" >&2
  exit 1
fi

section "完成"
echo "验证："
echo "  gz sim --version"
echo "  MicroXRCEAgent --help | head -3"
echo "  ros2 pkg prefix ros_gz_bridge"
echo
echo "下一步（不需要 root）： bash ${PROJ}/build.sh"
