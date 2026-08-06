#!/usr/bin/env bash
#
# start_sim_gpu.sh — Force NVIDIA GPU for the Yungu simulation stack.
#
# This wrapper unsets software-renderer variables and exports the NVIDIA
# adapter before launching the standard start_sim.sh.
#
# Usage:
#   ./temp/start_sim_gpu.sh
#   PX4_MODEL=gz_x500_lidar_yungu ./temp/start_sim_gpu.sh
#   XRCE_PORT=8888 ./temp/start_sim_gpu.sh
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE}"

# --- Unset software / WSL fallback variables --------------------------------
unset LIBGL_ALWAYS_SOFTWARE
unset GZ_SIM_RENDER_ENGINE
unset MESA_D3D12_DEFAULT_ADAPTER_NAME

# --- Force NVIDIA -----------------------------------------------------------
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# --- Quick GPU check --------------------------------------------------------
echo "=== GPU Info ==="
if command -v glxinfo &>/dev/null; then
    glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer|OpenGL version' || true
else
    echo "(glxinfo not available — install mesa-utils for GPU info)"
fi
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-unset}"
echo "================="
echo ""

# --- Launch the standard simulation script ----------------------------------
exec "${WORKSPACE}/src/utils/start_sim.sh" "$@"
