#!/usr/bin/env bash

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    echo "请在当前终端加载此文件：source scripts/setup_simdog.bash" >&2
    exit 1
fi

_go2_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_go2_project_root="$(cd -- "${_go2_script_dir}/.." && pwd)"

if [[ ! -f "${_go2_project_root}/simdog/install/setup.bash" ]]; then
    echo "simdog 尚未构建，请先执行：bash scripts/build_workspaces.sh" >&2
    unset _go2_script_dir _go2_project_root
    return 1
fi

# 清除 .bashrc 或之前 source 的其他 ROS 2 工作空间（如 dobot_ws）残留的
# PYTHONPATH/AMENT/CMAKE/COLCON 前缀，避免污染 Gazebo 子进程导致启动失败。
# 这些变量随后由 /opt/ros/humble 与 simdog 的 setup.bash 重新生成。
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/humble/setup.bash
source "${_go2_project_root}/simdog/install/setup.bash"
if command -v nvidia-smi >/dev/null 2>&1 &&
    [[ -x /usr/local/cuda-12.8/bin/nvcc ]]; then
    export PATH="/usr/local/cuda-12.8/bin:${PATH}"
    export LD_LIBRARY_PATH="/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH:-}"
    export GO2_GPU_DEVICE="${GO2_GPU_DEVICE:-0}"
    export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-${GO2_GPU_DEVICE}}"
    export GO2_USE_GPU=1
    export GO2_FORCE_NVIDIA_RENDERING="${GO2_FORCE_NVIDIA_RENDERING:-1}"
    if [[ ${GO2_FORCE_NVIDIA_RENDERING} == "1" ]]; then
        export __NV_PRIME_RENDER_OFFLOAD=1
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
        unset LIBGL_ALWAYS_SOFTWARE
    fi
else
    export GO2_USE_GPU=0
fi
export GO2_PROJECT_ROOT="${_go2_project_root}"
export GO2_WORKSPACE="${_go2_project_root}/simdog"
echo "已加载 simdog：${GO2_WORKSPACE}"
if [[ ${GO2_USE_GPU} == "1" ]]; then
    echo "GPU 默认启用：CUDA 逻辑设备 ${CUDA_VISIBLE_DEVICES}，NVIDIA 渲染=${GO2_FORCE_NVIDIA_RENDERING}"
else
    echo "未检测到完整 GPU 环境，将使用 CPU 回退。" >&2
fi

unset _go2_script_dir _go2_project_root
