#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
ros_setup="/opt/ros/humble/setup.bash"
cuda_root="/usr/local/cuda-12.8"

if [[ ! -f ${ros_setup} ]]; then
    echo "未找到 ${ros_setup}，请先安装 ROS 2 Humble。" >&2
    exit 1
fi

echo "开始构建 simdog 完整四足工作空间"
(
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    source "${ros_setup}"
    export MAKEFLAGS=-j4
    cd "${project_root}/simdog"

    common_build_arguments=(--symlink-install --parallel-workers 2)
    fast_gicp_arguments=()
    ndt_arguments=()
    cuda_architecture="${GO2_CUDA_ARCHITECTURE:-}"

    if [[ -x "${cuda_root}/bin/nvcc" ]]; then
        if [[ -z ${cuda_architecture} ]] && command -v nvidia-smi >/dev/null 2>&1; then
            cuda_architecture="$(nvidia-smi \
                --query-gpu=compute_cap \
                --format=csv,noheader,nounits \
                --id="${GO2_GPU_DEVICE:-0}" 2>/dev/null | head -n 1 | tr -d '.[:space:]')"
        fi
        if [[ ! ${cuda_architecture} =~ ^[0-9]+$ ]]; then
            echo "检测到 CUDA 12.8，但无法识别目标 GPU 架构。" >&2
            echo "请设置 GO2_CUDA_ARCHITECTURE（例如 RTX 5070 为 120），或暂时移除 CUDA 路径以构建 CPU 版本。" >&2
            exit 1
        fi
        export PATH="${cuda_root}/bin:${PATH}"
        export LD_LIBRARY_PATH="${cuda_root}/lib64:${LD_LIBRARY_PATH:-}"
        fast_gicp_arguments=(
            --cmake-args
            -DBUILD_VGICP_CUDA=ON
            "-DCUDA_TOOLKIT_ROOT_DIR=${cuda_root}"
            "-DFAST_GICP_CUDA_ARCHITECTURE=${cuda_architecture}"
        )
        ndt_arguments=(--cmake-args -DUSE_FAST_GICP_CUDA=ON)
        echo "检测到 CUDA 12.8，将构建 GPU NDT 后端（sm_${cuda_architecture}）。"
    else
        fast_gicp_arguments=(--cmake-args -DBUILD_VGICP_CUDA=OFF)
        ndt_arguments=(--cmake-args -DUSE_FAST_GICP_CUDA=OFF)
        echo "未检测到 CUDA 12.8，将构建 CPU 回退版本。"
    fi

    # GPU 选项只传给实际使用它们的包，避免其他包产生无关 CMake 警告。
    # go2_navigation 声明依赖 fast_gicp 和 ndt_relocalization；全新工作区必须最后构建。
    colcon build "${common_build_arguments[@]}" \
        --packages-skip fast_gicp ndt_relocalization go2_navigation
    colcon build "${common_build_arguments[@]}" \
        --packages-select fast_gicp "${fast_gicp_arguments[@]}"
    source install/setup.bash
    colcon build "${common_build_arguments[@]}" \
        --packages-select ndt_relocalization "${ndt_arguments[@]}"
    source install/setup.bash
    colcon build "${common_build_arguments[@]}" \
        --packages-select go2_navigation
)

echo
echo "simdog 完整四足工作空间已构建完成。"
echo "  source scripts/setup_simdog.bash"
