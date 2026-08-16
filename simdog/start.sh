#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_setup="${script_dir}/install/setup.bash"
unitree_setup="${script_dir}/../scripts/setup_unitree_sim.bash"
wait_time="${WAIT_TIME:-8}"
# 地图数据目录位于项目根内，保持项目独立；支持 GO2_MAPS_ROOT 覆盖。
go2_maps_root="${GO2_MAPS_ROOT:-${project_root}/go2_maps}"
map_path="${1:-${go2_maps_root}/latest/GlobalMap.pcd}"
gpu_device="${GO2_GPU_DEVICE:-0}"
force_nvidia_rendering="${GO2_FORCE_NVIDIA_RENDERING:-1}"

if [[ ! -f ${workspace_setup} ]]; then
    echo "simdog 尚未构建，请先执行：bash \"${script_dir}/../scripts/build_workspaces.sh\"" >&2
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "未找到 gnome-terminal，请按 README.md 中的命令分终端启动。" >&2
    exit 1
fi

printf -v unitree_setup_q '%q' "${unitree_setup}"
printf -v map_path_q '%q' "${map_path}"
common_setup="source ${unitree_setup_q}"
if command -v nvidia-smi >/dev/null 2>&1 &&
    [[ -x /usr/local/cuda-12.8/bin/nvcc ]]; then
    printf -v gpu_device_q '%q' "${gpu_device}"
    common_setup+=" && export PATH=/usr/local/cuda-12.8/bin:\${PATH}"
    common_setup+=" && export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:\${LD_LIBRARY_PATH:-}"
    common_setup+=" && export CUDA_VISIBLE_DEVICES=${gpu_device_q}"
    if [[ ${force_nvidia_rendering} == "1" ]]; then
        common_setup+=" && export __NV_PRIME_RENDER_OFFLOAD=1"
        common_setup+=" && export __GLX_VENDOR_LIBRARY_NAME=nvidia"
        common_setup+=" && unset LIBGL_ALWAYS_SOFTWARE"
    fi
    echo "GPU 默认启用：CUDA 设备 ${gpu_device}，NVIDIA 渲染=${force_nvidia_rendering}"
fi

open_terminal() {
    local title="$1"
    local command="$2"
    gnome-terminal --title="${title}" -- \
        bash -lc "${common_setup} && ${command}; exec bash"
}

open_terminal "Go2 Gazebo" \
    "ros2 launch go2_config gazebo_velodyne.launch.py gui:=true rviz:=true"

sleep "${wait_time}"

if [[ -f ${map_path} ]]; then
    open_terminal "Go2 LIO-SAM" \
        "ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=false"
    open_terminal "Go2 NDT 重定位" \
        "ros2 launch ndt_relocalization ndt_localization.launch.py map_path:=${map_path_q} registration_backend:=cuda gpu_device_id:=0"
    echo "已使用地图启动 NDT：${map_path}"
else
    open_terminal "Go2 LIO-SAM" \
        "ros2 launch lio_sam lidar.launch.py rviz:=true publish_map_to_odom:=true"
    echo "未找到 NDT 地图，已跳过重定位：${map_path}"
    echo "建图完成后执行 bash \"${script_dir}/save_Map.sh\"，再重新运行本脚本。"
fi

open_terminal "Go2 键盘遥控" \
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
