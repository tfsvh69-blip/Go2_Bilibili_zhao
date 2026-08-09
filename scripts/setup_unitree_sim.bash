#!/usr/bin/env bash

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    echo "请在当前终端加载此文件：source scripts/setup_unitree_sim.bash" >&2
    exit 1
fi

_unitree_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source "${_unitree_script_dir}/setup_simdog.bash" || {
    unset _unitree_script_dir
    return 1
}

if ! ros2 pkg prefix unitree_go >/dev/null 2>&1 ||
    ! ros2 pkg prefix unitree_api >/dev/null 2>&1; then
    echo "尚未构建 Unitree 接口包，请先执行：bash scripts/build_workspaces.sh" >&2
    unset _unitree_script_dir
    return 1
fi

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    echo "缺少 rmw_cyclonedds_cpp，请先执行：bash scripts/install_dependencies.sh" >&2
    unset _unitree_script_dir
    return 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${GO2_UNITREE_SIM_DOMAIN_ID:-${ROS_DOMAIN_ID:-1}}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
export GO2_UNITREE_MODE=simulation

echo "Unitree 仿真通信：CycloneDDS，接口 lo，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
unset _unitree_script_dir
