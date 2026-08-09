#!/usr/bin/env bash

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    echo "请在当前终端加载此文件：source scripts/setup_unitree_real.bash <网卡名>" >&2
    exit 1
fi

_unitree_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_unitree_network_interface="${1:-${GO2_NETWORK_INTERFACE:-}}"

if [[ -z ${_unitree_network_interface} ]]; then
    echo "必须显式提供连接 Go2 的网卡，例如：" >&2
    echo "  source scripts/setup_unitree_real.bash enp3s0" >&2
    unset _unitree_script_dir _unitree_network_interface
    return 1
fi

if ! ip link show dev "${_unitree_network_interface}" >/dev/null 2>&1; then
    echo "找不到网卡：${_unitree_network_interface}" >&2
    unset _unitree_script_dir _unitree_network_interface
    return 1
fi

if ! ip link show dev "${_unitree_network_interface}" | grep -q "state UP"; then
    echo "网卡未处于 UP 状态：${_unitree_network_interface}" >&2
    unset _unitree_script_dir _unitree_network_interface
    return 1
fi

source "${_unitree_script_dir}/setup_simdog.bash" || {
    unset _unitree_script_dir _unitree_network_interface
    return 1
}

if ! ros2 pkg prefix unitree_go >/dev/null 2>&1 ||
    ! ros2 pkg prefix unitree_api >/dev/null 2>&1; then
    echo "尚未构建 Unitree 接口包，请先执行：bash scripts/build_workspaces.sh" >&2
    unset _unitree_script_dir _unitree_network_interface
    return 1
fi

if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    echo "缺少 rmw_cyclonedds_cpp，请先执行：bash scripts/install_dependencies.sh" >&2
    unset _unitree_script_dir _unitree_network_interface
    return 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${GO2_UNITREE_REAL_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${_unitree_network_interface}\" priority=\"default\" multicast=\"default\"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
export GO2_NETWORK_INTERFACE="${_unitree_network_interface}"
export GO2_UNITREE_MODE=real

echo "Unitree 真机通信：CycloneDDS，接口 ${GO2_NETWORK_INTERFACE}，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "警告：当前终端将直接发现真机 DDS 服务；不要启动 go2_unitree_sim_bridge。"
unset _unitree_script_dir _unitree_network_interface
