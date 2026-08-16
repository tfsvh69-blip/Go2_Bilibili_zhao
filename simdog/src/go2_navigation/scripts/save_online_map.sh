#!/usr/bin/env bash

set -euo pipefail

map_name="${1:-$(date +%Y%m%d_%H%M%S)}"
if [[ ! ${map_name} =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "地图名只能包含字母、数字、点、下划线和连字符：${map_name}" >&2
    exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../../../.." && pwd)"
# 在线地图根目录位于项目根内，保持项目独立；支持 GO2_ONLINE_MAP_ROOT 覆盖。
map_root="${GO2_ONLINE_MAP_ROOT:-${project_root}/go2_maps/online}"
mkdir -p "${map_root}"
map_root="$(realpath "${map_root}")"
final_dir="${map_root}/${map_name}"
if [[ -e ${final_dir} ]]; then
    echo "目标地图目录已存在，为避免覆盖已拒绝：${final_dir}" >&2
    exit 2
fi

temp_dir="$(mktemp -d "${map_root}/.${map_name}.tmp.XXXXXX")"
cleanup() {
    if [[ -n ${temp_dir:-} && -d ${temp_dir} ]]; then
        rm -rf -- "${temp_dir}"
    fi
}
trap cleanup EXIT

echo "[1/3] 保存 Slam Toolbox pose graph"
serialize_output="$(ros2 service call \
    /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${temp_dir}/slam'}")"
# Humble 的 ros2cli 在不同补丁版本中分别打印 ``result=0`` 或
# YAML 风格的 ``result: 0``，两种都表示 Slam Toolbox 保存成功。
if ! grep -Eq "result[=:][[:space:]]*0" <<<"${serialize_output}"; then
    echo "pose graph 保存失败：" >&2
    echo "${serialize_output}" >&2
    exit 1
fi

echo "[2/3] 保存当前 /map 为 PGM/YAML"
ros2 run nav2_map_server map_saver_cli -f "${temp_dir}/map" \
    --ros-args -p use_sim_time:=true

for required in slam.posegraph slam.data map.pgm map.yaml; do
    if [[ ! -s ${temp_dir}/${required} ]]; then
        echo "在线地图保存结果缺少：${required}" >&2
        exit 1
    fi
done

echo "[3/3] 原子公布地图会话"
mv -- "${temp_dir}" "${final_dir}"
temp_dir=""
ln -sfn -- "${final_dir}" "${map_root}/latest"

echo "在线地图已保存：${final_dir}"
echo "下次续建："
echo "  ros2 launch go2_navigation simulation_navigation.launch.xml navigation_mode:=online_slam map_session:=${final_dir}"
echo "作为固定二维地图启动 AMCL："
echo "  ros2 launch go2_navigation simulation_navigation.launch.xml navigation_mode:=static_map localization:=amcl map_dir:=${final_dir}"
