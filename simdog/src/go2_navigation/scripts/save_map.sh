#!/usr/bin/env bash
# 保存 LIO-SAM 建图结果并生成同源导航地图包。
#
# 1) 调用 /lio_sam/save_map 保存 PCD（GlobalMap.pcd 等）；
# 2) 调用 build_map_bundle 由 GlobalMap.pcd 生成 Nav2 的 map.yaml/pgm 与
#    map_bundle.yaml（含 SHA-256 校验清单）。
#
# 用法：
#   bash simdog/src/go2_navigation/scripts/save_map.sh [destination] [resolution]
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../../../.." && pwd)"
go2_maps_root="${GO2_MAPS_ROOT:-${project_root}/go2_maps}"
destination="${1:-${go2_maps_root}/latest}"
resolution="${2:-0.2}"

source /opt/ros/humble/setup.bash
if [[ ! -f "${script_dir}/../../../install/setup.bash" ]]; then
    echo "simdog 尚未构建，请先执行：bash scripts/build_workspaces.sh" >&2
    exit 1
fi
source "${script_dir}/../../../install/setup.bash"

echo "[1/2] 保存 LIO-SAM PCD 地图到：${destination}"
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
    "{resolution: ${resolution}, destination: '${destination}'}"

echo "[2/2] 由 GlobalMap.pcd 生成同源导航地图包（map.yaml/pgm + map_bundle.yaml）"
ros2 run go2_navigation build_map_bundle --map-dir "${destination}" \
    --resolution 0.10

echo "地图包已生成：${destination}"
echo "  GlobalMap.pcd   -> NDT/GICP 定位"
echo "  map.yaml/pgm    -> Nav2 全局代价地图"
echo "  map_bundle.yaml -> 启动前完整性校验清单"
