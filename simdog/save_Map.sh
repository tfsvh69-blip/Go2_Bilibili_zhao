#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
destination="${1:-${HOME}/go2_maps/latest}"
resolution="${2:-0.2}"

source /opt/ros/humble/setup.bash
if [[ ! -f "${script_dir}/install/setup.bash" ]]; then
    echo "simdog 尚未构建，请先执行：bash \"${script_dir}/../scripts/build_workspaces.sh\"" >&2
    exit 1
fi
source "${script_dir}/install/setup.bash"

echo "保存地图到：${destination}"
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
    "{resolution: ${resolution}, destination: '${destination}'}"
