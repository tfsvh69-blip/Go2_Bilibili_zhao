#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
setup_script="${script_dir}/setup_simdog.bash"
sample_map="${project_root}/simdog/src/fast_gicp/data/251370668.pcd"
sample_scan="${project_root}/simdog/src/fast_gicp/data/251371071.pcd"
node_binary="${project_root}/simdog/install/ndt_relocalization/lib/ndt_relocalization/ndt_relocalization_node"
cuda_library="${project_root}/simdog/install/fast_gicp/lib/libfast_vgicp_cuda.so"
temporary_dir="$(mktemp -d /tmp/go2_gpu_verify.XXXXXX)"
node_log="${temporary_dir}/ndt.log"
pose_output="${temporary_dir}/pose.txt"
gpu_samples="${temporary_dir}/gpu_samples.csv"
node_pid=""
tf_pid=""
publisher_pid=""
verify_domain="${GO2_VERIFY_ROS_DOMAIN_ID:-$((100 + ($$ % 100)))}"

# 验证必须与用户正在运行的 Gazebo/LIO-SAM 图隔离，避免 /clock 和 TF 时间基准冲突。
export ROS_DOMAIN_ID="${verify_domain}"
export ROS_LOCALHOST_ONLY=1

cleanup() {
    local process_id
    local child_id
    local attempt

    for process_id in "${publisher_pid}" "${tf_pid}" "${node_pid}"; do
        if [[ -n ${process_id} ]] && kill -0 "${process_id}" 2>/dev/null; then
            for child_id in $(pgrep -P "${process_id}" 2>/dev/null || true); do
                kill -TERM "${child_id}" 2>/dev/null || true
            done
            kill -TERM "${process_id}" 2>/dev/null || true
        fi
    done

    for attempt in $(seq 1 20); do
        if ! {
            [[ -n ${publisher_pid} ]] && kill -0 "${publisher_pid}" 2>/dev/null
        } && ! {
            [[ -n ${tf_pid} ]] && kill -0 "${tf_pid}" 2>/dev/null
        } && ! {
            [[ -n ${node_pid} ]] && kill -0 "${node_pid}" 2>/dev/null
        }; then
            break
        fi
        sleep 0.1
    done

    for process_id in "${publisher_pid}" "${tf_pid}" "${node_pid}"; do
        if [[ -n ${process_id} ]] && kill -0 "${process_id}" 2>/dev/null; then
            for child_id in $(pgrep -P "${process_id}" 2>/dev/null || true); do
                kill -KILL "${child_id}" 2>/dev/null || true
            done
            kill -KILL "${process_id}" 2>/dev/null || true
        fi
        if [[ -n ${process_id} ]]; then
            wait "${process_id}" 2>/dev/null || true
        fi
    done

    if [[ -d ${temporary_dir} ]]; then
        find "${temporary_dir}" -type f -delete
        find "${temporary_dir}" -type l -delete
        find "${temporary_dir}" -depth -type d -empty -delete
    fi
}
trap cleanup EXIT INT TERM

fail() {
    echo "验证失败：$1" >&2
    if [[ -s ${node_log} ]]; then
        echo "NDT 最近日志：" >&2
        tail -n 30 "${node_log}" >&2
    fi
    exit 1
}

wait_for_log() {
    local pattern="$1"
    local attempts="${2:-30}"
    local attempt

    for ((attempt = 1; attempt <= attempts; attempt++)); do
        if rg -q "${pattern}" "${node_log}" 2>/dev/null; then
            return 0
        fi
        if [[ -n ${node_pid} ]] && ! kill -0 "${node_pid}" 2>/dev/null; then
            return 1
        fi
        sleep 1
    done
    return 1
}

echo "[1/6] 检查 NVIDIA、CUDA 和 GPU 二进制"
echo "隔离验证域：ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
command -v nvidia-smi >/dev/null 2>&1 ||
    fail "未找到 nvidia-smi。"
[[ -x /usr/local/cuda-12.8/bin/nvcc ]] ||
    fail "未找到 CUDA 12.8，请先执行 bash scripts/install_gpu_dependencies.sh。"
[[ -x ${node_binary} ]] ||
    fail "未找到 NDT 节点，请先执行 bash scripts/build_workspaces.sh。"
[[ -f ${cuda_library} ]] ||
    fail "未找到 fast_gicp CUDA 库。"
[[ -f ${sample_map} && -f ${sample_scan} ]] ||
    fail "缺少 fast_gicp 测试点云。"

nvidia-smi --query-gpu=index,name,compute_cap,driver_version,memory.total \
    --format=csv,noheader
/usr/local/cuda-12.8/bin/nvcc --version | tail -n 1

ldd "${node_binary}" | rg -q 'libcudart\.so\.12' ||
    fail "NDT 节点未链接 CUDA Runtime。"
ldd "${node_binary}" | rg -q 'libfast_vgicp_cuda\.so' ||
    fail "NDT 节点未链接 fast_gicp CUDA 库。"
/usr/local/cuda-12.8/bin/cuobjdump --list-elf "${cuda_library}" |
    rg -q 'sm_89' ||
    fail "CUDA 库中未找到 RTX 4060 所需的 sm_89 内核。"

echo "[2/6] 加载默认 GPU 环境"
source "${setup_script}"
[[ ${GO2_USE_GPU:-0} == "1" ]] ||
    fail "simdog 环境没有启用 GPU。"
echo "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-未设置}"
echo "NVIDIA PRIME 渲染=${GO2_FORCE_NVIDIA_RENDERING:-0}"

echo "[3/6] 启动 CUDA NDT 节点"
ros2 launch ndt_relocalization ndt_localization.launch.py \
    "map_path:=${sample_map}" \
    use_rviz:=false \
    use_sim_time:=false \
    debug_mode:=false \
    registration_backend:=cuda \
    gpu_device_id:=0 >"${node_log}" 2>&1 &
node_pid=$!

wait_for_log 'CUDA NDT enabled on device 0' 30 ||
    fail "节点未启用 CUDA NDT。"
wait_for_log 'NDT Localization node initialized successfully' 30 ||
    fail "CUDA NDT 节点初始化超时。"
rg 'CUDA NDT enabled|Registration backend|Created .* CUDA NDT' "${node_log}"

echo "[4/6] 注入 TF 和测试点云"
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id odom \
    --child-frame-id base_link >"${temporary_dir}/tf.log" 2>&1 &
tf_pid=$!

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
    -p "file_name:=${sample_scan}" \
    -p publish_rate:=0.05 \
    -p tf_frame:=base_link \
    -r cloud_pcd:=/velodyne_points >"${temporary_dir}/publisher.log" 2>&1 &
publisher_pid=$!

echo "[5/6] 采样 GPU 并检查 ROS 输出"
for _ in $(seq 1 40); do
    nvidia-smi \
        --query-gpu=utilization.gpu,memory.used \
        --format=csv,noheader,nounits >>"${gpu_samples}"
    sleep 0.2
done

timeout 20s ros2 topic echo --once /ndt_pose >"${pose_output}" ||
    fail "未收到 /ndt_pose。"

compute_processes="$(nvidia-smi \
    --query-compute-apps=pid,process_name,used_memory \
    --format=csv,noheader 2>/dev/null || true)"
if [[ ${compute_processes} != *"ndt_relocalization_node"* ]]; then
    fail "NVIDIA 计算进程列表中没有 NDT 节点。"
fi

max_gpu_utilization="$(awk -F',' '
    BEGIN {max = 0}
    {
        gsub(/ /, "", $1)
        if (($1 + 0) > max) {
            max = $1 + 0
        }
    }
    END {print max}
' "${gpu_samples}")"
max_gpu_memory="$(awk -F',' '
    BEGIN {max = 0}
    {
        gsub(/ /, "", $2)
        if (($2 + 0) > max) {
            max = $2 + 0
        }
    }
    END {print max}
' "${gpu_samples}")"

echo "NDT GPU 进程："
echo "${compute_processes}" | rg 'ndt_relocalization_node'
echo "采样峰值：GPU ${max_gpu_utilization}%，显存 ${max_gpu_memory} MiB"
echo "/ndt_pose 摘要："
rg -A4 'position:' "${pose_output}"

echo "[6/6] 验证通过"
echo "CUDA NDT 已完成真实点云配准并发布 /ndt_pose。"
