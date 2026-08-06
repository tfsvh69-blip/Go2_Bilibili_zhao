#!/usr/bin/env bash

set -euo pipefail

cuda_version="12-8"
cuda_root="/usr/local/cuda-12.8"
keyring_url="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb"

if [[ ${EUID} -ne 0 ]]; then
    echo "需要管理员权限，正在通过 sudo 重新执行……"
    exec sudo -- "$0" "$@"
fi

if [[ ! -r /etc/os-release ]]; then
    echo "无法识别操作系统。" >&2
    exit 1
fi

source /etc/os-release
if [[ ${ID:-} != "ubuntu" || ${VERSION_ID:-} != "22.04" ]]; then
    echo "此脚本仅支持 Ubuntu 22.04，当前系统为 ${PRETTY_NAME:-未知}。" >&2
    exit 1
fi

if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "未找到 nvidia-smi，请先安装可用的 NVIDIA 驱动。" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive

echo "[1/4] 准备 NVIDIA CUDA 软件源"
apt-get update
apt-get install -y ca-certificates curl

if ! dpkg-query -W cuda-keyring >/dev/null 2>&1; then
    temporary_dir="$(mktemp -d /tmp/go2_cuda_keyring.XXXXXX)"
    curl -fL --retry 3 \
        -o "${temporary_dir}/cuda-keyring.deb" \
        "${keyring_url}"
    dpkg -i "${temporary_dir}/cuda-keyring.deb"
    find "${temporary_dir}" -type f -delete
    find "${temporary_dir}" -depth -type d -empty -delete
fi

echo "[2/4] 安装 CUDA ${cuda_version} 编译器和最小运行库"
apt-get update
apt-get install -y \
    "cuda-compiler-${cuda_version}" \
    "cuda-cudart-dev-${cuda_version}" \
    "libcublas-dev-${cuda_version}"

echo "[3/4] 验证 CUDA 编译器与 GPU"
"${cuda_root}/bin/nvcc" --version
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

echo "[4/4] CUDA 环境完成"
echo "下一步执行：bash scripts/build_workspaces.sh"
