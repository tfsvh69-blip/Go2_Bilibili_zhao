#!/usr/bin/env bash

set -euo pipefail

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

export DEBIAN_FRONTEND=noninteractive

echo "[1/4] 更新 apt 索引并准备软件源工具"
apt-get update
apt-get install -y software-properties-common

if ! apt-cache show libgtsam-dev >/dev/null 2>&1; then
    echo "[2/4] 添加 GTSAM 4.1 软件源"
    add-apt-repository -y ppa:borglab/gtsam-release-4.1
    apt-get update
else
    echo "[2/4] GTSAM 软件源已可用"
fi

echo "[3/4] 安装 Go2 项目的构建、仿真、建图与导航依赖"
apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtsam-dev \
    libgtsam-unstable-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-camera-info-manager \
    ros-humble-control-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-ecl-threads \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-image-transport \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-pcl-msgs \
    ros-humble-perception-pcl \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-velodyne \
    ros-humble-velodyne-description \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-vision-opencv \
    ros-humble-xacro

echo "[4/4] 初始化 rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
else
    echo "rosdep 已初始化，跳过 rosdep init。"
fi

invoking_uid="${SUDO_UID:-${PKEXEC_UID:-}}"
if [[ -n ${invoking_uid} && ${invoking_uid} != "0" ]]; then
    invoking_user="$(getent passwd "${invoking_uid}" | cut -d: -f1)"
    invoking_home="$(getent passwd "${invoking_uid}" | cut -d: -f6)"
    if [[ -n ${invoking_user} && -n ${invoking_home} ]]; then
        rosdep_updated=false
        for attempt in 1 2 3; do
            if runuser -u "${invoking_user}" -- env \
                HOME="${invoking_home}" \
                ROS_DISTRO=humble \
                rosdep update --rosdistro humble; then
                rosdep_updated=true
                break
            fi
            echo "rosdep update 第 ${attempt} 次失败，准备重试……" >&2
            sleep 2
        done
        if [[ ${rosdep_updated} != "true" ]]; then
            echo "警告：rosdep 网络更新失败；系统依赖已安装，可稍后手动重试。" >&2
        fi
    fi
else
    echo "未识别到普通用户，请安装完成后自行执行：rosdep update --rosdistro humble"
fi

echo "基础依赖安装完成。"
echo "如需 GPU NDT，执行：bash scripts/install_gpu_dependencies.sh"
echo "然后执行：bash scripts/build_workspaces.sh"
