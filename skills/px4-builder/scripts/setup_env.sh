#!/bin/bash
# setup_env.sh - 安裝 PX4 建構所需的系統相依套件
# 使用方式：bash setup_env.sh [--install-agent]
#   --install-agent：同時安裝 MicroXRCEAgent

set -e

INSTALL_AGENT=false
if [ "$1" = "--install-agent" ]; then
    INSTALL_AGENT=true
fi

echo "=== 安裝 PX4 建構相依套件 ==="

# 更新套件清單
sudo apt-get update

# 安裝基本編譯工具
sudo apt-get install -y \
    git cmake ninja-build build-essential ccache g++ \
    python3 python3-pip python3-venv python3-jinja2 \
    python3-toml python3-numpy python3-empy python3-packaging \
    python3-wheel python3-setuptools \
    curl wget unzip tmux

# 安裝 PX4 特定相依套件
sudo apt-get install -y \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools \
    liblog4cxx-dev libzmq3-dev \
    libncurses5-dev libncursesw5-dev

# 安裝 Python 套件
pip3 install --user jinja2 toml numpy empy packaging kconfiglib

# 安裝 ROS2 Humble（若尚未安裝）
if [ ! -d "/opt/ros/humble" ]; then
    echo "=== 安裝 ROS2 Humble ==="
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base
    source /opt/ros/humble/setup.bash
    echo "ROS2 Humble 安裝完成"
else
    echo "ROS2 Humble 已安裝"
fi

# 安裝 MicroXRCEAgent（若指定）
if [ "$INSTALL_AGENT" = true ]; then
    echo "=== 安裝 MicroXRCEAgent ==="
    if command -v MicroXRCEAgent &>/dev/null; then
        echo "MicroXRCEAgent 已安裝"
    else
        cd /tmp
        git clone --recursive https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        cd Micro-XRCE-DDS-Agent
        mkdir -p build && cd build
        cmake ..
        cmake --build .
        sudo cmake --install .
        sudo cp MicroXRCEAgent /usr/local/bin/
        echo "MicroXRCEAgent 安裝完成"
    fi
fi

echo "=== 環境設定完成 ==="
echo "請執行 build_px4.sh 以開始建構 PX4"
