#!/bin/bash
# build_px4.sh - 複製 PX4 原始碼並建構 SITL 版本
# 使用方式：bash build_px4.sh [分支名稱]
#   分支名稱：預設為 main，可指定特定版本標籤或分支

set -e

PX4_DIR=~/PX4-Autopilot
BRANCH=${1:-main}

echo "=== 開始建構 PX4 SITL ==="

# 若目錄不存在，從 GitHub 複製原始碼
if [ ! -d "$PX4_DIR" ]; then
    echo "PX4-Autopilot 目錄不存在，正在從 GitHub 複製..."
    cd ~
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    echo "原始碼複製完成"
else
    echo "PX4-Autopilot 目錄已存在"
fi

cd "$PX4_DIR"

# 切換至指定分支
echo "切換至分支：${BRANCH}"
git checkout "$BRANCH" 2>/dev/null || echo "分支 ${BRANCH} 已存在或無法切換"

# 更新子模組
git submodule update --recursive --init

# 執行 PX4 自動相依性安裝腳本
echo "=== 執行 PX4 自動相依性安裝 ==="
if [ -f "Tools/setup/ubuntu.sh" ]; then
    bash Tools/setup/ubuntu.sh
fi

# 建構 SITL 版本
echo "=== 編譯 px4_sitl ==="
make px4_sitl

echo "=== 建構完成 ==="
echo "執行檔位置：${PX4_DIR}/build/px4_sitl_default/bin/px4"

# 驗證執行檔
if [ -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
    echo "建構成功！執行檔已產生"
    ls -lh "${PX4_DIR}/build/px4_sitl_default/bin/px4"
else
    echo "錯誤：執行檔未產生，建構可能失敗"
    exit 1
fi
