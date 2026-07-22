#!/bin/bash
# validate_build.sh - 驗證 PX4 建構結果與執行環境
# 使用方式：bash validate_build.sh

set -e

PX4_BIN=~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ERRORS=0

echo "=== PX4 建構環境驗證 ==="
echo ""

# 1. 檢查 PX4 執行檔
echo "1. PX4 執行檔..."
if [ -f "$PX4_BIN" ]; then
    echo "   [OK] 執行檔存在：${PX4_BIN}"
    file "$PX4_BIN"
else
    echo "   [FAIL] 執行檔不存在：${PX4_BIN}"
    echo "   請執行 build_px4.sh 重新建構"
    ERRORS=$((ERRORS + 1))
fi

# 2. 檢查 PX4 執行檔是否可執行
if [ -f "$PX4_BIN" ] && [ ! -x "$PX4_BIN" ]; then
    echo "   [WARN] 執行檔無執行權限，正在修復..."
    chmod +x "$PX4_BIN"
fi

# 3. 檢查 MicroXRCEAgent
echo ""
echo "2. MicroXRCEAgent..."
if command -v MicroXRCEAgent &>/dev/null; then
    echo "   [OK] 已安裝：$(which MicroXRCEAgent)"
else
    echo "   [FAIL] MicroXRCEAgent 未安裝"
    echo "   請執行 setup_env.sh --install-agent 安裝"
    ERRORS=$((ERRORS + 1))
fi

# 4. 檢查 ROS2 Humble
echo ""
echo "3. ROS2 Humble..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "   [OK] ROS2 Humble 環境存在"
else
    echo "   [FAIL] ROS2 Humble 未安裝"
    echo "   請執行 setup_env.sh 安裝"
    ERRORS=$((ERRORS + 1))
fi

# 5. 檢查 tmux
echo ""
echo "4. tmux..."
if command -v tmux &>/dev/null; then
    echo "   [OK] tmux 已安裝：$(tmux -V)"
else
    echo "   [WARN] tmux 未安裝"
    echo "   建議安裝 tmux 以管理多個模擬器工作階段"
fi

# 6. 檢查 git
echo ""
echo "5. git..."
if command -v git &>/dev/null; then
    echo "   [OK] git 已安裝：$(git --version)"
else
    echo "   [FAIL] git 未安裝"
    ERRORS=$((ERRORS + 1))
fi

# 7. 檢查 PX4-Autopilot 目錄
echo ""
echo "6. PX4-Autopilot 原始碼..."
if [ -d ~/PX4-Autopilot ]; then
    echo "   [OK] 目錄存在"
    cd ~/PX4-Autopilot
    echo "   目前分支：$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '無法取得')"
else
    echo "   [WARN] 目錄不存在，尚未複製原始碼"
fi

# 總結
echo ""
echo "=== 驗證結果 ==="
if [ $ERRORS -eq 0 ]; then
    echo "所有檢查通過！環境已準備就緒。"
    exit 0
else
    echo "發現 ${ERRORS} 個問題需處理，請依照上方提示修復。"
    exit 1
fi
