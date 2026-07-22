#!/bin/bash
# launch_all.sh - 啟動指定數量的 PX4 SITL 模擬器與 MicroXRCEAgent
# 使用方式：bash launch_all.sh [無人機數量]
#   無人機數量：預設為 3，可依需求調整

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 讀取無人機數量參數，預設為 3
COUNT=${1:-3}

# 啟動前清理任何殘留的 tmux session 與程序，避免埠號衝突
echo "=== 清理舊有模擬器 ==="
"${SCRIPT_DIR}/stop_all.sh" 2>/dev/null || true
sleep 1
echo ""

echo "=== 啟動 ${COUNT} 台 PX4 SITL 模擬器 ==="

for ((i=0; i<COUNT; i++)); do
    echo "--- 啟動 Instance ${i} ---"
    bash "${SCRIPT_DIR}/start_single_instance.sh" "${i}"
    sleep 2
done

echo "=== 全部 ${COUNT} 台已啟動完成 ==="
echo ""
echo "tmux 工作階段："
tmux ls 2>/dev/null | grep -E "agent_|sitl_" || echo "(無執行中的 tmux 工作階段)"
echo ""
echo "執行中程序："
ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep || echo "(無執行中的 PX4 程序)"