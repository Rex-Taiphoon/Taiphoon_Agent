#!/bin/bash
# launch_all.sh - 啟動 3 臺 PX4 SITL 模擬器 + MicroXRCEAgent
# 直接於 WSL 內執行即可

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 啟動 3 臺 PX4 SITL 模擬器 ==="

for i in 0 1 2; do
    echo "--- 啟動 Instance ${i} ---"
    bash "${SCRIPT_DIR}/start_single_instance.sh" "${i}"
    sleep 2
done

echo "=== 全部啟動完成 ==="
echo ""
echo "tmux sessions:"
tmux ls 2>/dev/null | grep -E "agent_|sitl_"
echo ""
echo "檢查程序:"
ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep
