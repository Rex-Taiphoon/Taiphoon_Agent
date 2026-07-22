#!/bin/bash
# stop_all.sh - 停止所有 PX4 SITL 模擬器與 MicroXRCEAgent
# 使用方式：bash stop_all.sh
#   不須參數，會自動停止所有執行中的實例

echo "=== 停止所有 PX4 SITL 與 MicroXRCEAgent ==="

# 終止所有 tmux 工作階段（agent_* 與 sitl_*）
tmux ls 2>/dev/null | grep -E "^agent_|^sitl_" | cut -d: -f1 | while read session; do
    tmux kill-session -t "$session" 2>/dev/null || true
done

# 直接終止所有相關程序
pkill -f "MicroXRCEAgent udp4" 2>/dev/null || true
pkill -f "px4 -d -i" 2>/dev/null || true

echo "清理完成。"
sleep 1

# 檢查是否還有殘留程序
REMAINING=$(ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep || true)
if [ -z "$REMAINING" ]; then
    echo "(所有程序已停止)"
else
    echo "殘留程序："
    echo "$REMAINING"
fi