#!/bin/bash
# stop_all.sh - 關閉所有 PX4 SITL 模擬器與 MicroXRCEAgent

echo "=== 關閉所有 PX4 SITL 與 MicroXRCEAgent ==="

# Kill tmux sessions
for i in 0 1 2; do
    tmux kill-session -t "agent_${i}" 2>/dev/null || true
    tmux kill-session -t "sitl_${i}" 2>/dev/null || true
done

# Kill processes directly as backup
pkill -f "MicroXRCEAgent udp4" 2>/dev/null || true
pkill -f "px4 -d -i" 2>/dev/null || true

echo "Done. 確認程序:"
sleep 1
ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep || echo "(all clear)"
