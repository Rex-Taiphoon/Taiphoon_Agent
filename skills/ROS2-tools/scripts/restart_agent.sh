#!/bin/bash
# restart_agent.sh - 重新啟動指定實例的 MicroXRCEAgent
# 使用方式：bash restart_agent.sh <實例編號>
#   實例編號：0, 1, 2 ...（對應埠號 8888, 8889, 8890 ...）

set -e

if [ $# -lt 1 ]; then
    echo "錯誤：請提供實例編號（0, 1, 2...）"
    echo "使用方式：$0 <實例編號>"
    exit 1
fi

INSTANCE=$1
PORT=$((8888 + INSTANCE))

echo "=== 重新啟動 MicroXRCEAgent 實例 ${INSTANCE}（埠號 ${PORT}）==="

# 終止舊的 tmux session
tmux kill-session -t "agent_${INSTANCE}" 2>/dev/null || true

# 在 tmux 中重新啟動 MicroXRCEAgent
source /opt/ros/humble/setup.bash
tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
tmux send-keys -t "agent_${INSTANCE}" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter

sleep 1
echo "MicroXRCEAgent 實例 ${INSTANCE} 已啟動（埠號 ${PORT}）"
