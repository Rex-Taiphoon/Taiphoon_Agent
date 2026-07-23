#!/bin/bash
sleep 8
echo "=== Verify params in PX4 ==="
tmux send-keys -t sitl_0 "param show COM_ARM_WO_GPS" Enter
sleep 0.5
tmux send-keys -t sitl_0 "param show COM_RCL_EXCEPT" Enter
sleep 0.5
tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -10