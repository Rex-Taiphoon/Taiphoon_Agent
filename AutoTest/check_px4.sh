#!/bin/bash
# Kill all background publishers
pkill -f 'ros2 topic pub' 2>/dev/null
sleep 1

# Check PX4 console for arm/offboard status
echo "=== PX4 Console (sitl_0) ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -20
echo ""
echo "=== Sending commander arm -f ==="
tmux send-keys -t sitl_0 "commander arm -f" Enter
sleep 3
echo ""
echo "=== Console after arm -f ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -10