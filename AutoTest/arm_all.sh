#!/bin/bash
# Arm all 3 PX4 instances and switch to offboard mode
for i in 0 1 2; do
    echo "Setting up instance $i..."
    tmux send-keys -t sitl_$i "commander mode offboard" Enter
    sleep 1
    tmux send-keys -t sitl_$i "commander arm" Enter
    sleep 0.5
done
echo "All instances armed in offboard mode"