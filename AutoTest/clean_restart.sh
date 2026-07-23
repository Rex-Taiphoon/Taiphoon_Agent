#!/bin/bash
# Kill everything
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f 'ros2 topic' 2>/dev/null || true
tmux kill-server 2>/dev/null || true
sleep 3

# Clean rootfs
rm -rf "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/" 2>/dev/null || true

# Restart with params
SCRIPT_DIR="/home/taiphoon/Taiphoon_Agent/skills/px4-sitl-launcher/scripts"
bash "$SCRIPT_DIR/launch_all.sh" 3

echo "PX4 instances restarted"