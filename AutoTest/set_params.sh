#!/bin/bash
for i in 0 1 2; do
    tmux send-keys -t sitl_$i "param set COM_ARM_WO_GPS 1" Enter
    sleep 0.3
    tmux send-keys -t sitl_$i "param set COM_RCL_EXCEPT 4" Enter
    sleep 0.3
    tmux send-keys -t sitl_$i "param set NAV_RCL_ACT 0" Enter
    sleep 0.3
    echo "Params set on instance $i"
done