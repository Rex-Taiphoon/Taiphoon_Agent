#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=3
