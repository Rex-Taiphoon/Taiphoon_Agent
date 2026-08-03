# 1. OFFBOARD 模式
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --once \
    /fmu_all_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'"

sleep 3

# 2. 解鎖
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --once \
    /fmu_all_agent/command std_msgs/msg/String 'data: ARM'"

sleep 2

# 3. 起飛 8m
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --once \
    /fmu_all_agent/command std_msgs/msg/String 'data: TAKEOFF 8.0'"