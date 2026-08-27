# 1. OFFBOARD 模式
docker exec uav_1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 2 --once \
    /uav_all_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'"

sleep 3

# 2. 解鎖
docker exec uav_1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 2 --once \
    /uav_all_agent/command std_msgs/msg/String 'data: ARM'"

sleep 2

# 3. 起飛至指定高度（未提供參數時維持既有 8m 行為）
TAKEOFF_ALTITUDE_M="${1:-8.0}"
docker exec uav_1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 2 --once \
    /uav_all_agent/command std_msgs/msg/String 'data: TAKEOFF ${TAKEOFF_ALTITUDE_M}'"
