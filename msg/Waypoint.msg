# Taiphoon Agent 軌跡 Waypoint 訊息
# 單一 waypoint，包含 NED 位置與到達時間

float32[3] position_ned           # NED 位置（前向/右向/向下，公尺）
builtin_interfaces/Time arrival_time  # 預定到達時間
