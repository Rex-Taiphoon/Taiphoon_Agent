# 航點任務的單一航點
# 水平位置使用 WGS84；高度基準由 WaypointMission.altitude_frame 決定

float64 latitude_deg      # WGS84 緯度（度）
float64 longitude_deg     # WGS84 經度（度）
float32 altitude_m        # 高度（公尺，向上為正），基準見 WaypointMission.altitude_frame
float32 dwell_s           # 到達後停留時間（秒）；最後一個航點不使用，到達即完成並懸停
float32 yaw_deg           # 保留欄位：NaN 表示維持目前目標航向；本版只接受 NaN
