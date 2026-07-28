# Taiphoon Agent NED 絕對目標位置訊息
# 用於 formation_node → basic_controller_node 之間的高頻位置指令

float32 x     # NED X 軸位置 (前向為正，公尺)
float32 y     # NED Y 軸位置 (右向為正，公尺)
float32 z     # NED Z 軸位置 (向下為正，公尺)
float32 yaw   # 目標航向角 (度，順時針為正)，NaN 表示不控制偏航
