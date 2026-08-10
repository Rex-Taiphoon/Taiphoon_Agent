# 未來相對定位 Adapter 使用的單一同儕觀測
builtin_interfaces/Time stamp                 # 觀測時間戳
uint32 agent_index                             # 同儕 Agent 索引
float32[3] relative_position_ned               # 相對 NED 位置（公尺）
float32[3] relative_velocity_ned               # 相對 NED 速度（公尺/秒）
float32 confidence                             # 觀測信心度（0.0 ~ 1.0）
bool valid                                     # 觀測是否有效
