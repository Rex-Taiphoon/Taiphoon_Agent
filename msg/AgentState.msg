# Taiphoon Agent 對外狀態訊息
# 發布於 /{agent_namespace}/state，頻率由 state_publish_rate_hz 參數控制（預設 10Hz）

builtin_interfaces/Time stamp   # 訊息時間戳
float32[3] position_ned         # NED 座標位置（前向/右向/向下，公尺）
float64[3] position_global      # 全域 GPS 座標 [lat(deg), lon(deg), alt(m AMSL)]，無資料時為 NaN
uint8 flight_mode               # 經 PX4 觀測校正後的任務階段（對應 FlightState 枚舉值）
float32 battery_remaining       # 剩餘電量比例（0.0 ~ 1.0），無資料時為 1.0
float32 battery_voltage_v       # 電池電壓 (V)，無資料時為 NaN
float32 battery_current_a       # 電池電流 (A)，無資料時為 NaN
bool online                     # Agent 是否在線
bool armed                      # PX4 是否已解鎖
bool landed                     # PX4 是否已落地
uint8 px4_nav_state             # PX4 VehicleStatus.nav_state 原始值
bool px4_state_valid            # PX4 狀態欄位是否有效
