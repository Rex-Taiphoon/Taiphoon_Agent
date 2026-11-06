# 地理圍籬設定，GCS 發布於 /uav_all_agent/geofence（reliable、transient_local、depth 1）
# 全體 Agent 共用同一份圍籬；飛行中可隨時設定、替換或清除，各機結果見 GeofenceStatus
uint8 ACTION_SET=1        # 設定或替換圍籬
uint8 ACTION_CLEAR=2      # 停用圍籬

builtin_interfaces/Time stamp     # 發布時間（只供日誌）
uint32 fence_id                   # GCS 指定的版本號，不得為 0；建議每次修改遞增
uint8 action                      # ACTION_SET 或 ACTION_CLEAR
float32 margin_m                  # 機體中心與邊界的最小水平距離（公尺），0～10；CLEAR 時忽略
GeofenceVertex[] vertices         # 3～64 點，依序相連、不可自交、不需重複第一點；CLEAR 時忽略
