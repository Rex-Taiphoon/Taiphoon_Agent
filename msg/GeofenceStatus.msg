# 地理圍籬狀態，Agent 發布於 /{agent_namespace}/geofence/status（reliable、transient_local、depth 1）
# 狀態改變時發布；GCS 重啟後可由此還原目前生效的圍籬
uint8 STATE_DISABLED=0    # 沒有圍籬
uint8 STATE_PENDING=1     # 已接受，等待 planner Home 建立後生效
uint8 STATE_ACTIVE=2      # 生效中

uint8 RESULT_NONE=0               # 尚未收到任何設定
uint8 RESULT_ACCEPTED=1           # 已接受（含內容相同的重送）
uint8 RESULT_REJECTED_INVALID=2   # fence_id、action、margin、頂點數量或座標不合法、自交或面積過小
uint8 RESULT_REJECTED_TOO_FAR=3   # 有頂點距 planner Home 超過 5 km

builtin_interfaces/Time stamp
uint8 state
uint32 fence_id                   # 目前生效或等待中的圍籬版本；DISABLED 時為 0
float32 margin_m                  # 目前圍籬的 margin；DISABLED 時為 0
GeofenceVertex[] vertices         # 目前圍籬的頂點；DISABLED 時為空
uint32 last_request_id            # 最近一次處理的 Geofence.fence_id
uint8 last_result                 # 最近一次處理的結果；拒絕時保留原本的圍籬
bool position_inside              # 目前位置在圍籬（含 margin）內；沒有圍籬或沒有位置時為 true
string detail                     # 英文補充說明，供日誌使用
