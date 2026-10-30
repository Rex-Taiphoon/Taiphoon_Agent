# 單機航點任務狀態，Agent 發布於 /{agent_namespace}/mission/status
# 狀態改變時立即發布，並以 mission_status_rate_hz 週期重送目前狀態

uint8 IDLE=0          # 沒有任務
uint8 LOADED=1        # 已接受上傳，等待 MISSION_START
uint8 RUNNING=2       # 前往 current_index 航點
uint8 DWELLING=3      # 已到達 current_index 航點，停留中
uint8 COMPLETED=4     # 已到達最後一個航點，懸停中
uint8 ABORTED=5       # 執行中被中止
uint8 REJECTED=6      # 上傳或開始請求被拒絕；屬於事件，只對該 mission_id 發布一次

uint8 REASON_NONE=0
uint8 REASON_INVALID_PLAN=1        # 航點數量、座標、高度、停留、航向或高度基準不合法
uint8 REASON_OUT_OF_RANGE=2        # 航點與目前位置的水平距離超過上限
uint8 REASON_NOT_READY=3           # 尚無有效狀態，或 Home／PX4 local 換算尚未建立
uint8 REASON_NOT_HOVERING=4        # 未解鎖、已落地或不在懸停
uint8 REASON_BUSY=5                # 已有任務執行中，或編隊中
uint8 REASON_ID_MISMATCH=6         # MISSION_START 的識別碼與已載入任務不符，或載入已逾時
uint8 REASON_OPERATOR_STOP=10      # 收到 MISSION_STOP
uint8 REASON_COMMAND_OVERRIDE=11   # 收到其他命令（操作人員介入）
uint8 REASON_FLIGHT_STATE=12       # 飛行狀態離開懸停／移動（RTL、LAND、ERROR 等）
uint8 REASON_LANDED=13             # 已落地或上鎖
uint8 REASON_LEG_TIMEOUT=14        # 單段逾時未到達

builtin_interfaces/Time stamp
uint32 mission_id          # 狀態所屬任務；IDLE 時為 0
uint8 state
uint8 reason               # REJECTED／ABORTED 的原因，其他狀態為 REASON_NONE
uint16 current_index       # 目前航點（0 起算）；INVALID_PLAN／OUT_OF_RANGE 時指向第一個不合法航點
uint16 waypoint_count
string detail              # 英文補充說明，供日誌使用（例如觸發中止的命令名稱）
