# 單機航點任務上傳，GCS 發布於 /{agent_namespace}/mission/plan
# 上傳只載入任務、不會移動；需另送 MISSION_START <mission_id> 才開始

uint8 ALTITUDE_PX4_LOCAL=1        # 高度與 -AgentState.position_ned[2] 同基準（PX4 local 原點起算）

builtin_interfaces/Time stamp     # 發布時間
uint32 mission_id                 # GCS 指定的任務識別碼，不得為 0
uint8 altitude_frame              # 高度基準；本版只接受 ALTITUDE_PX4_LOCAL（0 視為未設定而拒絕）
MissionWaypoint[] waypoints       # 依序執行的航點
