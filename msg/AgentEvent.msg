# Agent 事件（錯誤、警告、資訊），發布於 /{agent_namespace}/events
# basic_controller_node 與 ego_swarm_planner 都會發布到同一個 topic

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_ERROR=2

uint8 SOURCE_BASIC_CONTROLLER=1
uint8 SOURCE_PLANNER=2
uint8 SOURCE_SUPERVISOR=3

builtin_interfaces/Time stamp
uint32 sequence           # 各來源節點各自單調遞增，供 GCS 偵測遺失與去重
uint8 severity
uint8 source
uint16 code               # 事件代碼，見 docs/taiphoon-agent-interface.md 事件代碼表
string command            # 相關命令名稱（例如 ARM、REBOOT_FC），沒有時為空字串
int16 px4_result          # 相關的 PX4 VehicleCommandAck.result；沒有時為 -1
string detail             # 英文補充說明，供日誌與未知代碼顯示
