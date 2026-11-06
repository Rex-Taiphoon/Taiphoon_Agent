# EGO-Swarm Planner 狀態與安全指標
uint8 STATE_IDLE=0                             # 保留，目前不發布
uint8 STATE_RUNNING=1                          # 正沿軌跡飛往目標
uint8 STATE_HOLD=2                             # 沒有在移動：懸停、等待或煞停中，原因見 reason
uint8 STATE_ERROR=3                            # 無法產生煞停軌跡，需操作人員介入

builtin_interfaces/Time stamp                 # 狀態時間戳
uint8 state                                    # 規劃器狀態，見 STATE_*
uint32 trajectory_id                           # 目前軌跡識別碼
bool map_valid                                 # 障礙物地圖是否有效
bool localization_valid                        # 本 Agent 的 planner-local frame 是否有效
bool peer_data_valid                            # 同儕資料是否可用
float32 planning_time_ms                       # 最近一次規劃耗時
float32 min_obstacle_distance_m                # 最近障礙物距離
float32 min_peer_distance_m                    # 最近同儕中心距離
string reason                                  # 降級或失敗原因
