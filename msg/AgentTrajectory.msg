# EGO-Swarm 對外廣播的 Agent 未來軌跡
builtin_interfaces/Time stamp                 # 發布時間
builtin_interfaces/Time start_time            # 軌跡起始時間
uint32 trajectory_id                           # 單調遞增的軌跡識別碼
uint32 agent_index                             # 發布者 Agent 索引
uint32 frame_revision                          # swarm_ned 座標版本
uint8 order                                    # B-spline 階數
float32 knot_interval_s                        # knot 間隔（秒）
float32[] control_points                       # 扁平化 swarm_ned 控制點；每 3 個 float 為 [x,y,z]
float32 duration_s                             # 軌跡有效時間（秒）
float64[3] frame_origin_global                 # 共同座標原點 [lat, lon, alt]
bool valid                                     # 軌跡是否可供同儕使用
