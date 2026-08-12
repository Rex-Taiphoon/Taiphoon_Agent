# EGO-Swarm 對外廣播的 Agent 全球未來軌跡
# control_points 使用 WGS84 geodetic [latitude_deg, longitude_deg, height_m]。
# 接收端必須先轉成本機固定 Home 的 metric NED，再建立 B-spline。
uint8 COORDINATE_FRAME_WGS84_GEODETIC=1
builtin_interfaces/Time stamp                 # 發布時間
builtin_interfaces/Time start_time            # 軌跡起始時間
uint32 trajectory_id                           # 單調遞增的軌跡識別碼
uint32 agent_index                             # 發布者 Agent 索引
uint8 coordinate_frame                         # 全球座標格式；目前固定 WGS84 geodetic
uint32 frame_revision                          # 全球座標 contract 版本
uint8 order                                    # B-spline 階數
float32 knot_interval_s                        # knot 間隔（秒）
float64[] control_points                       # [lat,lon,height] 控制點；每 3 個 double 一組
float32 duration_s                             # 軌跡有效時間（秒）
bool terminal_hold                             # true 表示完成後固定佔位，需持續 heartbeat
bool valid                                     # 軌跡是否可供同儕使用
