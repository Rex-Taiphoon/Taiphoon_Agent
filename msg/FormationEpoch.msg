# Leaderless 編隊共同 snapshot 與收斂狀態
builtin_interfaces/Time creation_stamp       # epoch 建立時間
uint32 sequence                               # 單調遞增 epoch 序號
uint32 parent_sequence                        # 提案所依據的 parent 序號
uint32 parent_snapshot_hash                   # 提案所依據的 parent identity
uint32 snapshot_hash                          # 成員與設定的 deterministic hash
int32 proposer_agent_id                       # 提案者 Agent ID
int32 sender_agent_id                         # 本次訊息發布者 Agent ID
uint8 PROPOSAL=0                              # 尚未由所有成員採用
uint8 ADOPTED=1                               # sender 已採用此 snapshot
uint8 status                                  # PROPOSAL 或 ADOPTED
uint8 CIRCLE=0                                # formation_type
uint8 HORI_LINEAR=1
uint8 VERT_LINEAR=2
uint8 formation_type
float64 radius_m                              # CIRCLE 幾何參數
float64 spacing_m                             # 線性編隊間距
float64 vertical_offset_m                     # 編隊相對中心的 NED down offset
float64 center_latitude_deg                    # WGS84 latitude
float64 center_longitude_deg                   # WGS84 longitude
float64 center_altitude_m                      # WGS84 ellipsoid/AMSL contract
int32[] member_agent_ids                       # 排序後成員 ID
float64[] member_latitudes_deg                 # 成員 snapshot WGS84 latitude
float64[] member_longitudes_deg                # 成員 snapshot WGS84 longitude
float64[] member_altitudes_m                   # 成員 snapshot WGS84 altitude
int32[] parent_assignment_agent_ids            # parent epoch ownership
int32[] parent_assignment_slot_indices
bool valid
