# Agent 與 PX4 連線健康狀態，basic_controller_node 以 health_publish_rate_hz（預設 1 Hz）發布於 /{agent_namespace}/health
# *_age_s 為距離最後一筆訊息的秒數；從未收到為 -1

uint32 FAILSAFE_LOCAL_POSITION_INVALID=1
uint32 FAILSAFE_GLOBAL_POSITION_INVALID=2
uint32 FAILSAFE_HOME_POSITION_INVALID=4
uint32 FAILSAFE_ATTITUDE_INVALID=8
uint32 FAILSAFE_MANUAL_CONTROL_LOST=16
uint32 FAILSAFE_OFFBOARD_SIGNAL_LOST=32
uint32 FAILSAFE_BATTERY_WARNING=64
uint32 FAILSAFE_BATTERY_UNHEALTHY=128
uint32 FAILSAFE_GEOFENCE_BREACHED=256
uint32 FAILSAFE_FD_CRITICAL_FAILURE=512
uint32 FAILSAFE_FD_ESC_ARMING_FAILURE=1024
uint32 FAILSAFE_LOCAL_POSITION_ACCURACY_LOW=2048

builtin_interfaces/Time stamp
float32 px4_status_age_s              # VehicleStatus
float32 px4_odometry_age_s            # VehicleOdometry
float32 px4_global_position_age_s     # VehicleGlobalPosition
float32 px4_land_detected_age_s       # VehicleLandDetected
float32 px4_battery_age_s             # BatteryStatus
float32 px4_command_ack_age_s         # VehicleCommandAck；平常沒有命令時會很大，-1 表示從未收到
bool px4_pre_flight_checks_pass       # 以下 px4_* 欄位只在 VehicleStatus 新鮮時可信
bool px4_failsafe
uint8 px4_arming_state
uint8 px4_latest_arming_reason
bool failsafe_flags_valid             # 有收到新鮮的 fmu/out/failsafe_flags 才為 true
uint32 failsafe_flags                 # 依上方 FAILSAFE_* 位元整理
bool offboard_session_active
