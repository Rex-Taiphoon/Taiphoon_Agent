# Taiphoon Agent BasicController 操作手冊

## 系統架構說明

BasicController 與 FormationNode 是 Taiphoon Agent 的兩大核心模組：

- **BasicController**：單機飛行控制，處理 Offboard 模式下的起降、移動、返航、解鎖/上鎖等命令
- **FormationNode**：編隊計算節點（多機時自動啟動），接收 FORMATION_* 命令，計算自身 NED 目標位置

系統採用 SOLID 原則設計，包含以下層次：

- **核心介面層（`core/`）**：IController、IBasicFlightController 定義控制器抽象介面
- **工具層（`utils/`）**：math_utils 提供數學比較與數值合法性檢查；CommandTypes 提供命令解析
- **控制器層（`controllers/`）**：BasicController 實作起降邏輯與 PX4 Offboard 控制；AgentCommandHandler 負責命令路由
- **節點層（`nodes/`）**：BasicControllerNode 封裝 ROS2 通訊；FormationNode 封裝編隊運算

### 狀態機流程

```
IDLE -> OFFBOARD_INIT -> OFFBOARD_ACTIVE -> ARMED -> TAKEOFF -> HOVER
                                                             \-> MOVING
                                                             \-> RTL -> POST_RTL
                                                             \-> LAND
```

## 啟動步驟

### Launch 檔案（多機模式）

```bash
# 先載入環境
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash

# Launch（4 台無人機）
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4
```

當 `drone_count` > 1 時，Launch 會為每台無人機建立一組 **BasicController** + **FormationNode**，
並自動設定命名空間與啟動延遲（多機依索引錯開 2 秒）。

### 單獨啟動

```bash
ros2 run taiphoon_agent basic_controller --ros-args \
    -r __ns:=/uav_1 \
    -p target_altitude_m:=3.0 \
    -p hover_duration_s:=3.0

ros2 run taiphoon_agent formation_node --ros-args \
    -r __ns:=/uav_1 \
    -p drone_count:=4.0 \
    -p agent_namespace:=uav_1_agent
```

## 支援的高階命令

以下命令可透過 `/uav_{N}_agent/command`（單機）或 `/uav_all_agent/command`（全體廣播）發送。

### 基本飛行命令（BasicController 處理）

| 命令 | 完整格式 | 說明 |
|------|----------|------|
| `MODE_OFFBOARD` | `MODE_OFFBOARD` | 切換至 Offboard 模式 |
| `MODE_RTL` | `MODE_RTL` | 切換至返航模式 |
| `MODE_LAND` | `MODE_LAND` | 切換至降落模式 |
| `ARM` | `ARM` | 解鎖 |
| `DISARM` | `DISARM` | 上鎖 |
| `TAKEOFF` | `TAKEOFF [高度]` | 起飛至指定高度（公尺），無參數時使用預設值 3.0m；高度不是數值時不執行 |
| `HOLD` | `HOLD` | 保持目前位置懸停 |
| `MOVE_FORWARD` | `MOVE_FORWARD [距離]` | 向前移動指定距離（公尺） |
| `MOVE_BACKWARD` | `MOVE_BACKWARD [距離]` | 向後移動指定距離（公尺） |
| `MOVE_LEFT` | `MOVE_LEFT [距離]` | 向左移動指定距離（公尺） |
| `MOVE_RIGHT` | `MOVE_RIGHT [距離]` | 向右移動指定距離（公尺） |
| `MOVE_UP` | `MOVE_UP [距離]` | 向上移動指定距離（公尺），相對位移 |
| `MOVE_DOWN` | `MOVE_DOWN [距離]` | 向下移動指定距離（公尺），最低不低於 1m |
| `YAW` | `YAW [角度]` | 順時針旋轉指定角度（度），累加到目標航向；以 `yaw_rate_limit_deg_s`（預設 30 deg/s）限速轉向 |
| `MOVE_POINT` | `MOVE_POINT x y z yaw` | 移動至指定 NED 絕對位置（x/y/z/yaw）；yaw 同樣限速轉向 |
| `MISSION_START` | `MISSION_START <任務識別碼>` | 轉交 ego_swarm_planner 開始已上傳的航點任務；僅單機 topic |
| `MISSION_STOP` | `MISSION_STOP` | 轉交 ego_swarm_planner 中止航點任務；僅單機 topic |
| `REBOOT_FC` | `REBOOT_FC [0\|1]` | 重新啟動飛控（VehicleCommand 246，param1=1、param2=0）；僅單機 topic，安全條件見下方 |

> 參數缺少或不合法（例如 `TAKEOFF abc`、`YAW` 無參數）時不執行，並發布 `COMMAND_INVALID_PARAMS` 事件；
> 舊版 `TAKEOFF abc` 會以預設高度起飛，現在改為拒絕。全體 topic 收到 `MISSION_*` 或 `REBOOT_FC`
> 時忽略並發布 `COMMAND_REJECTED`。

### 編隊命令（FormationNode 處理，僅限 `/uav_all_agent/command`）

| 命令 | 完整格式 | 說明 |
|------|----------|------|
| `FORMATION_CIRCLE` | `FORMATION_CIRCLE 半徑 高度` | 環形編隊，等角分布 |
| `FORMATION_HORI_LINEAR` | `FORMATION_HORI_LINEAR 間距 高度` | 水平直列，沿 Y 軸排列 |
| `FORMATION_VERT_LINEAR` | `FORMATION_VERT_LINEAR 間距` | 垂直直列，沿 Z 軸排列 |
| `FORMATION_CANCEL` | `FORMATION_CANCEL` | 取消當前編隊任務 |

### 簡寫命令

| 完整形式 | 簡寫 |
|----------|------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

> 命令不區分大小寫。MOVE_UP / MOVE_DOWN / YAW / MOVE_POINT 需在飛行狀態（HOVER / TAKEOFF / MOVING）下執行。

## Agent 容器失效模擬（高風險）

`agent_supervisor` 提供單一 Agent 的 `/{agent_namespace}/shutdown` service，型別為
`std_srvs/srv/SetBool`。`request.data` 必須為 `true`；服務回應送出後會立即終止 Agent
容器 PID 1（SIGTERM 觸發立即退出），刻意模擬飛行電腦失效。

此操作不檢查飛行狀態、不執行 RTL/LAND，也不允許 `/uav_all_agent/shutdown` 廣播。後續行為
完全由 PX4 failsafe 決定。

```bash
# 在 Agent 容器內確認 service
ros2 service list | grep -F /uav_1_agent/shutdown
ros2 service type /uav_1_agent/shutdown
```

確認完成後，在另一個終端執行；回應送出後容器會終止：

```bash
docker compose exec uav_1_agent bash -c \
  'ros2 service call /uav_1_agent/shutdown std_srvs/srv/SetBool "{data: true}"'
```

若 Compose 服務設定為 `restart: no`，PID 1 終止後容器會保持停止；若設定自動重啟策略，
容器可能重新啟動，無法保持失效狀態。回到宿主機的另一個終端確認容器狀態：

```bash
docker compose ps uav_1_agent
docker inspect --format '{{.State.Status}} exit={{.State.ExitCode}} restart={{.RestartCount}}' uav_1_agent
```

## 依賴套件

- ROS2 Humble
- px4_msgs
- PX4-Autopilot（SIH 模擬器）

## 建置步驟（容器內）

```bash
# 1. 載入 ROS2 環境
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash   # px4_msgs

# 2. 進入專案目錄
cd ~/Taiphoon_Agent

# 3. 建置
colcon build --packages-select taiphoon_agent

# 4. 載入工作區
source install/setup.bash
```

## 修改參數

可透過以下方式修改起飛高度與停懸時間：

### 命令列參數

```bash
ros2 run taiphoon_agent basic_controller --ros-args \
    -p target_altitude_m:=5.0 \
    -p hover_duration_s:=10.0 \
    -p yaw_rate_limit_deg_s:=30.0
```

Launch 也接受同名參數，例如 `ros2 launch taiphoon_agent basic_controller.launch.py yaw_rate_limit_deg_s:=45.0`。

### FormationNode 參數命令列

```bash
ros2 run taiphoon_agent formation_node --ros-args \
    -p drone_count:=4.0 \
    -p agent_namespace:=uav_1_agent \
    -p formation_update_rate_hz:=5.0
```

## 預期行為（單機）

啟動後，無人機將自動依序執行：

1. **解鎖（Arm）**：發送 VehicleCommand（CMD=400, param1=1）
2. **起飛（Takeoff）**：以 10Hz 頻率發送 OffboardControlMode + TrajectorySetpoint
3. **抵達目標高度後自動停懸**：維持高度直到秒數達標
4. **降落（Land）**：設定目標高度為 0，降至地面後進入 IDLE

## PX4 狀態新鮮度與 Offboard 中止

BasicController 以 PX4 的 `VehicleStatus` 確認 Offboard 狀態，並以 `VehicleStatus`
與 `VehicleLandDetected` 判斷 `px4_state_valid`。

**背景：** PX4 以約 2 Hz 發布 `vehicle_status`。實測 Docker Desktop SITL 經 uXRCE-DDS
到 Agent 只收到約 1.5 Hz，最長間隔 3.08 秒；原本 2 秒門檻會在狀態只是短暫遺失時就
停止 Offboard 輸出，PX4 因 Offboard 訊號中斷觸發 failsafe（RTL）。實機飛控以 Wi-Fi UDP
連線，同樣可能遺失封包。

**判定規則（`controllers/Px4LinkFreshness.hpp`，純邏輯）：**

| 情況 | 行為 |
|---|---|
| 收到 `VehicleStatus` 且 `nav_state` 不是 Offboard | 立即停止 Offboard 輸出並進入 IDLE（PX4 已離開 Offboard，例如遙控接管或 failsafe） |
| `VehicleStatus` 超過 `PX4_STATE_TIMEOUT_MS`（5000 ms）未到達 | 停止 Offboard 輸出並進入 IDLE，log 標明為狀態過期 |
| 間隔在門檻內 | 繼續輸出；間隔超過 1.5 秒時輸出節流 WARN，供外場判斷連線品質 |

- 收到時間改用單調時鐘（steady clock），系統時間調整（對時、手動改時間）不會造成誤判。
- `px4_state_valid` 使用同一門檻；`VehicleLandDetected` 約 1 Hz，5 秒可容忍連續遺失 4 筆。
- 連線真的中斷時仍由 PX4 自身的 Offboard 遺失保護（`COM_OF_LOSS_T`）接手；Agent 端放寬門檻
  只影響「Agent 主動停止輸出」的時機，不影響 PX4 的保護。

**失效情境：**

| 情境 | 結果 |
|---|---|
| 狀態短暫遺失（< 5 秒） | 持續 Offboard，不再誤觸發 failsafe |
| PX4→Agent 單向中斷超過 5 秒 | Agent 停止輸出，PX4 依 Offboard 遺失保護處置 |
| Agent→PX4 中斷 | PX4 在 `COM_OF_LOSS_T` 後依保護處置，與本修正無關 |
| 遙控接管或 PX4 切換模式 | 收到新狀態即停止，不等待逾時 |

**驗證：** `test/px4_link_freshness_test.cpp`（門檻邊界、3.1 秒間隔不中止、離開 Offboard 立即
中止、未收到與時間倒退）；`test/basic_controller_setpoint_test.cpp` 以真實等待確認 3 秒無狀態
仍維持 Offboard、超過 5 秒停止；SITL 以 `basic_controller_offboard_validation.sh` 與
`dual_agent_regression_sitl_validation.sh --count 3`（三機）驗證。

## 事件與健康狀態

`basic_controller_node` 發布 `/{agent_namespace}/events`（`AgentEvent`）與
`/{agent_namespace}/health`（`AgentHealth`，預設 1 Hz）。代碼與欄位見
[介面規格書 3.12、3.13](taiphoon-agent-interface.md#312-agentevent-taiphoon_agentmsgagentevent)。

- 事件回呼由 `BasicController` 注入（`set_event_callback()`），只在控制迴圈或命令處理中呼叫；
  PX4 ack callback 只把結果排入佇列，由下一個控制週期比對並發布。
- 追蹤的 PX4 命令（ARM、DISARM、MODE_OFFBOARD、HOLD、MODE_RTL、MODE_LAND、REBOOT_FC）在
  `px4_command_ack_timeout_s`（預設 3 秒）內沒有 ack 即發布 `PX4_COMMAND_TIMEOUT`；
  拒絕發布 `PX4_COMMAND_REJECTED`，ARM／DISARM／REBOOT_FC 被接受另發布 `PX4_COMMAND_ACCEPTED`。
- 同一代碼與命令 5 秒內只發布一次；`PX4_STATUS_STALE`／`RECOVERED`、`PX4_PREFLIGHT_FAILED`、
  `PX4_FAILSAFE` 只在狀態轉換時發布。
- 位置資料（VehicleOdometry、VehicleGlobalPosition）超過 1 秒未更新發布 `PX4_POSITION_STALE`；
  啟動 5 秒內從未收到才視為「從未收到」。

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `health_publish_rate_hz` | 1.0 | `health` 發布頻率；不合法或大於 10 時回到 1.0 |
| `px4_command_ack_timeout_s` | 3.0 | PX4 命令 ack 等待時限 |
| `reboot_cooldown_s` | 15.0 | `REBOOT_FC` 送出後的冷卻時間 |

## 重新啟動飛控（REBOOT_FC）

安全條件（`controllers/FlightControllerReboot.hpp`，純邏輯）：

| PX4 狀態 | `REBOOT_FC` / `REBOOT_FC 0` | `REBOOT_FC 1` |
|----------|-----------------------------|---------------|
| VehicleStatus 新鮮、已上鎖、land detected 新鮮且已落地 | 送出 | 送出 |
| VehicleStatus 新鮮、已解鎖 | 拒絕 | 拒絕 |
| VehicleStatus 新鮮、已上鎖但未確認落地 | 拒絕 | 拒絕 |
| VehicleStatus 從未收到或逾時 | 拒絕，提示可用 force | 送出 |

- 冷卻（`reboot_cooldown_s`）優先於上表；任何拒絕都發布 `COMMAND_REJECTED` 並在 `detail` 寫明原因。
- 送出前停止 Offboard session、清除排隊中的 ARM／TAKEOFF 與 timed trajectory，flight state 回到 IDLE。
- `REBOOT_FC 1` 的前提是 PX4 Commander 在解鎖時以 DENIED 拒絕重新啟動，必須在實機驗證；
  若實機韌體不拒絕，`force` 改為不提供。PX4 SITL（POSIX 平台）不支援重新啟動，`REBOOT_FC` 一律回 DENIED。

**重新啟動偵測（`Px4RestartDetector`）**：上鎖狀態下 VehicleStatus 逾時後恢復，或 `REBOOT_FC`
被接受後出現超過 1.5 秒的中斷再恢復（60 秒內），即視為 PX4 重新啟動。BasicController 先清除
Offboard session、排隊命令、目標航向與暫存狀態，再發布 `PX4_RESTART_DETECTED`；
`basic_controller_node` 同時重建 WGS84→PX4 local 的軌跡平移，`ego_swarm_planner` 收到事件後清除
任務與編隊並重建 planner-local→PX4 local 平移。PX4 晚於 Agent 啟動時也會出現一次此事件，屬正常現象。

## 偏航限速與起飛航向

**背景：** 實機試飛發現兩個偏航問題。(1) 起飛後機頭自動轉向正北：目標航向尚未設定時，
TrajectorySetpoint 的 yaw 以 0 送出。(2) YAW 命令轉向過急造成瞬間拉高：yaw setpoint 一次
跳到最終角度，PX4 Offboard 位置模式不套用 `MPC_YAWRAUTO_MAX`，以 `MC_YAWRATE_MAX` 全速
追趕；偏航飽和時 Airmode 會提高總推力換取偏航權限。

**行為（`controllers/YawSlewLimiter.hpp`，純邏輯）：**

| 情況 | 行為 |
|---|---|
| 地面起飛（PX4 回報落地或尚未設定目標航向） | 目標與送出航向都鎖定為當下航向，機頭不轉向 |
| 空中重送 `TAKEOFF` | 保留既有目標航向，與 planner 的 target yaw 一致 |
| `YAW`／`MOVE_POINT` yaw | 累加或設定最終目標航向；每個 10 Hz 控制週期沿最短角度推進 `yaw_rate_limit_deg_s × 0.1 s`，`yawspeed` 填入前饋，最後一步送出目標並把前饋歸零 |
| 偏航未到達且沒有軌跡 | 逐週期重送 setpoint，位置沿用上一筆已送出的 setpoint（不使用可能過期的移動目標） |
| 目標航向未設定 | 防呆值為目前航向，不再送 0（正北） |
| Offboard 中斷後重新進入 | 送出航向從實際航向重新開始，限速轉回目標航向，不會瞬間跳轉 |
| 正好相差 180 度 | 取順時針，與 YAW 正方向一致 |

預設 30 deg/s 時，`YAW 90` 約需 3 秒。

**實機檢查（PX4 參數）：**

- `MC_AIRMODE`：2（Roll/Pitch/Yaw）在偏航飽和時會提高總推力，造成瞬間拉高；建議 0 或 1。
- `MC_YAWRATE_MAX`：Offboard 位置模式的偏航角速率上限；`yaw_rate_limit_deg_s` 應小於等於此值。

**驗證：** `test/yaw_slew_limiter_test.cpp`（正規化、最短路徑、跨 ±180 度、收斂與參數防呆）；
`test/basic_controller_setpoint_test.cpp`（起飛鎖航向、限速逼近、連續 YAW 累加、重入 Offboard、
二次起飛重鎖）；`test/basic_controller_trajectory_test.cpp`（軌跡完成後 YAW 維持終點）；SITL 以
`dual_agent_regression_sitl_validation.sh --count 3` 的 yaw 步驟量測轉向時間、峰值角速率與高度偏差。

## 預期行為（編隊）

1. 啟動 Launch 後，每台無人機各自執行 BasicController 自動 Offboard → Arm → Takeoff
2. 所有無人機起飛至目標高度後懸停
3. 透過 `/uav_all_agent/command` 發送 `FORMATION_CIRCLE R H`，
   所有 FormationNode 同時收到命令，各自計算目標位置
4. 若某無人機離線（`/uav_<N>_agent` 節點消失，或超過 2 秒未收到其
   `/uav_<N>_agent/state`），MissionPlanner 自動從編隊中排除，重新計算質心快照與角度分配
5. 若線上機體少於 2 台，編隊自動取消並原地懸停（不自動恢復）
6. 透過 `FORMATION_CANCEL` 取消編隊，各機恢復懸停

## 啟動驗證流程（快速測試）

```bash
# 終端 1：啟動 4 台模擬器
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --net Taiphoon_Net

# 終端 2：啟動 Taiphoon_Agent
docker compose exec uav_1_agent bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4

# 終端 3：編隊指令
docker compose exec uav_1_agent bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash

ros2 topic pub --times 5 /uav_all_agent/command std_msgs/msg/String 'data: ARM'
ros2 topic pub --times 5 /uav_all_agent/command std_msgs/msg/String 'data: TAKEOFF 15'
ros2 topic pub --times 5 /uav_all_agent/command std_msgs/msg/String 'data: FORMATION_CIRCLE 10 10'
```

## 日誌檢視

```bash
# 檢視 NED 目標（formation_node 輸出）
ros2 topic echo /uav_1_agent/ned_target

# 檢視所有 topic
ros2 topic list
```

## 故障排除

### 問題：PX4 未切換至 Offboard 模式
- 確認 `offboard_control_mode` topic 有持續發布（心跳）
- 確認 `auto_start_sequence` 參數為 true
- 多機模式下注意 `startup_delay_s` 是否造成延遲

### 問題：無人機不起飛
- 確認 `/uav_{N}/fmu/in/trajectory_setpoint` 有正確的 Z 軸設定
- 確認 PX4 參數 `COM_RCL_EXCEPT` 設為 4（允許 Offboard）
- 使用 `ros2 topic echo /uav_{N}/fmu/out/vehicle_status` 確認狀態

### 問題：Launch 後找不到節點
- 確認已 source 所有必要 workspace：
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
  source /home/taiphoon/Taiphoon_Agent/install/setup.bash
  ```

### 問題：編隊未生效
- 確認 `drone_count > 1`（FormationNode 只在多機模式下啟動）
- 確認命令透過 `/uav_all_agent/command` 而非單機 topic 發送
- 確認所有無人機的里程計 topic `/uav_{N}/fmu/out/vehicle_odometry` 有資料

### 問題：無人機離線後編隊未更新
- MissionPlanner 以 `/uav_<N>_agent` 節點存在性判定離線，並以 `peer_timeout_s`
  （預設 2 秒）state 靜默作為加速器；確認節點消失或逾時後應出現
  "Agent N marked offline" 日誌
- 若線上機體少於 2 台，編隊會自動取消，日誌出現 "Formation cancelled (online < 2)"

### 問題：建置失敗
- 確認 `px4_msgs` 已安裝：`sudo apt install ros-humble-px4-msgs`
- 確認 `colcon` 已安裝：`sudo apt install python3-colcon-common-extensions`
- 確認自訂訊息目錄 `msg/` 存在正確的 `.msg` 檔案

### 自訂訊息（NedTarget）

```msg
float32 x     # NED X（前向為正，公尺）
float32 y     # NED Y（右向為正，公尺）
float32 z     # NED Z（向下為正，高度取負值，公尺）
float32 yaw   # 目標航向角（度，順時針為正；NaN 表示不控制偏航）
```

> `yaw` 設為 NaN 時，飛控會維持目前朝向，適用於編隊飛行中不強制轉向的情境。
> 自訂訊息定義位於 `/msg/NedTarget.msg`，建置時由 `rosidl_generate_interfaces` 自動產生對應的 C++ 型別。
