# Taiphoon Agent ROS2 地面控制站 (GCS) 介面規格書

> **版本**：1.5
> **更新日期**：2026-08-10
> **適用對象**：ROS2 地面控制站 (GCS) 開發者
>
> 本文件完整列出 Taiphoon_Agent 專案中所有 ROS2 節點、Topic、Service、自訂訊息格式與命令協定，
> 供 GCS 開發團隊進行訂閱、發布與監控整合。

---

## 目錄

1. [系統架構總覽](#1-系統架構總覽)
2. [節點 (Node) 清單](#2-節點-node-清單)
3. [自訂訊息 (Custom Message) 定義](#3-自訂訊息-custom-message-定義)
4. [Topic 完整清單](#4-topic-完整清單)
5. [Service 完整清單](#5-service-完整清單)
6. [命令協定 (Command Protocol)](#6-命令協定-command-protocol)
7. [命名空間規則](#7-命名空間規則)
8. [QoS 設定摘要](#8-qos-設定摘要)
9. [多機部署注意事項](#9-多機部署注意事項)
10. [Launch 參數](#10-launch-參數)

---

## 1. 系統架構總覽

```
GCS 命令 ──▶ EgoSwarmPlanner ──▶ AgentTrajectory ──▶ BasicController ──▶ PX4 TrajectorySetpoint
                  ▲       ▲              │
                  │       │              └──▶ /fmu_all_agent/swarm/agent_trajectories
                  │       └── PointCloud2（局部障礙物）
                  └── GPS common frame／編隊槽位／同儕未來軌跡

ObstacleCloudSimulator ──▶ /{agent_namespace}/perception/obstacles
```

本文件依目前工作區的 Node、`.msg` 與 Launch 實作定義 ROS2 契約。EGO-Swarm 核心
目前是 deterministic candidate baseline，透過固定候選 B-spline 進行成本與安全評估；
不宣稱已完整重現原始 EGO-Swarm 的 nonlinear optimizer。

每一台無人機對應一個 Taiphoon_Agent 實例，內含四個節點：

| 節點 | 功能 | 必有 |
|------|------|------|
| `basic_controller_node` | 飛控橋接、狀態發布、命令轉譯，以及 EGO-Swarm 軌跡取樣後發布 PX4 setpoint | ✅ |
| `ego_swarm_planner` | GPS common frame、編隊槽位、PointCloud2 障礙物與同儕未來軌跡的 deterministic candidate baseline 局部規劃 | ✅ |
| `obstacle_cloud_simulator` | 依 YAML 障礙物描述發布 PointCloud2 模擬資料 | 選配 |
| `agent_supervisor` | Agent 容器生命週期管理，提供失效模擬服務 | ✅ |

`mission_planner` 與 `trajectory_controller` 已由 EGO-Swarm 執行鏈取代；若舊程式仍
存在，僅可作為純邏輯測試或過渡期間的非控制實作，不能再視為新的 PX4 控制資料流。

---

## 2. 節點 (Node) 清單

### 2.1 basic_controller_node

| 屬性 | 值 |
|------|-----|
| **執行檔** | `basic_controller` |
| **節點名稱** | `basic_controller_node` |
| **命名空間** | `fmu{N}`（例如 `fmu1`） |
| **描述** | 核心飛控橋接節點。連接 PX4 並提供 Agent 對外狀態。 |

**內部組件**：
- `BasicController`：PX4 Offboard 控制（arm/disarm/takeoff/land/move）
- `AgentCommandHandler`：接收並解析高階命令字串
- EGO-Swarm 軌跡執行器：取樣規劃器提供的軌跡，產生 PX4 `TrajectorySetpoint`
- `PositionSafetyMonitor`：位置偏差安全監控
- `StatePublisher`：填充 AgentState / AgentAttitude 訊息

`basic_controller_node` 是唯一允許發布 PX4 `TrajectorySetpoint` 的 Agent 節點。

### 2.2 ego_swarm_planner

| 屬性 | 值 |
|------|-----|
| **執行檔** | `ego_swarm_planner` |
| **節點名稱** | `ego_swarm_planner` |
| **命名空間** | `fmu{N}` |
| **描述** | EGO-Swarm 任務與局部軌跡規劃。接收 GCS 任務意圖、GPS common frame、PointCloud2 與同儕未來軌跡，輸出本機軌跡。 |
| **參數** | `agent_namespace`、`agent_index`、`planning_rate_hz`、`formation_min_distance_m`、`peer_collision_min_distance_m`、`obstacle_clearance_m`、`knot_interval_s`、`max_velocity_m_s`、`max_acceleration_m_s2`、`candidate_offset_m`、`local_arrival_tolerance_m`、`global_arrival_tolerance_m` |

**內部組件**：
- `SwarmFrame`：共同 GPS／NED 座標框架
- `FormationSlotPlanner`：虛擬編隊槽位計算
- `LocalObstacleMap`：PointCloud2 局部障礙物資料處理
- `PeerTrajectoryTable`：同儕未來軌跡保存
- `EgoSwarmPlanner`：deterministic candidate baseline 局部軌跡成本與安全評估；不宣稱完整原始 EGO-Swarm nonlinear optimizer

**實際參數**：

| 參數 | 型別 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | `string` | `fmu1_agent` | Agent 對外命名空間 |
| `agent_index` | `int` | `1` | Agent 索引；小於等於 0 時由 Node 推算 |
| `swarm_origin_agent_index` | `int` | `1` | 固定 `swarm_ned` 原點的 Agent 索引；等待其有效 GPS，建立後 freeze |
| `planning_rate_hz` | `double` | `10.0` | 規劃迴圈頻率 |
| `formation_min_distance_m` | `double` | `3.0` | 編隊槽位的機體中心到中心最小距離 |
| `peer_collision_min_distance_m` | `double` | `3.0` | 同儕軌跡的機體中心到中心最小距離 |
| `obstacle_clearance_m` | `double` | `1.0` | 障礙物安全間距 |
| `knot_interval_s` | `double` | `1.0` | B-spline 最小基準 knot 間隔；實際 duration 由候選路徑長度與動力學限制計算 |
| `max_velocity_m_s` | `double` | `2.0` | duration 計算使用的最大速度 |
| `max_acceleration_m_s2` | `double` | `1.0` | duration 計算使用的最大加速度 |
| `candidate_offset_m` | `double` | `3.0` | 局部候選軌跡偏移量 |
| `local_arrival_tolerance_m` | `double` | `0.5` | PX4 local NED 到達容許誤差 |
| `global_arrival_tolerance_m` | `double` | `1.0` | GPS common NED 到達容許誤差 |

### 2.3 obstacle_cloud_simulator

| 屬性 | 值 |
|------|-----|
| **執行檔** | `obstacle_cloud_simulator` |
| **節點名稱** | `obstacle_cloud_simulator` |
| **命名空間** | 依啟動設定決定 |
| **描述** | 讀取 YAML 幾何障礙物，產生 `sensor_msgs/msg/PointCloud2`，供 EGO-Swarm 局部障礙物地圖使用。 |
| **參數** | `agent_namespace`、`config_file`、`publish_rate_hz`、`mode`、`range_m`、`horizontal_fov_deg`、`vertical_fov_deg`、`noise_std_m`、`dropout_ratio`、`sample_spacing_m` |

支援的目標場景類型為 `box`、`cylinder` 與 `wall`，並規劃支援 `ground_truth_map`
與 `sensor_emulation` 兩種資料模式。輸出 Topic 固定為
`/{agent_namespace}/perception/obstacles`。

**實際參數**：

| 參數 | 型別 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | `string` | `fmu1_agent` | Agent 對外命名空間 |
| `config_file` | `string` | 空字串 | YAML 場景檔；Launch 預設使用 `config/obstacles/default.yaml` |
| `publish_rate_hz` | `double` | `15.0` | PointCloud2 發布頻率 |
| `mode` | `string` | `sensor_emulation` | `ground_truth_map` 或 `sensor_emulation` |
| `range_m` | `double` | `20.0` | 感測範圍 |
| `horizontal_fov_deg` | `double` | `270.0` | 水平視野 |
| `vertical_fov_deg` | `double` | `90.0` | 垂直視野 |
| `noise_std_m` | `double` | `0.02` | 感測位置雜訊標準差 |
| `dropout_ratio` | `double` | `0.05` | 點雲 dropout 比例 |
| `sample_spacing_m` | `double` | `0.5` | 障礙物表面採樣間距 |

### 2.4 agent_supervisor

| 屬性 | 值 |
|------|-----|
| **執行檔** | `agent_supervisor` |
| **節點名稱** | `agent_supervisor` |
| **命名空間** | `fmu{N}_agent`（例如 `fmu1_agent`） |
| **描述** | Agent 容器生命週期管理節點，提供刻意模擬飛行電腦失效的 shutdown service。 |

**對外介面**：
- `/{agent_namespace}/shutdown`：詳見 [第 5 節](#5-service-完整清單)。

---

## 3. 自訂訊息 (Custom Message) 定義

所有自訂訊息位於套件 `taiphoon_agent` 的 `msg/` 目錄。

### 3.1 AgentState (`taiphoon_agent/msg/AgentState`)

Agent 對外狀態，由 `basic_controller_node` 發布。`flight_mode` 是經 PX4 實際觀測結果校正後的
任務階段，不代表最近收到的命令意圖。位置資訊由既有的 `position_ned` 與
`position_global` 提供，不另外複製 PX4 Position 欄位。`position_global` 唯一取自
`/fmu{N}/fmu/out/vehicle_global_position`；AgentState 名目發布頻率至少為 10 Hz，若 PX4
原始資料更新較慢，期間會重複最近一筆有效值，不進行插值。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `position_ned` | `float32[3]` | NED 座標位置（前向/右向/向下，公尺） |
| `position_global` | `float32[3]` | 全域 GPS 座標 [lat(deg), lon(deg), alt(m AMSL)]，來自 PX4 VehicleGlobalPosition |
| `flight_mode` | `uint8` | 經 PX4 觀測校正後的任務階段（對應 FlightState 枚舉，見下表），不是命令意圖 |
| `battery_remaining` | `float32` | 剩餘電量比例（0.0 ~ 1.0），來自 PX4 BatteryStatus |
| `battery_voltage_v` | `float32` | 電池電壓（V），來自 PX4 BatteryStatus |
| `battery_current_a` | `float32` | 電池電流（A），來自 PX4 BatteryStatus |
| `online` | `bool` | Agent 是否在線 |
| `armed` | `bool` | PX4 實際解鎖狀態 |
| `landed` | `bool` | PX4 實際落地狀態 |
| `px4_nav_state` | `uint8` | PX4 `VehicleStatus.nav_state` 原始值；Manual、Position、Offboard 等 PX4 原生模式由此欄位表示 |
| `px4_state_valid` | `bool` | PX4 狀態欄位是否有效；只有此值為 `true` 時，`armed`、`landed` 與 `px4_nav_state` 才可信 |

> **PX4 模式判讀**：`px4_nav_state` 不經 Agent 重新編碼，原樣對應 PX4
> `VehicleStatus.nav_state`。GCS 應先確認 `px4_state_valid=true`，再依相同版本
> `px4_msgs` 的 `VehicleStatus` 常數判讀 Manual、Position、Offboard 等原生導航模式。

**FlightState 枚舉對照表**：

| 值 | 名稱 | 說明 |
|----|------|------|
| 0 | `IDLE` | 閒置 |
| 1 | `OFFBOARD_INIT` | 等待 Offboard 模式啟用 |
| 2 | `OFFBOARD_ACTIVE` | Offboard 已啟用，等待解鎖 |
| 3 | `ARMED` | 已解鎖 |
| 4 | `TAKEOFF` | 起飛中 |
| 5 | `HOVER` | 懸停中 |
| 6 | `RTL` | 返航中 |
| 7 | `LAND` | 降落中 |
| 8 | `MOVING` | 水平移動中 |
| 9 | `POST_RTL` | RTL 降落後 |
| 10 | `ERROR` | 錯誤狀態 |

### 3.2 AgentAttitude (`taiphoon_agent/msg/AgentAttitude`)

Agent 姿態訊息，由 `basic_controller_node` 發布。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `q` | `float32[4]` | 四元數姿態（w, x, y, z） |

### 3.3 NedTarget (`taiphoon_agent/msg/NedTarget`)

既有 NED 絕對目標位置訊息。EGO-Swarm 取代舊任務／直線軌跡執行鏈後，僅作相容或
純邏輯測試用途，不是新的 EGO-Swarm 未來軌跡契約。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `x` | `float32` | NED X 軸位置（前向為正，公尺） |
| `y` | `float32` | NED Y 軸位置（右向為正，公尺） |
| `z` | `float32` | NED Z 軸位置（向下為正，公尺） |
| `yaw` | `float32` | 目標航向角（度，順時針為正），`NaN` 表示不控制偏航 |

### 3.4 Waypoint (`taiphoon_agent/msg/Waypoint`)

單一軌跡 waypoint。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `position_ned` | `float32[3]` | NED 位置（前向/右向/向下，公尺） |
| `arrival_time` | `builtin_interfaces/Time` | 預定到達時間 |

### 3.5 WaypointSequence (`taiphoon_agent/msg/WaypointSequence`)

既有 waypoint 序列訊息。EGO-Swarm 取代 `trajectory_controller` 後，僅作純邏輯測試
用途，不是 PX4 控制來源。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `waypoints` | `Waypoint[]` | waypoint 序列 |

### 3.6 AgentTrajectory (`taiphoon_agent/msg/AgentTrajectory`)

EGO-Swarm Agent 未來軌跡訊息，供 BasicController 執行與同儕避碰。
ROSIDL 對 `control_points` 產生 `std::vector<float>`；欄位採扁平化格式，
每三個 float 依序表示一個 `swarm_ned` 控制點 `[x, y, z]`。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 發布時間 |
| `start_time` | `builtin_interfaces/Time` | 軌跡起始時間；active trajectory 重發時保持不變 |
| `trajectory_id` | `uint32` | 單調遞增的軌跡識別碼；同一條 active trajectory 重發時保持不變 |
| `agent_index` | `uint32` | 發布者 Agent 索引 |
| `frame_revision` | `uint32` | `swarm_ned` 座標版本 |
| `order` | `uint8` | B-spline 階數 |
| `knot_interval_s` | `float32` | knot 間隔（秒） |
| `control_points` | `float32[]` | 扁平化 `swarm_ned` 控制點；每 3 個連續元素為一個 `[x, y, z]`，排列為 `[x0,y0,z0,x1,y1,z1,...]`，長度必須為 3 的倍數 |
| `duration_s` | `float32` | 由 EGO-Swarm 依候選路徑長度與最大速度／加速度計算的軌跡有效時間（秒） |
| `frame_origin_global` | `float64[3]` | 共同座標原點 `[lat, lon, alt]` |
| `valid` | `bool` | 軌跡是否可供同儕使用 |

### 3.7 PlannerStatus (`taiphoon_agent/msg/PlannerStatus`)

EGO-Swarm Planner 狀態與安全指標。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 狀態時間戳 |
| `state` | `uint8` | 規劃器狀態 |
| `trajectory_id` | `uint32` | 目前軌跡識別碼 |
| `map_valid` | `bool` | 障礙物地圖是否有效 |
| `localization_valid` | `bool` | 共同座標是否有效 |
| `peer_data_valid` | `bool` | 同儕資料是否可用 |
| `planning_time_ms` | `float32` | 最近一次規劃耗時 |
| `min_obstacle_distance_m` | `float32` | 最近障礙物距離 |
| `min_peer_distance_m` | `float32` | 最近同儕中心距離 |
| `reason` | `string` | 降級或失敗原因 |

### 3.8 RelativeAgentObservation (`taiphoon_agent/msg/RelativeAgentObservation`)

未來相對定位 Adapter 使用的單一同儕觀測。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 觀測時間戳 |
| `agent_index` | `uint32` | 同儕 Agent 索引 |
| `relative_position_ned` | `float32[3]` | 相對 NED 位置（公尺） |
| `relative_velocity_ned` | `float32[3]` | 相對 NED 速度（公尺／秒） |
| `confidence` | `float32` | 觀測信心度（0.0 ~ 1.0） |
| `valid` | `bool` | 觀測是否有效 |

---

## 4. Topic 完整清單

### 4.1 PX4 飛控橋接 Topic（命名空間：`fmu{N}`）

這些 Topic 是 Taiphoon_Agent 與 PX4 SITL 之間的內部通訊。**GCS 通常不需直接操作**，但理解這些 Topic 有助於除錯。

| Topic | 方向 | 型別 | QoS |
|-------|------|------|-----|
| `fmu/in/offboard_control_mode` | 發布 → PX4 | `px4_msgs/OffboardControlMode` | best_effort (10) |
| `fmu/in/trajectory_setpoint` | 發布 → PX4 | `px4_msgs/TrajectorySetpoint` | best_effort (10) |
| `fmu/in/vehicle_command` | 發布 → PX4 | `px4_msgs/VehicleCommand` | best_effort (10) |
| `fmu/out/vehicle_odometry` | 訂閱 ← PX4 | `px4_msgs/VehicleOdometry` | best_effort (10) |
| `fmu/out/vehicle_global_position` | 訂閱 ← PX4 | `px4_msgs/VehicleGlobalPosition` | best_effort (10) |
| `fmu/out/vehicle_command_ack_v1` | 訂閱 ← PX4 | `px4_msgs/VehicleCommandAck` | best_effort (10) |
| `fmu/out/vehicle_status_v4` | 訂閱 ← PX4 | `px4_msgs/VehicleStatus` | best_effort (10) |

> **注意**：Topic 名稱為相對路徑，實際解析後會加上節點命名空間，例如 `/fmu1/fmu/in/offboard_control_mode`。

### 4.2 Agent 對外 Topic（GCS 關注重點）

這些是 **GCS 應直接訂閱與發布** 的 Topic。

#### 4.2.1 狀態發布（Agent → GCS）

| Topic 模板 | 說明 | 型別 | 頻率 | QoS |
|------------|------|------|------|-----|
| `/{agent_namespace}/state` | Agent 狀態（位置、飛行模式、電量） | `taiphoon_agent/AgentState` | ≥ 10 Hz（名目） | reliable (10) |
| `/{agent_namespace}/attitude` | Agent 姿態（四元數） | `taiphoon_agent/AgentAttitude` | ≥ 10 Hz（名目） | reliable (10) |

**實際範例**（4 機部署）：

| 無人機 | state Topic | attitude Topic |
|--------|------------|----------------|
| fmu1 | `/fmu1_agent/state` | `/fmu1_agent/attitude` |
| fmu2 | `/fmu2_agent/state` | `/fmu2_agent/attitude` |
| fmu3 | `/fmu3_agent/state` | `/fmu3_agent/attitude` |
| fmu4 | `/fmu4_agent/state` | `/fmu4_agent/attitude` |

#### 4.2.2 命令下達（GCS → Agent）

| Topic | 說明 | 型別 | QoS |
|-------|------|------|-----|
| `/{agent_namespace}/command` | 發送命令給**指定** Agent | `std_msgs/String` | reliable (10) |
| `/fmu_all_agent/command` | 廣播命令給**所有** Agent | `std_msgs/String` | reliable (10) |

> **GCS 設計建議**：
> - 使用 `/{agent_namespace}/command` 對單一無人機下達移動、模式切換命令。
> - 使用 `/fmu_all_agent/command` 發送編隊命令（FORMATION_*）或全體同步命令（如全體 RTL）。
> - 命令格式詳見 [第 6 節](#6-命令協定-command-protocol)。

#### 4.2.3 內部控制流（Agent 內部通訊，GCS 可監控）

| Topic 模板 | 說明 | 型別 | QoS |
|------------|------|------|-----|
| `/fmu_all_agent/swarm/agent_trajectories` | 各 Agent 發布的 EGO-Swarm 未來軌跡，供同儕規劃與 BasicController 取樣使用 | `taiphoon_agent/AgentTrajectory` | reliable (pub) / best_effort (sub), depth 10 |
| `/{agent_namespace}/perception/obstacles` | 本機局部障礙物 PointCloud2 | `sensor_msgs/msg/PointCloud2` | best_effort (pub/sub), depth 10 |
| `/{agent_namespace}/planning/mission_intent` | 本機任務意圖文字命令，供 EGO-Swarm Planner 設定目標 | `std_msgs/msg/String` | reliable (10) |
| `/{agent_namespace}/planning/status` | 本機 EGO-Swarm 規劃狀態與診斷 | `taiphoon_agent/PlannerStatus` | reliable (10) |
| `/{agent_namespace}/planning/trajectory` | 本機 EGO-Swarm 規劃結果 | `taiphoon_agent/AgentTrajectory` | reliable (10) |
| `/{agent_namespace}/ned_target` | 舊 NED 目標介面；EGO-Swarm 取代後僅作相容或純邏輯測試用途 | `taiphoon_agent/NedTarget` | 既有介面 |
| `/{agent_namespace}/waypoints` | 舊 waypoint 介面；EGO-Swarm 取代後僅作純邏輯測試用途，不能視為 PX4 控制來源 | `taiphoon_agent/WaypointSequence` | 既有介面 |

#### 4.2.4 跨 Agent 通訊（同儕發現）

EGO-Swarm 以 AgentState 取得同儕目前狀態與 GPS common frame 所需位置，並以同儕
未來軌跡進行時間域避碰：

| 角色 | Topic | 用途 |
|--------|-----------|------|
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/state`、`/{peer_namespace}/state` | 取得自身與同儕位置、GPS common frame 資料 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/ned_target` | 接收既有 NED 目標相容輸入 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/planning/mission_intent` | 接收本機任務意圖 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/command` | 接收本機 `MOVE_*`、`MOVE_POINT` 與 `FORMATION_CANCEL`，轉換為 EGO-Swarm 目標；模式、起飛與 YAW 仍由 BasicController 處理 |
| `ego_swarm_planner`（訂閱） | `/fmu_all_agent/command` | 接收全體編隊與同步命令 |
| `ego_swarm_planner`（訂閱） | `/fmu_all_agent/swarm/agent_trajectories` | 取得同儕未來軌跡進行避碰 |
| `ego_swarm_planner`（發布） | `/fmu_all_agent/swarm/agent_trajectories` | 發布本機未來軌跡 |
| `ego_swarm_planner`（發布） | `/{agent_namespace}/planning/status`、`/{agent_namespace}/planning/trajectory` | 發布本機規劃狀態與軌跡 |
| `obstacle_cloud_simulator`（發布） | `/{agent_namespace}/perception/obstacles` | 發布 PointCloud2 障礙物資料 |
| `basic_controller_node`（訂閱） | `/fmu_all_agent/swarm/agent_trajectories` | 取樣目前 Agent 的軌跡並發布 PX4 setpoint |

> **同儕資料來源**：AgentState 維持 GCS 與 Agent 的目前狀態介面；EGO-Swarm 的
> 未來軌跡透過 `/fmu_all_agent/swarm/agent_trajectories` 交換。EGO planner 每 3 秒
> 掃描同儕 Agent，並以各 Agent 的 state Topic 建立同儕訂閱。
>
> 機群之間一律只訂閱 `/fmu<N>_agent/state` 等 Agent 對外 topic，
> 不直接訂閱其他無人機的 PX4 原生 topic（`/fmu<N>/fmu/out/*`）。

### 4.2.5 EGO-Swarm 編隊與避碰契約

- GCS 命令先由 `ego_swarm_planner` 轉換為本機任務意圖與虛擬編隊槽位；規劃器輸出
  未來軌跡，再由 `basic_controller_node` 取樣成 PX4 `TrajectorySetpoint`。
- 目前 EGO-Swarm 核心是 deterministic candidate baseline：建立固定候選 B-spline，
  依目標、平滑、障礙物與同儕距離成本選擇安全候選；不宣稱完整原始 EGO-Swarm
  nonlinear optimizer。
- GPS common frame 是多機編隊的共同座標基準。GPS-only 可以正常執行編隊；PX4
  odometry 與 attitude 仍由 BasicController 用於即時軌跡追蹤。
- 同儕避碰的距離定義為機體中心到機體中心的三維直線距離，不加計機體半徑。
- 最小距離不得在文件或實作中硬編碼為 3 公尺；Launch 與 entrypoint 以
  `formation_min_distance_m`、`peer_collision_min_distance_m` 提供設定，可使用
  1.5 m、2 m 或 3 m 等值。
- 短暫缺少同儕未來軌跡時，使用最後一份有效軌跡，且不依資料年齡擴張安全半徑。
- PointCloud2 應以共同 `swarm_ned` 座標供局部障礙物地圖使用；YAML 障礙物描述
  用於建立 `box`、`cylinder` 與 `wall` 等模擬場景。
- `mission_planner`、`trajectory_controller` 與第一版觀測式防撞已被 EGO-Swarm
  取代；舊 `/ned_target` 與 `/waypoints` 僅保留相容或純邏輯測試用途。

---

## 5. Service 完整清單

### 5.1 Agent shutdown service

| Service | 提供節點 | 型別 | 說明 |
|---------|----------|------|------|
| `/{agent_namespace}/shutdown` | `agent_supervisor` | `std_srvs/srv/SetBool` | 僅接受 `request.data = true`；回應送出後以 SIGTERM 觸發 Agent 容器 PID 1 立即退出 |

**契約與安全限制**：

- `request.data` 必須為 `true`；`false` 不會觸發終止。
- 接受請求後，服務回應先送出，接著以 SIGTERM 觸發 PID 1 的立即退出，刻意模擬飛行電腦失效。
- 此服務不檢查飛行狀態、不執行 RTL 或 LAND；後續行為完全由 PX4 failsafe 決定。
- 不提供也不允許 `/fmu_all_agent/shutdown` 廣播；shutdown 必須指定單一 Agent namespace。
- 若 Compose 服務使用 `restart: no`，PID 1 終止後容器會保持停止，不會自動重新啟動。若使用自動重啟策略，容器可能重新啟動，服務也會再次出現，將無法保持失效狀態。

**呼叫範例**：

```bash
ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

**驗證服務與容器狀態**：

```bash
# 在 Agent 容器內確認服務存在與型別
ros2 service list | grep -F /fmu1_agent/shutdown
ros2 service type /fmu1_agent/shutdown
```

```bash
# 在 Agent 容器內呼叫；回應送出後容器 PID 1 應立即終止
ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

```bash
# 回到宿主機確認容器狀態與退出碼
docker compose ps fmu1_agent
docker inspect --format '{{.State.Status}} exit={{.State.ExitCode}} restart={{.RestartCount}}' fmu1_agent
```

## 6. 命令協定 (Command Protocol)

### 6.1 傳輸格式

- **Topic**：`/{agent_namespace}/command` 或 `/fmu_all_agent/command`
- **型別**：`std_msgs/msg/String`
- **編碼**：純文字 ASCII，不區分大小寫
- **格式**：`<COMMAND> [param1] [param2] [param3] [param4]`

### 6.2 支援的命令

#### 模式切換命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MODE_RTL` | 無 | 返航 (Return to Launch) |
| `MODE_LAND` | 無 | 降落 |
| `MODE_OFFBOARD` | 無 | 切換至 Offboard 模式 |
| `HOLD` | 無 | 位置保持 (Position Hold) |

#### 移動命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MOVE_FORWARD <x>` | 距離 (公尺) | 向前移動 |
| `MOVE_BACKWARD <x>` | 距離 (公尺) | 向後移動 |
| `MOVE_LEFT <x>` | 距離 (公尺) | 向左移動 |
| `MOVE_RIGHT <x>` | 距離 (公尺) | 向右移動 |
| `MOVE_UP <x>` | 距離 (公尺) | 向上移動 |
| `MOVE_DOWN <x>` | 距離 (公尺) | 向下移動 |
| `YAW <angle>` | 相對角度 (度) | 以目前航向為基準順時針旋轉，由 BasicController 直接發布 |
| `MOVE_POINT <x> <y> <z> <yaw>` | NED 座標 + 絕對偏航 | 位置由 Planner 規劃，yaw 由 BasicController 直接設定 |

#### 基礎控制命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `ARM` | 無 | 解鎖 |
| `DISARM` | 無 | 上鎖 |
| `TAKEOFF [altitude]` | 高度 (公尺，預設 3.0) | 起飛 |

#### 編隊命令（僅透過 `/fmu_all_agent/command` 廣播）

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `FORMATION_CIRCLE <radius> <altitude>` | 半徑 (公尺), 高度 (公尺) | 環形編隊 |
| `FORMATION_HORI_LINEAR <spacing> <altitude>` | 間距 (公尺), 高度 (公尺) | 水平一字編隊 |
| `FORMATION_VERT_LINEAR <spacing>` | 間距 (公尺) | 垂直一字編隊 |
| `FORMATION_CANCEL` | 無 | 取消編隊 |

> **EGO-Swarm 編隊基準點**：收到 `FORMATION_*` 指令後，由 `ego_swarm_planner`
> 使用 GPS common frame 計算共同參考點與虛擬編隊槽位；每台 Agent 自行規劃前往
> 自己的槽位。既有命令字串與廣播 Topic 保持不變。成員掃描週期為 3 秒。

#### 相容性簡寫

| 命令 | 簡寫 |
|------|------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

### 6.3 命令使用範例

```bash
# 讓 fmu1 起飛至 5 公尺
ros2 topic pub /fmu1_agent/command std_msgs/msg/String "data: 'TAKEOFF 5'"

# 讓所有無人機返航
ros2 topic pub /fmu_all_agent/command std_msgs/msg/String "data: 'MODE_RTL'"

# 環形編隊（半徑 10 公尺，高度 20 公尺）
ros2 topic pub /fmu_all_agent/command std_msgs/msg/String \
  "data: 'FORMATION_CIRCLE 10 20'"

# fmu2 向前移動 3 公尺
ros2 topic pub /fmu2_agent/command std_msgs/msg/String "data: 'MOVE_FORWARD 3'"
```

---

## 7. 命名空間規則

| 命名空間 | 用途 | 範例 |
|----------|------|------|
| `fmu{N}` | 節點命名空間，對應 PX4 實例 | `fmu1`, `fmu2`, `fmu3` |
| `fmu{N}_agent` | Agent 對外 Topic 前綴 | `fmu1_agent`, `fmu2_agent` |
| `fmu_all_agent` | 全體 Agent 廣播 Topic | `/fmu_all_agent/command` |

**規則**：
- 一個 Taiphoon_Agent 只控制一台 PX4（`fmu{N}`）。
- 所有 Agent 對外 Topic 使用 `/{agent_namespace}/...` 前綴。
- 廣播命令固定使用 `/fmu_all_agent/command`。
- Shutdown service 僅使用 `/{agent_namespace}/shutdown`，不提供 `/fmu_all_agent/shutdown`。

---

## 8. QoS 設定摘要

| 介面類型 | Reliability | Durability | Depth |
|-----------|-------------|------------|-------|
| PX4 飛控 Topic | best_effort | volatile | 10 |
| Agent 狀態 Topic (`/state`, `/attitude`) | reliable | volatile | 10 |
| 命令 Topic (`/command`) | reliable | volatile | 10 |
| Shutdown Service (`/shutdown`) | reliable | volatile | 10 |
| 控制流 Topic (`/ned_target`, `/waypoints`) | reliable (pub) / best_effort (sub) | volatile | 10 |
| `/fmu_all_agent/swarm/agent_trajectories` | reliable (pub) / best_effort (sub) | volatile | 10 |
| `/{agent_namespace}/perception/obstacles` | best_effort | volatile | 10 |
| `/{agent_namespace}/planning/status`、`/planning/trajectory` | reliable | volatile | 10 |

> 上述 QoS 與目前 `ego_swarm_planner`、`obstacle_cloud_simulator` 實作一致。

---

> **文件維護**：此文件由 Taiphoon_Agent 團隊維護，每次新增或修改 Topic/Service/Node 時應同步更新。
