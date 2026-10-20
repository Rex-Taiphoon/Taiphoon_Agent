# Taiphoon Agent ROS2 地面控制站 (GCS) 介面規格書

> **版本**：1.9
> **更新日期**：2026-08-27
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
GCS 命令 ──▶ EgoSwarmPlanner（Local Goal／FIXED_Z 時空路徑引導／optimizer）
          ──▶ /{agent_namespace}/planning/trajectory ──▶ BasicController ──▶ PX4 TrajectorySetpoint
                  │       ▲
                  │       └── PointCloud2（局部障礙物）
                  └── /uav_all_agent/swarm/agent_trajectories（同儕 future occupancy）
                      WGS84 → receiver-local NED／編隊槽位／同儕未來軌跡

ObstacleCloudSimulator ──▶ /{agent_namespace}/perception/obstacles
```

本文件依目前工作區的 Node、`.msg` 與 Launch 實作定義 ROS2 契約。EGO-Swarm 核心
目前 runtime 是 optimized continuous B-spline planner，透過中間 control-point optimization
進行成本與安全評估；deterministic candidate 類別僅保留相容與回歸用途。

每一台無人機對應一個 Taiphoon_Agent 實例，內含四個節點：

| 節點 | 功能 | 必有 |
|------|------|------|
| `basic_controller_node` | 飛控橋接、狀態發布、命令轉譯，以及 EGO-Swarm 軌跡取樣後發布 PX4 setpoint | ✅ |
| `ego_swarm_planner` | WGS84 軌跡交換、本機 local NED、編隊槽位、PointCloud2 障礙物與同儕未來軌跡的 optimized continuous 局部規劃 | ✅ |
| `obstacle_cloud_simulator` | 依 YAML 障礙物描述發布 PointCloud2 模擬資料 | 選配 |
| `agent_supervisor` | Agent 容器生命週期管理，提供失效模擬服務 | ✅ |

Docker Desktop 模擬＋主機 QGCS 單播模式另會啟動 `dds_peer_relay`。它不是 ROS2 節點、不建立任何
Topic，只在本機容器 IP 綁定其他 Agent 的 DDS 單播埠段，把被 CycloneDDS 改寫成本機 IP 的同儕
封包轉給對應 Agent 容器；實機與 Linux Docker Engine 不啟動。

`mission_planner` 與 `trajectory_controller` 已由 EGO-Swarm 執行鏈取代；若舊程式仍
存在，僅可作為純邏輯測試或過渡期間的非控制實作，不能再視為新的 PX4 控制資料流。

---

## 2. 節點 (Node) 清單

### 2.1 basic_controller_node

| 屬性 | 值 |
|------|-----|
| **執行檔** | `basic_controller` |
| **節點名稱** | `basic_controller_node` |
| **命名空間** | `uav_{N}`（例如 `uav_1`） |
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
| **命名空間** | `uav_{N}` |
| **描述** | EGO-Swarm 任務與局部軌跡規劃。接收 GCS 任務意圖、WGS84 同儕軌跡、PointCloud2，並在本機 planner-local NED 輸出軌跡。 |
| **參數** | 上述基礎參數，加上 `move_attitude_timeout_s`、`formation_slot_switch_penalty_m`、`formation_member_stable_scan_count`、planner backend／optimizer／reservation 參數 |

**內部組件**：
- `LocalNedFrame`：將 WGS84 全球座標與單一指定 Home 的公尺制 local NED 互轉；planner 實例使用本 Agent freeze 的 Home
- `FormationSlotPlanner`：虛擬編隊槽位計算
- `LocalGoalSelector`／`PlanningTargetSelector`：Local Goal 開啟時依 Final Target、
  PathConstraint、horizon、fresh 同儕 future trajectory 與 fresh 局部障礙物地圖，
  先過濾終點佔位衝突並產生安全側向候選，再交給完整 trajectory planner；功能關閉時直接保留 Final Target
- `ReplanTrigger`：Local Goal 開啟時依 segment progress、Final Target 變更、
  segment 結束與 active trajectory safety 判斷是否允許重規劃
- `PlannerAcceptancePolicy`：只有 planner 成功且輸出 finite、reservation（若需要）
  成功、ContinuousSafetyValidator 通過後，才更新 active trajectory
- `Planner target semantics`：明確區分任務 `Final Target`、本輪 `Planning Target`、
  目前 active segment 的 `active_plan_target_` 與其所屬任務的
  `active_plan_final_target_`；到達判斷、terminal hold 與 recovery 只使用 `Final Target`
- `LocalObstacleMap`：PointCloud2 局部障礙物資料處理
- `PeerTrajectoryTable`：同儕未來軌跡保存、timestamp sanity、freshness 與 expiration；ACTIVE 與 TERMINAL_HOLD 軌跡參與 planner，完成任務的終點由 heartbeat 保持佔位
- `DeterministicPlanner`／`EgoSwarmPlanner`：保留既有純邏輯相容檔案；runtime 不組裝此 backend
- `OptimizedEgoPlanner`：runtime 唯一使用的 pure C++ continuous control-point optimizer；失敗時 fail-closed HOLD／EmergencyBrake
- `SpatiotemporalRouteInitializer`：只在 Local Goal + `FIXED_Z` 且直線 seed 受阻時執行 bounded `(North, East, time)` A*；結果只作 B-spline seed，不新增 ROS2 介面
- `FormationReservationPlanner`：以 future `AgentTrajectory.start_time` 表示 reservation，不新增 ROS2 message
- `FormationConvergenceEvaluator`：由 fresh state、槽位誤差、epoch 與 terminal occupancy 推導 convergence；槽位到達不使用速度門檻

**實際參數**：

| 參數 | 型別 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | `string` | `uav_1_agent` | Agent 對外命名空間 |
| `agent_index` | `int` | `1` | Agent 索引；小於等於 0 時由 Node 推算 |
| `swarm_origin_agent_index` | `int` | `1` | 保留的相容參數；WGS84 軌跡交換時各 Agent 以自己的固定 Home 建立 local NED |
| `peer_trajectory_timeout_s` | `double` | `2.0` | 同儕 `AgentTrajectory` 收到後可供 planner 使用的最長時間；逾時後標記為 stale |
| `max_peer_time_skew_s` | `double` | `1.0` | `AgentTrajectory` message／start time 與本機 receive time 允許的最大共同時間偏差 |
| `peer_clock_skew_warning_s` | `double` | `0.1` | 同儕主機時鐘差估計（`收到時間 − AgentTrajectory.stamp` 於 5 秒內最小值）超過此值時輸出 WARN；只影響診斷 |
| `obstacle_cloud_timeout_s` | `double` | `1.0` | 局部障礙物地圖 freshness timeout；逾時後不再規劃 |
| `require_obstacle_cloud` | `bool` | `true` | 要求 fresh obstacle map；missing/stale 不得當成空環境 |
| `planning_rate_hz` | `double` | `10.0` | 規劃迴圈頻率 |
| `enable_local_goal_planning` | `bool` | `true` | Local Goal segmented planning；水平 FIXED_Z 受阻時使用 bounded 時空 A*，FREE_3D／LINEAR_Z 沿用既有 planner |
| `enable_generic_trajectory_reservation` | `bool` | `false` | 將 reservation 擴展至 MOVE 等所有 moving trajectory；Formation 仍固定使用 reservation |
| `peer_planning_margin_m` | `m` | `1.0` | planner 產生路徑時額外保留的機間距離；最終 safety validator 仍以 peer collision distance 驗證 |
| `local_goal_horizon_m` | `double` | `10.0` | Local Goal 單一 planning segment 的距離 horizon（公尺） |
| `local_goal_detour_margin_m` | `double` | `3.0` | Local Goal 側向繞行相對於 peer collision distance 額外保留的追蹤裕度（公尺） |
| `local_goal_replan_ratio` | `double` | `0.70` | Local Goal segment progress 達此比例後，允許嘗試下一段規劃 |
| `minimum_replan_interval_s` | `double` | `0.5` | 正常 progress-based replan 的最短間隔（秒）；不阻擋 safety、Final Target 變更與 segment 結束 |
| `formation_min_distance_m` | `double` | `2.0` | 編隊槽位的機體中心到中心最小距離 |
| `peer_collision_min_distance_m` | `double` | `2.0` | 同儕軌跡的機體中心到中心最小距離 |
| `obstacle_clearance_m` | `double` | `1.0` | 障礙物安全間距 |
| `knot_interval_s` | `double` | `1.0` | B-spline 最小基準 knot 間隔；實際 duration 由候選路徑長度與動力學限制計算 |
| `max_velocity_m_s` | `double` | `2.5` | duration 計算使用的最大速度 |
| `max_acceleration_m_s2` | `double` | `1.5` | duration 計算使用的最大加速度 |
| `max_velocity_mps` | `double` | `2.5` | safety validator／emergency brake 的最大速度；canonical 名稱 |
| `max_acceleration_mps2` | `double` | `1.5` | safety validator／emergency brake 的最大加速度 |
| `max_jerk_mps3` | `double` | `3.0` | safety validator／emergency brake 的最大 jerk |
| `candidate_offset_m` | `double` | `3.0` | 局部候選軌跡偏移量 |
| `local_arrival_tolerance_m` | `double` | `0.5` | PX4 local NED 到達容許誤差 |
| `global_arrival_tolerance_m` | `double` | `1.0` | 相容參數名稱；WGS84 投影至本機 planner-local NED 的到達容許誤差 |
| `move_attitude_timeout_s` | `double` | `1.0` | 水平 MOVE 接受的 AgentAttitude freshness window |
| `formation_slot_switch_penalty_m` | `double` | `2.0` | minimum-cost assignment 換槽位成本；只在 parent 與目前槽位數相同時加計 |
| `formation_member_stable_scan_count` | `int` | `2` | 成員集合連續穩定 discovery 次數 |
| `planner_backend` | `string` | `optimized` | 只接受 `optimized`；其他值使 planner 初始化失敗 |
| `optimizer_max_iterations` | `int` | `40` | optimized projected-gradient iteration 上限 |
| `optimizer_time_budget_ms` | `double` | `50.0` | optimized backend 單次 wall-time budget |
| `formation_crossing_penalty_m` | `double` | `10.0` | assignment 水平 crossing 等價成本 |
| `reservation_lead_time_s` | `double` | `0.5` | future trajectory reservation 最短提前時間 |
| `reservation_step_s` | `double` | `0.5` | reservation start time 搜尋步進 |
| `reservation_horizon_s` | `double` | `5.0` | reservation start time 最遠搜尋範圍 |

### 2.3 obstacle_cloud_simulator

| 屬性 | 值 |
|------|-----|
| **執行檔** | `obstacle_cloud_simulator` |
| **節點名稱** | `obstacle_cloud_simulator` |
| **命名空間** | 依啟動設定決定 |
| **描述** | 讀取 YAML 幾何障礙物，產生 `sensor_msgs/msg/PointCloud2`，供 EGO-Swarm 局部障礙物地圖使用。 |
| **參數** | `agent_namespace`、`agent_index`、`swarm_origin_agent_index`、`config_file`、`publish_rate_hz`、`mode`、`range_m`、`horizontal_fov_deg`、`vertical_fov_deg`、`noise_std_m`、`dropout_ratio`、`sample_spacing_m` |

支援的目標場景類型為 `box`、`cylinder` 與 `wall`，資料模式為
`explicit_empty`、`ground_truth` 與 `synthetic_sensor`（相容
`ground_truth_map`／`sensor_emulation`）。輸出 Topic 固定為
`/{agent_namespace}/perception/obstacles`。

**實際參數**：

| 參數 | 型別 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | `string` | `uav_1_agent` | Agent 對外命名空間 |
| `agent_index` | `int` | `1` | Agent 索引；供 synthetic sensor 訂閱自身 pose |
| `swarm_origin_agent_index` | `int` | `1` | synthetic sensor 的固定 `swarm_ned` 原點 Agent |
| `config_file` | `string` | 空字串 | YAML 場景檔；Launch 預設使用 `config/obstacles/default.yaml` |
| `publish_rate_hz` | `double` | `15.0` | PointCloud2 發布頻率 |
| `mode` | `string` | `explicit_empty` | ROS2 parameter 優先於 YAML `mode`；模擬預設發布 fresh zero-point cloud |
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
| **命名空間** | `uav_{N}_agent`（例如 `uav_1_agent`） |
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
`/uav_{N}/fmu/out/vehicle_global_position`；AgentState 名目發布頻率至少為 10 Hz，若 PX4
原始資料更新較慢，期間會重複最近一筆有效值，不進行插值。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `position_ned` | `float32[3]` | NED 座標位置（前向/右向/向下，公尺） |
| `position_global` | `float64[3]` | 全域 GPS 座標 [lat(deg), lon(deg), alt(m AMSL)]，來自 PX4 VehicleGlobalPosition；保留 GPS double 精度 |
| `flight_mode` | `uint8` | 經 PX4 觀測校正後的任務階段（對應 FlightState 枚舉，見下表），不是命令意圖 |
| `battery_remaining` | `float32` | 剩餘電量比例（0.0 ~ 1.0），來自 PX4 BatteryStatus |
| `battery_voltage_v` | `float32` | 電池電壓（V），來自 PX4 BatteryStatus |
| `battery_current_a` | `float32` | 電池電流（A），來自 PX4 BatteryStatus |
| `online` | `bool` | Agent 是否在線 |
| `armed` | `bool` | PX4 實際解鎖狀態 |
| `landed` | `bool` | PX4 實際落地狀態 |
| `px4_nav_state` | `uint8` | PX4 `VehicleStatus.nav_state` 原始值；Manual、Position、Offboard 等 PX4 原生模式由此欄位表示 |
| `px4_state_valid` | `bool` | PX4 狀態欄位是否有效：`VehicleStatus` 與 `VehicleLandDetected` 皆在 5 秒內（單調時鐘）收到；只有此值為 `true` 時，`armed`、`landed` 與 `px4_nav_state` 才可信 |

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

### 3.3 FormationEpoch (`taiphoon_agent/msg/FormationEpoch`)

由所有 Agent 透過 `/uav_all_agent/swarm/formation_epochs` 交換 leaderless
編隊的 WGS84 成員 snapshot、parent identity、proposal/adopted 狀態與 formation
設定。Topic 使用 Reliable + Transient Local QoS；只有在 epoch 內所有 members
都發布相同 `snapshot_hash` 的 `ADOPTED` 後，Agent 才啟用新 assignment。

訊息包含 `sequence`、`parent_sequence`、`parent_snapshot_hash`、
`proposer_agent_id`、`sender_agent_id`、`status`、formation geometry、WGS84 center、
排序後的 `member_agent_ids` 與位置陣列、parent assignment、`snapshot_hash`、
`creation_stamp` 與 `valid`。

採用規則（2026-10-15 起）：
- `sequence` 較新的 epoch 即使 parent 與本機 accepted epoch 不同也可成為候選，
  避免 Agent 在 epoch 切換途中關閉後，存活成員停在不同 parent 而互相拒絕。
- 含有已穩定確認離線成員的 epoch 不採用；候選 epoch 15 秒內未收齊全員 `ADOPTED`
  即丟棄並記錄，heartbeat 不會再把它帶回，之後以目前存活成員重新提案。
- 只有收到本趟飛行編隊命令的 Agent 會送出 epoch；非既有成員需近 3 秒內送出過 epoch
  才會被納入重組，因此飛行中重新啟動的 Agent 需重新下達 `FORMATION_*` 才會加入。

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
對外 `control_points` 使用 WGS84 geodetic 全球座標；接收端以自己的固定 Home
轉成公尺制 local NED 後，才建立 B-spline 並執行安全檢查。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 發布時間 |
| `start_time` | `builtin_interfaces/Time` | 軌跡起始時間；active trajectory 重發時保持不變 |
| `trajectory_id` | `uint32` | 單調遞增的軌跡識別碼；同一條 active trajectory 重發時保持不變 |
| `agent_index` | `uint32` | 發布者 Agent 索引 |
| `coordinate_frame` | `uint8` | 固定為 `COORDINATE_FRAME_WGS84_GEODETIC` |
| `frame_revision` | `uint32` | 全球 WGS84 contract 版本；不代表發布者或接收者的 local Home |
| `order` | `uint8` | B-spline 階數 |
| `knot_interval_s` | `float32` | knot 間隔（秒） |
| `control_points` | `float64[]` | 扁平化 WGS84 控制點；每 3 個連續元素為 `[latitude_deg,longitude_deg,height_m]`，長度必須為 3 的倍數 |
| `duration_s` | `float32` | 由 EGO-Swarm 依候選路徑長度與最大速度／加速度／jerk 計算，並在動力學取樣超限時延長的軌跡有效時間（秒） |
| `terminal_hold` | `bool` | 完成任務後固定終點的佔位 heartbeat；只要 heartbeat fresh，接收端持續把終點納入避碰 |
| `valid` | `bool` | 軌跡是否可供同儕使用 |

### 3.7 PlannerStatus (`taiphoon_agent/msg/PlannerStatus`)

EGO-Swarm Planner 狀態與安全指標。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 狀態時間戳 |
| `state` | `uint8` | 規劃器狀態 |
| `trajectory_id` | `uint32` | 目前軌跡識別碼 |
| `map_valid` | `bool` | 障礙物地圖是否有效 |
| `localization_valid` | `bool` | 本 Agent 的 planner-local frame 是否有效 |
| `peer_data_valid` | `bool` | 同儕資料是否可用 |
| `planning_time_ms` | `float32` | 最近一次規劃耗時 |
| `min_obstacle_distance_m` | `float32` | 最近障礙物距離 |
| `min_peer_distance_m` | `float32` | 最近同儕中心距離 |
| `reason` | `string` | 降級或失敗原因 |

> 同儕資料無效且某台同儕的時鐘差估計超過 `max_peer_time_skew_s` 時，`reason` 為
> `peer clock skew agent=<N> offset=<±秒> s exceeds <門檻> s; check time sync`；
> 其餘情況維持 `peer trajectory stale or invalid` 或 `Formation sync waiting for peer trajectory`。
> 控制行為（等待／煞停）不因此改變。主機對時方式見 [多機時間同步](time-sync.md)。

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

### 4.1 PX4 飛控橋接 Topic（命名空間：`uav_{N}`）

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

> **注意**：Topic 名稱為相對路徑，實際解析後會加上節點命名空間，例如 `/uav_1/fmu/in/offboard_control_mode`。

> **偏航**：PX4 Offboard 位置模式不套用 `MPC_YAWRAUTO_MAX`，因此 `TrajectorySetpoint.yaw`
> 由 BasicController 以 `yaw_rate_limit_deg_s`（預設 30 deg/s）每個控制週期沿最短角度逼近目標航向，
> `yawspeed` 填入前饋角速度（rad/s，收斂後為 0）；偏航未到達前即使沒有軌跡也會逐週期重送，
> 位置沿用上一筆已送出的 setpoint。地面起飛鎖定當下航向，目標航向未設定時的防呆值為目前航向（不再送 0／正北）。

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
| uav_1 | `/uav_1_agent/state` | `/uav_1_agent/attitude` |
| uav_2 | `/uav_2_agent/state` | `/uav_2_agent/attitude` |
| uav_3 | `/uav_3_agent/state` | `/uav_3_agent/attitude` |
| uav_4 | `/uav_4_agent/state` | `/uav_4_agent/attitude` |

#### 4.2.2 命令下達（GCS → Agent）

| Topic | 說明 | 型別 | QoS |
|-------|------|------|-----|
| `/{agent_namespace}/command` | 發送命令給**指定** Agent | `std_msgs/String` | reliable (10) |
| `/uav_all_agent/command` | 廣播命令給**所有** Agent | `std_msgs/String` | reliable (10) |

> **GCS 設計建議**：
> - 使用 `/{agent_namespace}/command` 對單一無人機下達移動、模式切換命令。
> - 使用 `/uav_all_agent/command` 發送編隊命令（FORMATION_*）或全體同步命令（如全體 RTL）。
> - 命令格式詳見 [第 6 節](#6-命令協定-command-protocol)。

#### 4.2.3 內部控制流（Agent 內部通訊，GCS 可監控）

| Topic 模板 | 說明 | 型別 | QoS |
|------------|------|------|-----|
| `/uav_all_agent/swarm/agent_trajectories` | 各 Agent 發布的 EGO-Swarm 未來軌跡，供同儕規劃建立 occupancy 與避碰使用；不作為本機 BasicController 控制輸入 | `taiphoon_agent/AgentTrajectory` | reliable (pub) / best_effort (sub), depth 10 |
| `/{agent_namespace}/perception/obstacles` | 本機局部障礙物 PointCloud2 | `sensor_msgs/msg/PointCloud2` | best_effort (pub/sub), depth 10 |
| `/{agent_namespace}/planning/mission_intent` | 本機任務意圖文字命令，供 EGO-Swarm Planner 設定目標 | `std_msgs/msg/String` | reliable (10) |
| `/{agent_namespace}/planning/status` | 本機 EGO-Swarm 規劃狀態與診斷 | `taiphoon_agent/PlannerStatus` | reliable (10) |
| `/{agent_namespace}/planning/trajectory` | 本機 EGO-Swarm 規劃結果，供本機 BasicController 取樣並發布 PX4 setpoint | `taiphoon_agent/AgentTrajectory` | reliable (10) |
| `/uav_all_agent/swarm/formation_epochs` | Leaderless 編隊 epoch、WGS84 snapshot 與 ADOPTED heartbeat | `taiphoon_agent/FormationEpoch` | reliable + transient_local (10) |
| `/{agent_namespace}/ned_target` | 舊 NED 目標介面；EGO-Swarm 取代後僅作相容或純邏輯測試用途 | `taiphoon_agent/NedTarget` | 既有介面 |
| `/{agent_namespace}/waypoints` | 舊 waypoint 介面；EGO-Swarm 取代後僅作純邏輯測試用途，不能視為 PX4 控制來源 | `taiphoon_agent/WaypointSequence` | 既有介面 |

#### 4.2.4 跨 Agent 通訊（同儕發現）

EGO-Swarm 以 AgentState 取得同儕目前狀態，使用 WGS84 位置建立接收端 local NED，並以同儕
未來軌跡進行時間域避碰：

| 角色 | Topic | 用途 |
|--------|-----------|------|
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/state`、`/{peer_namespace}/state` | 取得自身與同儕 WGS84 位置，建立接收端 local NED |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/ned_target` | 接收既有 NED 目標相容輸入 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/planning/mission_intent` | 接收本機任務意圖 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/command` | 接收本機 `MOVE_*`、`MOVE_POINT` 與 `FORMATION_CANCEL`，轉換為 EGO-Swarm 目標；模式、起飛與 YAW 仍由 BasicController 處理 |
| `ego_swarm_planner`（訂閱） | `/{agent_namespace}/attitude` | 取得命令收到當下的機頭 yaw，固定水平 MOVE 的 local NED 目標 |
| `ego_swarm_planner`（訂閱） | `/uav_all_agent/command` | 接收全體編隊與同步命令 |
| `ego_swarm_planner`（訂閱／發布） | `/uav_all_agent/swarm/formation_epochs` | 交換 proposal/adopted、共同 WGS84 snapshot 與編隊 heartbeat |
| `ego_swarm_planner`（訂閱） | `/uav_all_agent/swarm/agent_trajectories` | 取得同儕未來軌跡進行避碰 |
| `ego_swarm_planner`（發布） | `/uav_all_agent/swarm/agent_trajectories` | 發布本機未來軌跡 |
| `ego_swarm_planner`（發布） | `/{agent_namespace}/planning/status`、`/{agent_namespace}/planning/trajectory` | 發布本機規劃狀態與軌跡 |
| `obstacle_cloud_simulator`（發布） | `/{agent_namespace}/perception/obstacles` | 發布 PointCloud2 障礙物資料 |
| `basic_controller_node`（訂閱） | `/{agent_namespace}/planning/trajectory` | 取樣本機 EGO-Swarm 軌跡並發布 PX4 setpoint |

> **同儕資料來源**：AgentState 維持 GCS 與 Agent 的目前狀態介面；EGO-Swarm 的
> 未來軌跡透過 `/uav_all_agent/swarm/agent_trajectories` 交換。EGO planner 每 3 秒
> 掃描同儕 Agent，並以各 Agent 的 state Topic 建立同儕訂閱。
>
> 確認落地或解除解鎖後會清除任務目標、編隊 Epoch／assignment 與 active plan，
> 停止 optimizer 與有效本機控制軌跡；local `planning/trajectory` 發布一次
> `valid=false` 撤銷舊軌跡，同時於 global 軌跡 topic 發布撤銷訊息；已降落機體不再
> 廣播佔位，其他 Agent 依有效落地狀態移除其舊軌跡與編隊成員資格。
> 地面移動／編隊命令不排隊，重新起飛必須重新下達編隊命令，
> 舊 Epoch heartbeat 不會自動恢復編隊。無效 PX4 觀測暫停規劃，但不視為確認落地。
>
> 空中待命或沒有任務時，planner 仍會在 global topic 發布 `terminal_hold=true` heartbeat
> 供同儕保留 occupancy，但不會同步發布到本機 `planning/trajectory`；因此不會改寫
> BasicController 在 HOVER／TAKEOFF 後鎖定的本機 setpoint。移動中、煞車中及任務終點
> 的本機執行軌跡才會同時進入 local topic。
>
> 機群之間一律只訂閱 `/uav_<N>_agent/state` 等 Agent 對外 topic，
> 不直接訂閱其他無人機的 PX4 原生 topic（`/uav_<N>/fmu/out/*`）。

### 4.2.5 EGO-Swarm 編隊與避碰契約

- `MOVE_*`、`MOVE_POINT` 與編隊位置目標由 `ego_swarm_planner` 唯一擁有；
  `basic_controller_node` 只負責 ARM、TAKEOFF、LAND、RTL、HOLD、YAW 與軌跡取樣。
  規劃器輸出未來軌跡後，再由 BasicController 以 analytic B-spline derivative
  取樣 p/v/a 成 PX4 `TrajectorySetpoint`。
- 目前 EGO-Swarm runtime 是內建 pure C++ optimized continuous B-spline planner。水平
  `FIXED_Z` 先經 Local Goal；直線 seed 受阻時由 pure C++ bounded 時空 A* 提供側繞／讓路 seed，
  再進行 control-point optimization。`FREE_3D`、`LINEAR_Z` 與垂直 MOVE 不經此 A*。
  deterministic candidate 類別僅保留相容與回歸用途。
- AgentTrajectory 對外使用 WGS84 全球座標；各接收端以自己的固定 Home 轉成公尺制
  `planner_ned`，再交給 B-spline、動力學限制與同儕避碰。GPS-only 可以正常執行編隊；
  PX4 odometry 與 attitude 仍由 BasicController 用於即時軌跡追蹤。
- 單機 `MOVE_FORWARD/BACKWARD/LEFT/RIGHT/UP/DOWN` 以保存的 mission goal position
  累加，水平位移使用保存的 target yaw。全體 `/uav_all_agent/command` 的同名命令只在
  FormationEpoch 啟用時有效。位移沿固定 NED 軸定義，各 Agent 在自身 `planner_ned`
  套用相同向量後，以 WGS84 FormationEpoch 交換 center；未在編隊時拒絕。
- 全體 `MOVE_POINT x y z yaw` 保留既有行為：收到命令的 Agent 以自身 `planner_ned`
  解讀 `x/y/z`，再轉成 WGS84 Formation center。不同 Home 下這不是唯一的全球絕對目標；
  修正需重新定義命令契約與 proposal／adoption 行為，本次不更動。`yaw` 仍由
  BasicController 以絕對航向語意套用至每台 Agent。
- 單機命令只有尚未建立 target yaw 的第一個水平命令需要新鮮 AgentAttitude；後續命令
  不會因實際 yaw 或 odometry 暫態改變而改寫既有目標。
- `YAW` 的相對角度累加到保存的 target yaw；`MOVE_POINT` 的絕對 yaw 會重設 target yaw。
  BasicController 與 planner 都以「最終 target yaw」為累加基準，BasicController 只把實際送出的
  yaw 限速逼近；逼近期間下達的水平 MOVE 依最終航向計算位移，機頭仍在轉屬預期行為。
  地面起飛時 BasicController 鎖定當下航向，planner 也在落地時清除 target yaw、下次由 attitude 初始化，兩者一致。
- 編隊 assignment 的基礎條件是總移動距離最短：以 FormationEpoch center 為原點的
  canonical NED（只取決於 epoch 內容，各 Agent 逐位元相同）計算三維距離，加上換槽
  penalty（僅 parent 與目前槽位數相同時）做 deterministic Hungarian，再以 crossing
  penalty 減少水平交叉。CIRCLE 的槽位相位不影響隊形，另以 0.5° 解析度搜尋總成本最低
  的相位（同成本取較小相位），因此成員減少時存活成員只做最短的移動，不會整圈換位。
  成員 discovery 連續兩次穩定後自動重編隊。
- 編隊至少需要 2 台穩定成員：少於 2 台的 `FORMATION_*` 直接拒絕、不排隊；編隊中成員
  只剩本機時解散編隊並以目前位置原地懸停，不飛往單機虛擬槽位。
- 槽位中心距離小於 `formation_min_distance_m` 的編隊參數（例如半徑過小）會在命令階段
  拒絕，不會在飛行中才因目標無效而煞停。
- 編隊 transition 每次只釋放一台安全前序 Agent；路徑優先使用直線，受阻時只允許
  North/East 水平側向折線，Z 僅沿起點到目標線性插值，禁止以額外的上／下偏移繞行。
  無安全水平候選時維持 terminal hold，等前序 Agent 到位後依 deterministic 順序放行。
- Agent 失效或重新上線會在 discovery 成員集合連續兩次穩定後建立下一代 FormationEpoch，
  移除或納回成員並重新計算 assignment。既有成員離開時，新 epoch 採用前先維持安全 hold；
  新成員加入時既有成員繼續維持目前槽位，直到新 epoch 全員 `ADOPTED`。
- 飛行中重新啟動的 Agent 不會自動採用既有編隊，需重新下達 `FORMATION_*`；
  存活成員不會因等待它而停止。
- 同儕避碰的距離定義為機體中心到機體中心的三維直線距離，不加計機體半徑。
- 最小距離不得在文件或實作中硬編碼為 3 公尺；Launch 與 entrypoint 以
  `formation_min_distance_m`、`peer_collision_min_distance_m` 提供設定，可使用
  1.5 m、2 m 或 3 m 等值。
- ACTIVE 與 TERMINAL_HOLD 且 fresh 的同儕未來軌跡會進入驗證；stale 或 invalid 的
  資料不會繼續使用，關鍵資料失效時改由 `EmergencyBrakePlanner` fail-closed 煞車。
  TERMINAL_HOLD 由完成任務的 Agent 持續 heartbeat，保留最終位置供其他 Agent 迴避。
- `ContinuousSafetyValidator` 獨立於 planner backend，逐段取樣檢查 finite p/v/a/j、
  frame/time、map／peer freshness、動力學限制、同儕 separation 與 obstacle clearance。
  Candidate 必須依序通過 planner、reservation（若需要）與 safety validation 才能更新
  active state 並發布 `AgentTrajectory`；失敗則保留安全舊路線或發布 dynamic braking trajectory。
- PointCloud2 的 `frame_id="swarm_ned"` 是既有障礙物介面；模擬器目前以
  `swarm_origin_agent_index` 建立共享場景原點。planner 內部則使用 receiver 自己的
  `planner_ned`，現行 callback 尚未執行兩者間的 frame conversion。修正這個邊界會
  改變 runtime 行為，因此本次只記錄現況，不改 frame_id 或轉換流程。YAML 障礙物描述
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
- 不提供也不允許 `/uav_all_agent/shutdown` 廣播；shutdown 必須指定單一 Agent namespace。
- 若 Compose 服務使用 `restart: no`，PID 1 終止後容器會保持停止，不會自動重新啟動。若使用自動重啟策略，容器可能重新啟動，服務也會再次出現，將無法保持失效狀態。

**呼叫範例**：

```bash
ros2 service call /uav_1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

**驗證服務與容器狀態**：

```bash
# 在 Agent 容器內確認服務存在與型別
ros2 service list | grep -F /uav_1_agent/shutdown
ros2 service type /uav_1_agent/shutdown
```

```bash
# 在 Agent 容器內呼叫；回應送出後容器 PID 1 應立即終止
ros2 service call /uav_1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

```bash
# 回到宿主機確認容器狀態與退出碼
docker compose ps uav_1_agent
docker inspect --format '{{.State.Status}} exit={{.State.ExitCode}} restart={{.RestartCount}}' uav_1_agent
```

## 6. 命令協定 (Command Protocol)

### 6.1 傳輸格式

- **Topic**：`/{agent_namespace}/command` 或 `/uav_all_agent/command`
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
| `YAW <angle>` | 相對角度 (度) | 累加到保存的目標航向（順時針為正），由 BasicController 以 `yaw_rate_limit_deg_s` 限速沿最短角度轉向 |
| `MOVE_POINT <x> <y> <z> <yaw>` | NED 座標 + 絕對偏航 | 單機時為 Agent 目標；全體編隊時由 proposal Agent 的 planner-local NED 解讀 Formation center，yaw 由 BasicController 設定並限速轉向 |

#### 基礎控制命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `ARM` | 無 | 解鎖 |
| `DISARM` | 無 | 上鎖 |
| `TAKEOFF [altitude]` | 高度 (公尺，預設 3.0) | 起飛 |

#### 編隊命令（僅透過 `/uav_all_agent/command` 廣播）

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `FORMATION_CIRCLE <radius> <altitude>` | 半徑 (公尺), 高度 (公尺) | 環形編隊 |
| `FORMATION_HORI_LINEAR <spacing> <altitude>` | 間距 (公尺), 高度 (公尺) | 水平一字編隊 |
| `FORMATION_VERT_LINEAR <spacing>` | 間距 (公尺) | 垂直一字編隊（Taiphoon_QGCS 暫未提供此按鈕） |
| `FORMATION_V_SHAPE <spacing> <altitude>` | 翼上間距 (公尺), 高度 (公尺) | 燕行（V 字）編隊：頂點朝 North，其餘機依序左、右交替排在後方兩翼，半角 45°；同一翼相鄰兩機距離為 `spacing`，2 台時為斜一字 |
| `FORMATION_CANCEL` | 無 | 取消編隊 |

> 編隊至少需要 2 台已起飛的穩定成員；少於 2 台時 Agent 拒絕 `FORMATION_*`。

> 全體 `MOVE_*`／`MOVE_POINT` 必須在已啟用 FormationEpoch 時使用；全體相對命令的方向
> 固定為 `FORWARD=+North`、`BACKWARD=-North`、`RIGHT=+East`、`LEFT=-East`、
> `UP=-Down`、`DOWN=+Down`。取消編隊或單機移動後，重新發送相同 Formation command
> 會重新啟用既有 Epoch，不重建 assignment 或 center。

> **EGO-Swarm 編隊基準點**：收到 `FORMATION_*` 指令後，由 `ego_swarm_planner`
> 使用 WGS84 位置交換與各自固定 Home 的 local NED 計算共同參考點與虛擬編隊槽位；每台 Agent 自行規劃前往
> 自己的槽位。既有命令字串與廣播 Topic 保持不變。成員掃描週期為 3 秒。
> 編隊高度（`<altitude>`）一律以提案 Agent 的 Home 高度為基準；即使先移動再下達編隊，
> 初次中心也只取成員目標的水平平均，不會再疊加目前飛行高度。

#### 相容性簡寫

| 命令 | 簡寫 |
|------|------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

### 6.3 命令使用範例

```bash
# 讓 uav_1 起飛至 5 公尺
ros2 topic pub /uav_1_agent/command std_msgs/msg/String "data: 'TAKEOFF 5'"

# 讓所有無人機返航
ros2 topic pub /uav_all_agent/command std_msgs/msg/String "data: 'MODE_RTL'"

# 環形編隊（半徑 10 公尺，高度 20 公尺）
ros2 topic pub /uav_all_agent/command std_msgs/msg/String \
  "data: 'FORMATION_CIRCLE 10 20'"

# uav_2 向前移動 3 公尺
ros2 topic pub /uav_2_agent/command std_msgs/msg/String "data: 'MOVE_FORWARD 3'"
```

---

## 7. 命名空間規則

PX4 透過 `UXRCE_DDS_NS_IDX` 產生索引式 namespace。SITL 與實機部署都必須使用
相同的 1-based UAV 索引：`UXRCE_DDS_NS_IDX=1` 對應 `/uav_1`，
`UXRCE_DDS_NS_IDX=2` 對應 `/uav_2`。因此 PX4 的相對 `fmu/...` topic 解析後分別為
`/uav_1/fmu/...` 與 `/uav_2/fmu/...`；Agent 對外 topic 則分別使用
`/uav_1_agent/...` 與 `/uav_2_agent/...`。

SITL launcher 以 `PX4_PARAM_UXRCE_DDS_NS_IDX=N` 將此參數傳入 PX4。對尚未內建
`UXRCE_DDS_NS_IDX` 的舊版 PX4 SITL image，launcher 另以相同結果的
`PX4_UXRCE_DDS_NS=uav_N` 作為相容 fallback，確保實際 topic 仍為 `/uav_N/fmu/...`；
具備新參數的 PX4 則由 `UXRCE_DDS_NS_IDX` 決定同一個 namespace。`fmu` 僅保留作為
PX4 uXRCE-DDS topic 的固定路徑元件，不代表 UAV 的實例識別碼。

| 命名空間 | 用途 | 範例 |
|----------|------|------|
| `uav_{N}` | 節點命名空間，對應 PX4 實例 | `uav_1`, `uav_2`, `uav_3` |
| `uav_{N}_agent` | Agent 對外 Topic 前綴 | `uav_1_agent`, `uav_2_agent` |
| `uav_all_agent` | 全體 Agent 廣播 Topic | `/uav_all_agent/command` |

**規則**：
- 一個 Taiphoon_Agent 只控制一台 PX4（`uav_{N}`）。
- 所有 Agent 對外 Topic 使用 `/{agent_namespace}/...` 前綴。
- 廣播命令固定使用 `/uav_all_agent/command`。
- Shutdown service 僅使用 `/{agent_namespace}/shutdown`，不提供 `/uav_all_agent/shutdown`。

---

## 8. QoS 設定摘要

| 介面類型 | Reliability | Durability | Depth |
|-----------|-------------|------------|-------|
| PX4 飛控 Topic | best_effort | volatile | 10 |
| Agent 狀態 Topic (`/state`, `/attitude`) | reliable | volatile | 10 |
| 命令 Topic (`/command`) | reliable | volatile | 10 |
| Shutdown Service (`/shutdown`) | reliable | volatile | 10 |
| 控制流 Topic (`/ned_target`, `/waypoints`) | reliable (pub) / best_effort (sub) | volatile | 10 |
| `/uav_all_agent/swarm/agent_trajectories` | reliable (pub) / best_effort (sub) | volatile | 10 |
| `/{agent_namespace}/perception/obstacles` | best_effort | volatile | 10 |
| `/{agent_namespace}/planning/status`、`/planning/trajectory` | reliable | volatile | 10 |

> 上述 QoS 與目前 `ego_swarm_planner`、`obstacle_cloud_simulator` 實作一致。

---

## 9. EGO 路徑規劃 SITL 驗證

測試腳本不新增控制介面；它只使用既有 PX4 launcher、Agent command topic、
`PlannerStatus` 與 `AgentTrajectory`：

```bash
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario single
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario east
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario dual
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario swarm
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario all
```

水平 MOVE 的 `FIXED_Z` 軌跡會維持起點高度；直線 seed 受阻時才使用 bounded 時空 A*
引導 B-spline。runtime 固定使用 10 個控制點，duration 以舊 6 點 cubic 的 3 spans
作為 timing reference，實際 knot interval 為 `duration/(10-3)`；直線 seed 使用
smoothstep，並由 hard dynamic validator 保護 velocity/acceleration/jerk。側移候選在
solver 內依 `0.5, 1, 1.5, 2, 3, 4, 6, 8, 10 m` 由小到大評估，同一幅度先選最低
成本且偏移較小者。`FREE_3D`、`LINEAR_Z`、垂直 MOVE、FormationEpoch、reservation、
priority 與既有 HOLD／Brake 行為不因這個 initializer 改變。

SITL 單機／雙機路徑驗收會先啟動高度監測器取得地面基準，再使用既有 `TAKEOFF 8.0`；
四機 Formation 驗收則使用既有 `TAKEOFF 10.0`，再發送 `FORMATION_CIRCLE 10 10`，讓
編隊高度與起飛高度一致。讀取
既有 `AgentState.position_global`，確認每台 Agent 相對起飛基準至少上升 `7.5 m`
（單機／雙機）或 `9.5 m`（四機 Formation），並穩定 1 秒後才發布 MOVE 或 Formation；
兩者皆允許 0.5 m SITL 誤差。`east` 另使用 `config/obstacles/east_wall.yaml`：牆中心
為 `[North=0, East=15, Down=-8]`、尺寸 `[4, 0.5, 4]`，固定高度禁止垂直繞行；驗收
要求 East progress ≥29 m、障礙 clearance ≥1 m、North 橫偏 3.0～4.5 m、路徑長 ≤35 m。
Monitor 會輸出 `east_progress_m`、`max_state_north_offset_m`、
`max_trajectory_north_offset_m`、`flown_path_length_m`、高度與 planner failure 指標，
並將完整命令與結果寫入 `AutoTest/Log/`。這是測試前置條件，不新增 runtime topic 或命令。

---

### Windows 小場域三機驗證

`AutoTest/scripts/small_field_three_agent_validation.ps1` 在獨立 Docker observer 中啟動
測試節點 `/autotest/small_field_three_agent_validation`；此節點不屬於正式 Agent launch。
沿用三台 Agent 的 `state`、`planning/status`、全體 FormationEpoch／AgentTrajectory，
以及既有 `command`、`shutdown` 介面，不新增 runtime message 或 topic。
前五步驗證與交叉移動均監測 9×9×8 m 場域與 2 m 中心距離；observer 成功仍須使用
`analyze_small_field_px4_logs.py` 離線查核失效機原始日誌，不能以 peer 被移除推論飛機已消失。
本輪不下達 RTL，PX4 自主失聯處置仍由既有參數決定。

> **文件維護**：此文件由 Taiphoon_Agent 團隊維護，每次新增或修改 Topic/Service/Node 時應同步更新。
