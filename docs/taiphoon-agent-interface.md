# Taiphoon Agent ROS2 地面控制站 (GCS) 介面規格書

> **版本**：2.0
> **更新日期**：2026-10-30
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

**事件與健康狀態**：發布 `/{agent_namespace}/events`（`AgentEvent`，命令與 PX4 類事件 100～305）
與 `/{agent_namespace}/health`（`AgentHealth`，PX4 完全沒有資料時也照常發布，age 為 -1）。
另訂閱 PX4 `fmu/out/failsafe_flags`（best_effort）；PX4 沒有輸出這個 topic 時
`failsafe_flags_valid=false`，其他功能不受影響。

**事件與 REBOOT_FC 相關參數**：

| 參數 | 型別 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `health_publish_rate_hz` | `double` | `1.0` | `health` 發布頻率；非有限值、≤ 0 或大於 10 時回到 1.0 |
| `px4_command_ack_timeout_s` | `double` | `3.0` | 送出 VehicleCommand 後等待 ack 的時限；逾時發布 `PX4_COMMAND_TIMEOUT` |
| `reboot_cooldown_s` | `double` | `15.0` | `REBOOT_FC` 送出後的冷卻時間；冷卻中一律拒絕 |

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
- `WaypointMissionExecutor`：單機航點任務的驗證、WGS84／PX4 local 高度換算、狀態機、停留與單段逾時；
  每段航點以一般 Final Target 流程規劃（同儕避碰、障礙物、reservation 與 safety validator 全部照舊），
  詳見 [4.3 航點任務](#43-航點任務waypoint-mission)

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
| `formation_slot_switch_penalty_m` | `double` | `2.0` | minimum-cost assignment 換槽位成本；只在 parent 與目前隊形類型相同、槽位數也相同時加計 |
| `formation_member_stable_scan_count` | `int` | `2` | 成員集合連續穩定 discovery 次數 |
| `planner_backend` | `string` | `optimized` | 只接受 `optimized`；其他值使 planner 初始化失敗 |
| `optimizer_max_iterations` | `int` | `40` | optimized projected-gradient iteration 上限 |
| `optimizer_time_budget_ms` | `double` | `70.0` | optimized backend 單次 wall-time budget；時空 A* 最多使用 60%，其餘保留給 solver |
| `formation_crossing_penalty_m` | `double` | `10.0` | assignment 水平 crossing 等價成本 |
| `reservation_lead_time_s` | `double` | `0.0` | future trajectory reservation 最短提前時間；實機參數下 0.5 秒會使隊形切換最近距離降至 1.4～2.3 m，預設與開發環境一致為 0.0 |
| `reservation_step_s` | `double` | `0.5` | reservation start time 搜尋步進 |
| `reservation_horizon_s` | `double` | `5.0` | reservation start time 最遠搜尋範圍 |
| `mission_max_waypoints` | `int` | `50` | 單一航點任務的航點上限（1～65535） |
| `mission_min_altitude_m` | `double` | `1.0` | 航點高度下限（PX4 local，向上為正） |
| `mission_max_altitude_m` | `double` | `120.0` | 航點高度上限；不大於下限時兩者都回到預設 |
| `mission_max_range_m` | `double` | `1000.0` | 每個航點與目前位置的水平距離上限；上傳與 `MISSION_START` 時各檢查一次 |
| `mission_max_dwell_s` | `double` | `60.0` | 航點停留時間上限 |
| `mission_leg_base_timeout_s` | `double` | `30.0` | 單段逾時的基本時間 |
| `mission_min_expected_speed_mps` | `double` | `0.3` | 單段逾時＝基本時間＋段長 ÷ 此速度 |
| `mission_load_timeout_s` | `double` | `10.0` | 上傳後未收到 `MISSION_START` 即清除的時限 |
| `mission_status_rate_hz` | `double` | `1.0` | `mission/status` 週期重送頻率（狀態改變時另外立即發布）；大於 10 時回到 1.0 |
| `planner_failure_event_s` | `double` | `3.0` | 連續規劃失敗超過此秒數發布一次 `PLANNER_FAILURE_PERSISTENT` |

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
原始資料更新較慢，期間會重複最近一筆有效值，不進行插值。收到第一筆 `VehicleOdometry` 之前不發布
AgentState（位置未定義，避免同儕與 planner 把無效位置當成真實位置）；這段期間 GCS 以 `health`
（固定發布）判斷 Agent 在線與 PX4 連線狀態。

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
| `armed` | `bool` | PX4 實際解鎖狀態；`px4_state_valid=false` 時保留最後收到的值，從未收到時為 `false`（以 `health.px4_arming_state` 區分） |
| `landed` | `bool` | PX4 實際落地狀態；`px4_state_valid=false` 時保留最後收到的值，從未收到時為 `false`（以 `health.px4_land_detected_age_s` 區分） |
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

### 3.9 MissionWaypoint (`taiphoon_agent/msg/MissionWaypoint`)

航點任務的單一航點。水平位置使用 WGS84，高度基準由 `WaypointMission.altitude_frame` 決定。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `latitude_deg` | `float64` | WGS84 緯度（度） |
| `longitude_deg` | `float64` | WGS84 經度（度） |
| `altitude_m` | `float32` | 高度（公尺，向上為正） |
| `dwell_s` | `float32` | 到達後停留時間（秒）；最後一個航點不使用，到達即完成並懸停 |
| `yaw_deg` | `float32` | 保留欄位；本版只接受 NaN（維持目前目標航向） |

### 3.10 WaypointMission (`taiphoon_agent/msg/WaypointMission`)

單機航點任務上傳。上傳只載入任務、不會移動；需另送 `MISSION_START <mission_id>` 才開始。

| 欄位／常數 | 型別 | 說明 |
|------|------|------|
| `ALTITUDE_PX4_LOCAL=1` | `uint8` 常數 | 高度與 `-AgentState.position_ned[2]` 同基準（PX4 local 原點起算），與 QGCS 機群面板顯示的高度一致 |
| `stamp` | `builtin_interfaces/Time` | 發布時間 |
| `mission_id` | `uint32` | GCS 指定的任務識別碼，不得為 0 |
| `altitude_frame` | `uint8` | 高度基準；本版只接受 `ALTITUDE_PX4_LOCAL`（0 視為未設定而拒絕） |
| `waypoints` | `MissionWaypoint[]` | 依序執行的航點 |

### 3.11 WaypointMissionStatus (`taiphoon_agent/msg/WaypointMissionStatus`)

單機航點任務狀態。狀態、`current_index` 或原因改變時立即發布，並以 `mission_status_rate_hz` 週期重送。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 發布時間 |
| `mission_id` | `uint32` | 狀態所屬任務；IDLE 時為 0 |
| `state` | `uint8` | `IDLE=0`、`LOADED=1`、`RUNNING=2`、`DWELLING=3`、`COMPLETED=4`、`ABORTED=5`、`REJECTED=6` |
| `reason` | `uint8` | REJECTED／ABORTED 的原因，其他狀態為 `REASON_NONE=0` |
| `current_index` | `uint16` | 目前航點（0 起算）；`INVALID_PLAN`／`OUT_OF_RANGE` 時指向第一個不合法航點 |
| `waypoint_count` | `uint16` | 航點數 |
| `detail` | `string` | 英文補充說明（例如觸發中止的命令名稱） |

| 原因碼 | 值 | 說明 |
|--------|----|------|
| `REASON_NONE` | 0 | — |
| `REASON_INVALID_PLAN` | 1 | 航點數量、座標、高度、停留、航向或高度基準不合法 |
| `REASON_OUT_OF_RANGE` | 2 | 航點與目前位置的水平距離超過 `mission_max_range_m` |
| `REASON_NOT_READY` | 3 | 尚無有效狀態，或 Home／PX4 local 換算尚未建立 |
| `REASON_NOT_HOVERING` | 4 | 未在 Offboard 懸停（未解鎖、已落地、移動中或遙控器接手） |
| `REASON_BUSY` | 5 | 已有任務執行中，或編隊中 |
| `REASON_ID_MISMATCH` | 6 | `MISSION_START` 的識別碼與已載入任務不符，或載入已逾時 |
| `REASON_OPERATOR_STOP` | 10 | 收到 `MISSION_STOP` |
| `REASON_COMMAND_OVERRIDE` | 11 | 收到其他有效命令（操作人員介入），`detail` 為命令名稱 |
| `REASON_FLIGHT_STATE` | 12 | 飛行狀態離開 Offboard 懸停／移動（PX4 自行 RTL、LAND、failsafe 或遙控器切換） |
| `REASON_LANDED` | 13 | 已落地、上鎖或偵測到 PX4 重新啟動 |
| `REASON_LEG_TIMEOUT` | 14 | 單段逾時未到達 |

> `REJECTED` 屬於事件：只對被拒的 `mission_id` 發布一次，目前狀態不變；下一次週期發布回到實際狀態
> （可能是另一個任務的 RUNNING）。

### 3.12 AgentEvent (`taiphoon_agent/msg/AgentEvent`)

Agent 事件（錯誤、警告、資訊）。`basic_controller_node` 與 `ego_swarm_planner` 都發布到同一個
`/{agent_namespace}/events`。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 事件時間 |
| `sequence` | `uint32` | 各來源節點各自從 1 單調遞增，供 GCS 偵測遺失與去重（以 `source` + `sequence` 判斷） |
| `severity` | `uint8` | `SEVERITY_INFO=0`、`SEVERITY_WARNING=1`、`SEVERITY_ERROR=2` |
| `source` | `uint8` | `SOURCE_BASIC_CONTROLLER=1`、`SOURCE_PLANNER=2`、`SOURCE_SUPERVISOR=3`（保留） |
| `code` | `uint16` | 事件代碼，見下表 |
| `command` | `string` | 相關命令名稱（例如 `ARM`、`REBOOT_FC`），沒有時為空字串 |
| `px4_result` | `int16` | 相關的 PX4 `VehicleCommandAck.result`；沒有時為 -1 |
| `detail` | `string` | 英文補充說明，供日誌與未知代碼顯示 |

**事件代碼**（只增不改；GCS 對未知代碼顯示 `detail`）：

| 代碼 | 名稱 | 嚴重度 | 來源 | 觸發 |
|------|------|--------|------|------|
| 100 | `COMMAND_UNKNOWN` | WARNING | basic_controller | 無法解析的命令字串 |
| 101 | `COMMAND_INVALID_PARAMS` | WARNING | basic_controller | 參數缺少或不合法（例如 `TAKEOFF abc`、`YAW` 無參數、`MISSION_START 1.5`） |
| 102 | `COMMAND_REJECTED` | WARNING | basic_controller | Agent 狀態不允許（`REBOOT_FC` 安全條件或冷卻、全體 topic 的 `MISSION_*`／`REBOOT_FC`），`detail` 寫明原因 |
| 103 | `COMMAND_FAILED` | ERROR | basic_controller | 命令處理失敗（例如 Offboard 前置條件不成立） |
| 200 | `PX4_COMMAND_REJECTED` | ERROR | basic_controller | `VehicleCommandAck` 不是 ACCEPTED／IN_PROGRESS；`command` 為 ARM、DISARM、MODE_OFFBOARD、HOLD、MODE_RTL、MODE_LAND 或 REBOOT_FC，`px4_result` 為結果 |
| 201 | `PX4_COMMAND_TIMEOUT` | ERROR | basic_controller | `px4_command_ack_timeout_s` 內沒有 ack，或 Offboard 等待確認逾時；`REBOOT_FC` 逾時的 `detail` 提示觀察 `health` 是否恢復 |
| 202 | `PX4_COMMAND_ACCEPTED` | INFO | basic_controller | 只用於 ARM、DISARM、REBOOT_FC |
| 203 | `TAKEOFF_CANCELLED` | WARNING | basic_controller | 排隊中的 TAKEOFF 因等待解鎖逾時取消 |
| 204 | `OFFBOARD_LOST` | WARNING | basic_controller | PX4 離開 Offboard（failsafe 或遙控器切換） |
| 300 | `PX4_STATUS_STALE` | ERROR | basic_controller | VehicleStatus 啟動 5 秒內從未收到，或超過 5 秒未更新（轉換時發布一次） |
| 301 | `PX4_STATUS_RECOVERED` | INFO | basic_controller | VehicleStatus 恢復 |
| 302 | `PX4_POSITION_STALE` | WARNING | basic_controller | VehicleOdometry 或 VehicleGlobalPosition 超過 1 秒未更新（`detail` 標明哪一個） |
| 303 | `PX4_PREFLIGHT_FAILED` | WARNING | basic_controller | `pre_flight_checks_pass` 由 true 轉 false，或收到 ARM 時為 false；不去抖動，PX4 降落自動上鎖的瞬間可能短暫回報 false 而出現一筆，GCS 以 `health.px4_pre_flight_checks_pass` 確認是否持續 |
| 304 | `PX4_FAILSAFE` | WARNING | basic_controller | `failsafe` 由 false 轉 true |
| 305 | `PX4_RESTART_DETECTED` | INFO | basic_controller | 偵測到 PX4 重新連線或重新啟動（上鎖狀態下 VehicleStatus 逾時後恢復，或 `REBOOT_FC` 被接受後中斷再恢復） |
| 400 | `PLANNER_COMMAND_REJECTED` | WARNING | planner | 編隊成員不足、幾何不安全、沒有新鮮姿態、Home 未建立、未在編隊中下達全體 MOVE 等 |
| 401 | `PLANNER_EMERGENCY_BRAKE` | WARNING | planner | planner 啟動緊急煞停（同儕距離、規劃失敗等）；任務停止與到達時仍在移動的預期煞停不發布 |
| 402 | `PLANNER_FAILURE_PERSISTENT` | ERROR | planner | 連續規劃失敗超過 `planner_failure_event_s`（每輪只發布一次） |
| 500 | `SAFETY_POSITION_DEVIATION` | ERROR | — | 保留；PositionSafetyMonitor 介入目前停用，暫不發布 |

- 同一代碼與 `command` 5 秒內只發布一次；狀態轉換事件（300、301、303、304）只在轉換時發布、不節流。
- 原有的 WARN／ERROR 日誌保留，事件是額外發布。
- 事件使用 volatile：GCS 連上前的舊事件不重播；持續性的問題由 `health` 週期呈現。

### 3.13 AgentHealth (`taiphoon_agent/msg/AgentHealth`)

Agent 與 PX4 連線健康狀態，`basic_controller_node` 以 `health_publish_rate_hz`（預設 1 Hz）發布。
`*_age_s` 為距離最後一筆訊息的秒數，從未收到為 -1。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 發布時間 |
| `px4_status_age_s` | `float32` | VehicleStatus |
| `px4_odometry_age_s` | `float32` | VehicleOdometry |
| `px4_global_position_age_s` | `float32` | VehicleGlobalPosition |
| `px4_land_detected_age_s` | `float32` | VehicleLandDetected |
| `px4_battery_age_s` | `float32` | BatteryStatus |
| `px4_command_ack_age_s` | `float32` | VehicleCommandAck；沒有命令時會很大 |
| `px4_pre_flight_checks_pass` | `bool` | 以下 `px4_*` 欄位只在 VehicleStatus 新鮮時可信 |
| `px4_failsafe` | `bool` | PX4 failsafe 是否啟動 |
| `px4_arming_state` | `uint8` | VehicleStatus.arming_state（1=DISARMED、2=ARMED，0 表示從未收到）；逾時後保留最後收到的值 |
| `px4_latest_arming_reason` | `uint8` | VehicleStatus.latest_arming_reason |
| `failsafe_flags_valid` | `bool` | 5 秒內收到 `fmu/out/failsafe_flags` 才為 true |
| `failsafe_flags` | `uint32` | `FAILSAFE_*` 位元；無效時為 0 |
| `offboard_session_active` | `bool` | Agent 是否持有 Offboard session |

| 位元常數 | 值 | PX4 FailsafeFlags 欄位 |
|----------|----|------------------------|
| `FAILSAFE_LOCAL_POSITION_INVALID` | 1 | `local_position_invalid` |
| `FAILSAFE_GLOBAL_POSITION_INVALID` | 2 | `global_position_invalid` |
| `FAILSAFE_HOME_POSITION_INVALID` | 4 | `home_position_invalid` |
| `FAILSAFE_ATTITUDE_INVALID` | 8 | `attitude_invalid` |
| `FAILSAFE_MANUAL_CONTROL_LOST` | 16 | `manual_control_signal_lost` |
| `FAILSAFE_OFFBOARD_SIGNAL_LOST` | 32 | `offboard_control_signal_lost` |
| `FAILSAFE_BATTERY_WARNING` | 64 | `battery_warning`（非 0 即設定） |
| `FAILSAFE_BATTERY_UNHEALTHY` | 128 | `battery_unhealthy` |
| `FAILSAFE_GEOFENCE_BREACHED` | 256 | `geofence_breached` |
| `FAILSAFE_FD_CRITICAL_FAILURE` | 512 | `fd_critical_failure` |
| `FAILSAFE_FD_ESC_ARMING_FAILURE` | 1024 | `fd_esc_arming_failure` |
| `FAILSAFE_LOCAL_POSITION_ACCURACY_LOW` | 2048 | `position_accuracy_low` |

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
| `fmu/out/failsafe_flags` | 訂閱 ← PX4 | `px4_msgs/FailsafeFlags` | best_effort (10)；不存在時只讓 `health.failsafe_flags_valid=false` |

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
| `/{agent_namespace}/mission/status` | 航點任務狀態（ego_swarm_planner） | `taiphoon_agent/WaypointMissionStatus` | 改變時立即＋1 Hz 重送 | reliable、transient_local (1) |
| `/{agent_namespace}/events` | Agent 事件（basic_controller_node 與 ego_swarm_planner） | `taiphoon_agent/AgentEvent` | 事件發生時 | reliable、volatile (20) |
| `/{agent_namespace}/health` | Agent 與 PX4 連線健康狀態（basic_controller_node） | `taiphoon_agent/AgentHealth` | 1 Hz | reliable、volatile (1) |

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
| `/{agent_namespace}/mission/plan` | 上傳航點任務給**指定** Agent（只載入，不會移動） | `taiphoon_agent/WaypointMission` | reliable、volatile (10) |

> **GCS 設計建議**：
> - 使用 `/{agent_namespace}/command` 對單一無人機下達移動、模式切換命令。
> - 使用 `/uav_all_agent/command` 發送編隊命令（FORMATION_*）或全體同步命令（如全體 RTL）。
> - 命令格式詳見 [第 6 節](#6-命令協定-command-protocol)。
> - 航點任務先發布 `mission/plan`，確認 `mission/status` 為 `LOADED` 後再送 `MISSION_START <mission_id>`；
>   流程詳見 [4.3 航點任務](#43-航點任務waypoint-mission)。

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

> `ego_swarm_planner` 也訂閱 `/{agent_namespace}/events`，只處理 `source=SOURCE_BASIC_CONTROLLER`
> 的 `PX4_RESTART_DETECTED`：清除任務與編隊，並重建 planner-local 至 PX4 local 的平移（planner Home 保留）。

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
  penalty（僅 parent 與目前隊形類型相同、槽位數也相同時）做 deterministic Hungarian，再以 crossing
  penalty 減少水平交叉。CIRCLE 的槽位相位不影響隊形，另以 0.5° 解析度搜尋總成本最低
  的相位（同成本取較小相位），因此成員減少時存活成員只做最短的移動，不會整圈換位。
  成員 discovery 連續兩次穩定後自動重編隊。
  隊形類型改變（例如一字 → 燕行）時，提案端不帶入 parent 槽位歸屬，因為兩種隊形的
  槽位編號代表不同幾何角色，指派只依新隊形的幾何決定。
- 編隊到位以 planner-local 誤差 ≤ `global_arrival_tolerance_m` 判斷，並直接以槽位座標
  建立 terminal hold；PX4 會以位置控制直線飛完剩下的距離，不經 optimizer 與 validator。
  因此只有「目前位置 → 槽位」直線與每台編隊同儕的路徑都保持
  `max(formation_min_distance_m, peer_collision_min_distance_m)` 時才直接到位；同儕路徑為
  即時位置接上 fresh 軌跡未來 4 s 的取樣（terminal hold 同儕即為「即時位置 → hold 槽位」）。
  不通過或同儕資料無效時繼續一般規劃，直到 PX4 local 誤差 ≤ `local_arrival_tolerance_m`
  才建立 terminal hold；已建立的同槽位 hold 不重新檢查。
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

### 4.3 航點任務（Waypoint Mission）

單機航點任務由本機 `ego_swarm_planner` 執行；GCS 斷線不影響執行。

```
         上傳成功                MISSION_START(id 相符)
 IDLE ──────────────▶ LOADED ─────────────────────────▶ RUNNING ◀──────┐
  ▲  ◀── 載入逾時 ─────┘ │                                │  到達非最後點   │ 停留結束
  │      MISSION_STOP    │ 新上傳取代                      ▼              │
  │                      ▼                              DWELLING ───────┘
  │                    LOADED(新 id)                       │
  │                                                   到達最後點 ▼
  └──── 新上傳時才離開 ── COMPLETED／ABORTED ◀── 中止條件（RUNNING／DWELLING）
```

**上傳驗證**（不符即發布一次 `REJECTED`，目前狀態不變）：任務執行中或編隊中（BUSY）；
`mission_id == 0`、`altitude_frame` 不是 `ALTITUDE_PX4_LOCAL`、航點數為 0 或超過上限、經緯度不合法、
高度不在 `mission_min_altitude_m`～`mission_max_altitude_m`、`dwell_s` 不合法、`yaw_deg` 不是 NaN（INVALID_PLAN）；
沒有有效 AgentState、planner Home 或 PX4 local 換算（NOT_READY）；任一航點水平距離超過上限（OUT_OF_RANGE）。
上傳本身不需在懸停；新上傳會取代已載入、已完成或已中止的任務。

**座標換算**：水平以 planner Home 由 WGS84 換算；垂直以 PX4 local 換算
（`z_planner = -altitude_m - planner_to_px4_ned_offset.z`）。換算在上傳時完成並保存，執行中不重算。

**開始條件**（`MISSION_START <id>`，不符即 `REJECTED`）：狀態為 LOADED 且識別碼相符（ID_MISMATCH）；
PX4 為 Offboard（`px4_nav_state=14`）、AgentState `flight_mode=HOVER`、`px4_state_valid`（NOT_HOVERING）；
不在編隊中（BUSY）；再檢查一次水平距離上限（OUT_OF_RANGE）。開始時不改變目標航向。

**執行**：每段以航點為 Final Target，到達判斷與單機 `MOVE_POINT` 相同（PX4 local 與 planner-local 都在容許誤差內）。
非最後一點到達後停留 `dwell_s`（terminal hold），最後一點到達即 COMPLETED 並懸停。
單段逾時＝`mission_leg_base_timeout_s`＋段長 ÷ `mission_min_expected_speed_mps`，同儕仲裁、煞停與規劃失敗的等待都計入。

**中止條件**：

| 觸發 | 原因碼 | Agent 動作 |
|------|--------|------------|
| `MISSION_STOP` | `OPERATOR_STOP` | 依目前速度煞停並停在原地 |
| 單機或全體 topic 收到任何非 `MISSION_*` 的有效命令（含 `HOLD`、`YAW`、`ARM`、`MODE_RTL`、`MODE_LAND`、`FORMATION_*`、`REBOOT_FC`）；`ned_target`、`mission_intent` 收到目標 | `COMMAND_OVERRIDE`（`detail` 為命令名稱） | 先煞停並停在原地，再照常處理該命令 |
| PX4 離開 Offboard，或 `flight_mode` 不是 HOVER／MOVING（PX4 自行 RTL、LAND、failsafe、遙控器切換） | `FLIGHT_STATE` | 只清除 planner 目標，不發布停止軌跡（控制權已在 PX4） |
| 確認落地、上鎖或偵測到 PX4 重新啟動 | `LANDED` | 隨既有流程清除 |
| 單段逾時 | `LEG_TIMEOUT` | 煞停並停在原地 |

- 無法解析或參數不合法的命令不算介入（改發布 `COMMAND_UNKNOWN`／`COMMAND_INVALID_PARAMS`）。
- 全體 topic 的 `MISSION_*`／`REBOOT_FC` 會被拒絕（`COMMAND_REJECTED`），也不算介入。
- LOADED 只會因載入逾時、`MISSION_STOP`、新上傳或落地清除；其他命令不影響已載入的任務。
- 煞停終點作為之後相對 `MOVE_*` 的累加基準；中止與完成後都不自動執行 RTL、LAND 或其他命令。
- Agent 重新啟動後任務遺失並發布 IDLE；QGCS 以狀態不符判定任務中止。

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
| `ARM` | 無 | 解鎖；PX4 未解鎖且尚無 Offboard session 時，先自動進入 Offboard（10 筆心跳 prestream → 模式請求 → VehicleStatus 確認）再送出 ARM |
| `DISARM` | 無 | 上鎖 |
| `TAKEOFF [altitude]` | 高度 (公尺，預設 3.0) | 起飛；PX4 已落地且尚無 Offboard session 時先自動進入 Offboard，並等 PX4 回報已解鎖才起飛（3 秒內未解鎖即取消，發布 `TAKEOFF_CANCELLED`）。高度必須是數值，`TAKEOFF abc` 不執行並發布 `COMMAND_INVALID_PARAMS` |

> 降落後的地面狀態：PX4 已上鎖、仍停在 Offboard 而 Agent 沒有 Offboard session 時
> （RTL、LAND、DISARM 或自動上鎖之後），BasicController 會把 PX4 切到 Hold
> （`DO_SET_MODE` main mode 4 AUTO + sub mode 3 LOITER），避免 PX4 停在缺少心跳的
> Offboard 而預檢失敗。因此降落後可直接送 `ARM` → `TAKEOFF`，不必先送 `MODE_OFFBOARD`。
> PX4 拒絕 ARM 時，Agent 記錄 WARN（含 VehicleCommandAck result）、停止 Offboard session，
> 並發布 `PX4_COMMAND_REJECTED`（`command=ARM`、`px4_result` 為 ack 結果）。

#### 航點任務與飛控命令（僅透過 `/{agent_namespace}/command`）

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MISSION_START <mission_id>` | 十進位整數 1～4294967295（不接受小數點、指數或正負號） | 開始已載入的航點任務；結果以 `mission/status` 回報（RUNNING 或 REJECTED） |
| `MISSION_STOP` | 無 | 中止執行中的任務並停在目前位置（ABORTED／OPERATOR_STOP）；只有已載入時清除為 IDLE |
| `REBOOT_FC [force]` | 選填 `0` 或 `1`（預設 0） | 重新啟動**飛控**（PX4 `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`，param1=1、param2=0）；與 Agent 容器的 shutdown service 不同 |

`REBOOT_FC` 安全條件：

| PX4 狀態 | `force = 0` | `force = 1` |
|----------|-------------|-------------|
| VehicleStatus 新鮮、已上鎖、land detected 新鮮且已落地 | 送出 | 送出 |
| VehicleStatus 新鮮、已解鎖 | 拒絕 | **拒絕** |
| VehicleStatus 新鮮、已上鎖但未確認落地 | 拒絕 | 拒絕 |
| VehicleStatus 從未收到或逾時（狀態不明） | 拒絕，`detail` 提示可用 force | 送出（依賴 PX4 在解鎖時自行拒絕） |

- 任何拒絕都發布 `COMMAND_REJECTED`（`command=REBOOT_FC`），`detail` 說明是哪一條。
- 送出前停止 Offboard session、清除排隊中的 ARM／TAKEOFF 與 timed trajectory，飛行狀態回到 IDLE。
- 送出後 `reboot_cooldown_s`（預設 15 秒）內的 `REBOOT_FC` 一律拒絕。
- 結果以 `PX4_COMMAND_ACCEPTED`／`PX4_COMMAND_REJECTED`／`PX4_COMMAND_TIMEOUT` 回報；
  飛控重新連線後發布 `PX4_RESTART_DETECTED`，Agent 清除任務、編隊與 Offboard 狀態並重建 PX4 local 平移。
- `force=1` 的安全前提（PX4 解鎖時以 DENIED 拒絕重新啟動）必須在實機驗證。PX4 SITL（POSIX 平台）不支援重新啟動，
  `REBOOT_FC` 一律收到 `PX4_COMMAND_REJECTED`（DENIED，`px4_result=2`），無法用來驗證此前提。

#### 參數錯誤與事件

- 參數缺少或不合法（例如 `TAKEOFF abc`、`YAW` 無參數、`MISSION_START 1.5`、`MISSION_START 1e3`、
  `MISSION_START 4294967296`、`REBOOT_FC 2`）：
  不執行並發布 `COMMAND_INVALID_PARAMS`。
- 無法解析的命令字串：發布 `COMMAND_UNKNOWN`（`detail` 保留前 64 個字元）。
- 全體 topic 收到 `MISSION_*` 或 `REBOOT_FC`：忽略並發布 `COMMAND_REJECTED`。

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

# 上傳 uav_1 的航點任務（兩點，高度 10 公尺，第一點停留 3 秒）後開始
ros2 topic pub --once /uav_1_agent/mission/plan taiphoon_agent/msg/WaypointMission \
  "{mission_id: 7, altitude_frame: 1, waypoints: [
    {latitude_deg: 24.00010, longitude_deg: 121.00000, altitude_m: 10.0, dwell_s: 3.0, yaw_deg: .nan},
    {latitude_deg: 24.00010, longitude_deg: 121.00010, altitude_m: 10.0, dwell_s: 0.0, yaw_deg: .nan}]}"
ros2 topic pub --once /uav_1_agent/command std_msgs/msg/String "data: 'MISSION_START 7'"

# 觀察任務狀態、事件與健康狀態
ros2 topic echo /uav_1_agent/mission/status
ros2 topic echo /uav_1_agent/events
ros2 topic echo /uav_1_agent/health
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
| `/{agent_namespace}/mission/plan` | reliable | volatile | 10 |
| `/{agent_namespace}/mission/status` | reliable | transient_local | 1 |
| `/{agent_namespace}/events` | reliable | volatile | 20 |
| `/{agent_namespace}/health` | reliable | volatile | 1 |

> 上述 QoS 與目前 `basic_controller_node`、`ego_swarm_planner`、`obstacle_cloud_simulator` 實作一致。
> `mission/status` 使用 transient_local，晚加入的 GCS 會先收到最後一筆狀態；`events` 刻意使用 volatile，不重播舊事件。

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
