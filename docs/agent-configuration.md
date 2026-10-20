# Agent ROS2 參數參考

啟動工具的基本設定與跨平台操作請見根目錄 README。以下只列 ROS2 節點參數。

## 參數設定

### BasicController 參數

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `target_altitude_m` | double | 3.0 | 預設起飛高度（公尺） |
| `hover_duration_s` | double | 3.0 | 預設懸停時間（秒） |
| `startup_delay_s` | double | 0.0 | 啟動後延遲自動 Offboard 的秒數（多機依 `(i-1) * 2.0` 錯開） |
| `auto_start_sequence` | bool | true | 連線後是否自動切換至 Offboard 模式 |
| `target_system` | int | 1 | PX4 MAVLink 系統 ID |
| `agent_namespace` | string | uav_1_agent | Agent 命令命名空間 |
| `state_publish_rate_hz` | double | 10.0 | AgentState／AgentAttitude 名目發布頻率（Hz）；非法值或低於 10 時固定使用 10 Hz |
| `yaw_rate_limit_deg_s` | double | 30.0 | YAW／MOVE_POINT 偏航角速率上限（度/秒）；每個 10 Hz 控制週期沿最短角度推進並填入 `yawspeed` 前饋。合法範圍 1～180，NaN／非正值改用 30 並 WARN，超出範圍夾到邊界。PX4 Offboard 不套用 `MPC_YAWRAUTO_MAX`，此值應小於等於 PX4 `MC_YAWRATE_MAX`；launch 參數同名，容器部署以環境變數 `BASIC_CONTROLLER_YAW_RATE_LIMIT_DEG_S` 設定 |
| `collision_safety_margin_m` | double | 3.0 | 舊觀測式防撞參數；EGO-Swarm 取代後不作為新的規劃契約 |
| `collision_peer_timeout_s` | double | 2.0 | 舊觀測式防撞參數；EGO-Swarm 取代後不作為新的規劃契約 |

### EGO-Swarm Planner 實際參數

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `agent_namespace` | string | `uav_1_agent` | Agent 對外命名空間 |
| `agent_index` | int | `1` | Agent 索引；小於等於 0 時由 Node 推算 |
| `swarm_origin_agent_index` | int | `1` | 保留的相容參數；WGS84 軌跡交換時各 Agent 以自己的固定 Home 建立 local NED |
| `peer_trajectory_timeout_s` | double | `2.0` | 同儕未來軌跡 freshness timeout；逾時後不再交給 planner 使用 |
| `max_peer_time_skew_s` | double | `1.0` | 同儕軌跡 message／start time 與本機 receive time 允許的最大時間偏差 |
| `peer_clock_skew_warning_s` | double | `0.1` | 同儕主機時鐘差估計超過此值時輸出 WARN；只影響診斷，不改變規劃（見 [多機時間同步](time-sync.md)） |
| `obstacle_cloud_timeout_s` | double | `1.0` | 局部障礙物地圖 freshness timeout；逾時後 planner 進入 HOLD |
| `require_obstacle_cloud` | bool | `true` | 是否要求 fresh obstacle map；missing/stale 不會被當成 explicit empty |
| `planning_rate_hz` | double | `10.0` | 規劃迴圈頻率 |
| `enable_local_goal_planning` | bool | `true` | Local Goal segmented planning；水平 FIXED_Z 受阻時啟用 bounded 時空 A*，FREE_3D／LINEAR_Z 維持既有 planner |
| `enable_generic_trajectory_reservation` | bool | `false` | 將 reservation 擴展至 MOVE 等所有 moving trajectory；Formation 仍固定使用 reservation |
| `peer_planning_margin_m` | m | `1.0` | planner 產生路徑時額外保留的機間距離；最終安全門檻仍由 `peer_collision_min_distance_m` 驗證 |
| `local_goal_horizon_m` | double | `10.0` | Local Goal 單一 planning segment 的距離 horizon（公尺） |
| `local_goal_detour_margin_m` | double | `3.0` | Local Goal 側向繞行相對於 peer collision distance 額外保留的追蹤裕度（公尺） |
| `local_goal_replan_ratio` | double | `0.70` | Local Goal segment progress 達此比例後，允許嘗試下一段規劃 |
| `minimum_replan_interval_s` | double | `0.5` | 正常 progress-based replan 的最短間隔（秒）；不阻擋 safety、Final Target 變更與 segment 結束 |
| `formation_min_distance_m` | double | `2.0` | 編隊槽位的機體中心到中心最小距離；可調為 1.5、2 或 3 m |
| `peer_collision_min_distance_m` | double | `2.0` | 同儕未來軌跡的機體中心到中心最小距離；不因資料年齡擴張 |
| `obstacle_clearance_m` | double | `1.0` | EGO-Swarm 與局部障礙物的安全間距 |
| `knot_interval_s` | double | `1.0` | B-spline 最小基準 knot 間隔；實際 duration 由 EGO-Swarm 依候選路徑長度與限制計算 |
| `max_velocity_m_s` | double | `2.5` | EGO-Swarm 計算軌跡 duration 使用的最大速度（公尺/秒） |
| `max_acceleration_m_s2` | double | `1.5` | EGO-Swarm 計算軌跡 duration 使用的最大加速度（公尺/秒^2） |
| `max_velocity_mps` | double | `2.5` | safety validator／煞車使用的最大速度（公尺/秒）；與舊名稱同步時以此名稱為 canonical |
| `max_acceleration_mps2` | double | `1.5` | safety validator／煞車使用的最大加速度（公尺/秒^2） |
| `max_jerk_mps3` | double | `3.0` | safety validator／煞車使用的最大 jerk（公尺/秒^3） |
| `candidate_offset_m` | double | `3.0` | 局部候選軌跡側向／垂直偏移 |
| `local_arrival_tolerance_m` | double | `0.5` | PX4 local NED 到達容許誤差（公尺） |
| `global_arrival_tolerance_m` | double | `1.0` | 相容參數名稱；GPS 轉換至本機 `planner_ned` 的到達容許誤差（公尺） |
| `move_attitude_timeout_s` | double | `1.0` | 水平 MOVE 可使用的 AgentAttitude freshness window |
| `formation_slot_switch_penalty_m` | double | `2.0` | minimum-cost assignment 換槽位成本 |
| `formation_member_stable_scan_count` | int | `2` | discovery 成員集合連續穩定掃描次數 |
| `planner_backend` | string | `optimized` | optimized-only runtime；其他值使 planner 初始化失敗 |
| `optimizer_max_iterations` | int | `40` | optimized projected-gradient iteration 上限 |
| `optimizer_time_budget_ms` | double | `50.0` | optimized backend 單次 wall-time 上限（毫秒）；保留 finite-difference 與 SITL callback 負載餘裕 |
| `formation_crossing_penalty_m` | double | `10.0` | assignment 水平 crossing 等價成本 |
| `reservation_lead_time_s` | double | `0.5` | future trajectory reservation 最短提前時間 |
| `reservation_step_s` | double | `0.5` | reservation start time 搜尋步進 |
| `reservation_horizon_s` | double | `5.0` | reservation start time 最遠搜尋範圍 |

目前 runtime 只使用內建 pure C++ continuous control-point projected-gradient backend，
不引入外部 optimizer dependency。`planner_backend` 只接受 `optimized`；deterministic
類別保留作為既有純邏輯相容檔案，但不再由 runtime 組裝或作 fallback。Optimized backend
只使用單一直線 B-spline seed，起終點為硬約束，中間控制點的水平搜尋範圍為內部 10 m；
`FIXED_Z` 與 `LINEAR_Z` 路徑會分別固定高度或只允許起終點之間的線性高度變化。
objective 包含 terminal goal、smoothness、動力學超限、障礙物 clearance barrier 與
peer time-aligned separation barrier，並保留 obstacle 1 m、peer 2 m 的 hard safety
門檻。動力學 gate 預設上限為 velocity 2.5 m/s、acceleration 1.5 m/s²、jerk 3.0 m/s³；
optimizer wall-time budget 為 50 ms。Optimizer 失敗時不使用 deterministic fallback；靜止／低速 Agent 立即建立固定
位置 HOLD 並週期性重試，移動中的 Agent 使用 EmergencyBrakePlanner 後再 HOLD。
EGO-Swarm 會依候選路徑長度、最大速度、最大加速度與最大 jerk 計算每條 B-spline
的 duration；若 analytic p/v/a/j 取樣仍超限，會逐步延長候選時間後才交給
ContinuousSafetyValidator。
同一條 active trajectory 在執行期間保留原始 `start_time` 與 `trajectory_id`，不會每個
planning cycle 重置進度。Planner 以自己的固定 Home 將 WGS84 與 PX4 local NED
轉換到同一個公尺制 local frame；兩者都在容許誤差內才視為到達。到達後持續發布
`terminal_hold=true` 的目標位置 hold trajectory，清除任務但保留 peer occupancy
heartbeat；沒有移動或編隊目標時，BasicController 維持原本鎖定的 finite hold setpoint，
不會被 global occupancy heartbeat 改寫。
同儕 `AgentTrajectory` 另受 `peer_trajectory_timeout_s`、`max_peer_time_skew_s`、
控制點／duration 合理性與 `frame_revision` 約束；ACTIVE 與 TERMINAL_HOLD 資料都
參與 peer collision 評估。ContinuousSafetyValidator 會再檢查整條候選軌跡的 p/v/a/j、
peer/map freshness、同儕 separation 與 obstacle clearance；移動軌跡逾時或資料失效時
切換至 EmergencyBrakePlanner，完成後由 sender 發布終點 terminal hold。
編隊 terminal hold 會清除舊 hold 後再發布單調遞增的 trajectory ID，避免舊 heartbeat
覆蓋新軌跡；EmergencyBrakePlanner 進行中的減速軌跡沿原時間軸發布，不會每個 cycle
重新煞車。

### ObstacleCloudSimulator 實際參數

| 參數 | 類型 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | string | `uav_1_agent` | Agent 對外命名空間 |
| `config_file` | string | 空字串 | YAML 場景檔；Launch 預設為 `config/obstacles/default.yaml` |
| `publish_rate_hz` | double | `15.0` | PointCloud2 發布頻率 |
| `agent_index` | int | `1` | Agent 索引；供 synthetic sensor 訂閱自身 pose |
| `swarm_origin_agent_index` | int | `1` | synthetic sensor 既有共享 `swarm_ned` 障礙物場景的原點 Agent；仍實際生效 |
| `mode` | string | `explicit_empty` | ROS2 parameter 覆蓋 YAML `mode`；預設發布 fresh zero-point cloud |
| `range_m` | double | `20.0` | 感測範圍 |
| `horizontal_fov_deg` | double | `270.0` | 水平視野 |
| `vertical_fov_deg` | double | `90.0` | 垂直視野 |
| `noise_std_m` | double | `0.02` | 感測位置雜訊標準差 |
| `dropout_ratio` | double | `0.05` | 點雲 dropout 比例 |
| `sample_spacing_m` | double | `0.5` | 障礙物表面採樣間距 |

Planner 只接受 `IObstacleMap` 的 fresh map。未收到 cloud 是 `missing`，收到但超過
`obstacle_cloud_timeout_s` 是 `stale`；只有 `explicit_empty` source 產生的 zero-point
map 才是 `fresh + valid + empty`。因此啟動 obstacle simulator 時，無障礙場景應明確使用
`obstacle_mode:=explicit_empty`，不能以停發 PointCloud2 代表空環境。

範例：停用自動 Offboard 並設定起飛高度 5m：

```bash
docker compose exec uav_1_agent bash -c "
  source /opt/ros/humble/setup.bash
  source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
  source /home/taiphoon/Taiphoon_Agent/install/setup.bash
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=1 \
      auto_start_sequence:=false target_altitude_m:=5.0
"
```

---
