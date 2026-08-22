# EGO 式時空路徑規劃開發計畫

## 目標與範圍

在不改變 ARM、TAKEOFF、MOVE、MOVE_POINT、YAW、編隊指令、ROS2 topic／message、
BasicController 與既有安全流程的前提下，只強化 `OptimizedEgoPlanner` 的路徑初始化。

水平移動仍使用 `FIXED_Z`；`FREE_3D`、`LINEAR_Z`、垂直 MOVE 與垂直編隊沿用現有
planner。不得修改 FormationEpoch、slot assignment、arrival、terminal hold、priority、
reservation 或 emergency brake 規則，也不得複製 EGO-Planner GPL 原始碼。

## 目標資料流

```text
Final Target
→ 現有 Local Goal（10 m horizon）
→ 直線 B-spline seed
→ 衝突區段偵測
→ FIXED_Z 時空 A* 路徑引導
→ 依搜尋路徑建立 B-spline seed
→ 現有 ProjectedGradientSolver
→ 現有 reservation（需要時）
→ 現有 ContinuousSafetyValidator
→ 發布或沿用既有 HOLD／Brake
```

## 核心實作

新增 pure C++ `SpatiotemporalRouteInitializer`，由 `OptimizedEgoPlanner` 內部呼叫。
其 interface 接受 `PlanningSnapshot`、`PlannerConfig` 與直線 seed，回傳固定高度的
timed route points、建議 trajectory start time、成功／失敗原因、搜尋節點數與耗時。

規則：

- 僅在 `enable_local_goal_planning=true` 且 `PathConstraint::FIXED_Z` 時啟用。
- 直線 seed 沒有障礙或 peer 衝突時直接沿用。
- 有衝突時，在 Local Goal window 執行 bounded `(North, East, time)` A*。
- 空間解析度 `0.5 m`；搜尋範圍為起終點 bounding box 加上現有
  `optimizer_search_radius_m`。
- 時間解析度沿用 `sample_interval_s`；邊的時間依現有動力學 duration 推導的平均速度計算。
- 使用八方向移動，每條邊都按時間取樣；起點延遲候選為 `0` 到
  `reservation_horizon_s`，間隔為 `reservation_step_s`；不支援途中停等。
- 障礙使用 `IObstacleMap::is_position_safe()` 與 `obstacle_clearance_m` 判斷。
- peer 使用現有 B-spline 依絕對時間預測，中心距離不得低於
  `peer_collision_min_distance_m`；沿用 `TrajectoryPriority`，但 TERMINAL_HOLD 永遠佔位。
- A* 依最早抵達、路徑長度、固定鄰居順序排序，確保結果可重現。
- A* 與 optimizer 共用現有 `optimizer_time_budget_ms`；超時回傳 `TIMEOUT`，不使用不安全的直線 fallback。

## B-spline 整合與失效處理

- 將 timed route 依時間等距取樣成現有 control point 數量，固定首尾點與全部 Z。
- knot interval 依 route travel time 設定；延長時間後必須重新檢查 peer 時間衝突。
- ProjectedGradientSolver 的 projection envelope 改以 A* seed 為中心。
- 使用既有 `PlannerResult::trajectory_start_time_s` 保存起點延遲；Node 以此時間驗證與發布，
  reservation 只能再延後。
- A*、optimizer 或 validator 失敗時，沿用現有 active trajectory、HOLD 或 EmergencyBrake 行為。
- `enable_local_goal_planning` 同時作為新流程回退開關；目前預設為 `true`，
  `enable_generic_trajectory_reservation` 維持原設定。

## 測試與驗收

Pure C++ 測試涵蓋：直線 fast path、固定障礙繞行、錯位 TERMINAL_HOLD 中間通道、
關閉通道、moving peer 的起點延遲／繞行、priority、鏡像結果、stale／NaN／Inf／timeout、
FREE_3D／LINEAR_Z 回歸，以及最終動力學與 safety validator。

Docker 測試順序：

```bash
docker compose up -d fmu1_agent
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon build"
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test --packages-select taiphoon_agent"
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"
```

新增 `AutoTest/scripts/ego_route_planning_sitl_validation.sh`，涵蓋：

- 單機：既有 OFFBOARD／ARM／TAKEOFF／MOVE／YAW 回歸、無障礙直線、固定牆繞行、封閉區域 HOLD／Brake。
- 雙機：相距 8 m 對向交換，驗證 priority、繞行／起點延遲、最小中心距離與無持續 timeout。
- 四機：以 `FORMATION_CIRCLE 10 10` 驗證完整環形編隊固定高度，以及既有
  `FORMATION_CIRCLE → FORMATION_CANCEL → MOVE` 回歸；錯位 TERMINAL_HOLD 的中間通道由
  pure C++ 路徑測試驗證。

驗收要求：水平 trajectory Z span ≤ `0.05 m`、障礙 clearance 與 peer 最小距離符合現有參數、
命令格式與對外 topic 不變；單機／雙機／四機 SITL 均由同一腳本提供執行入口。
單機／雙機 SITL 路徑案例在發送既有 `TAKEOFF 8.0` 前先啟動高度監測器取得地面基準；
四機 Formation 案例則先以既有 `TAKEOFF 10.0` 起飛，再發送
`FORMATION_CIRCLE 10 10`。兩者都以
`AgentState.position_global` 確認相對起飛基準上升至少 `7.5 m`（單機／雙機）或
`9.5 m`（四機 Formation），允許 0.5 m SITL 誤差且穩定 1 秒，避免
起飛尚未完成造成高度異常被誤判為路徑規劃問題。

## 文件同步

只更新 README 與 `docs/taiphoon-agent-interface.md` 中的 Local Goal 預設值、新路徑行為與 SITL
指令；不得修改既有 development spec、舊架構分析文件或新增 ROS2 對外介面。
