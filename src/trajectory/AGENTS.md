# AGENTS.md — src/trajectory（軌跡控制層）

## 目的

`src/trajectory/` 是舊直線 waypoint 規劃模組。現行 runtime 已由
`ego_swarm_planner` 的 deterministic candidate baseline 取代；`TrajectoryController`
不再由 CMake 或 Launch 啟動，舊檔案僅保留純邏輯測試與歷史相容用途。

## 內容

| 檔案 | 角色 |
|------|------|
| `TrajectoryController.hpp/.cpp` | 舊 ROS2 節點實作；不在現行 CMake／Launch runtime，僅保留測試與歷史參考 |
| `TrajectoryPlanner.hpp` | 純邏輯（`taiphoon::trajectory`）：直線 waypoint 生成、到達時間計算、與同儕位置的安全距離檢查 |
| `main_trajectory_controller.cpp` | 舊 `TrajectoryController` 進入點；不在現行 CMake／Launch runtime |

## 核心原則

- `TrajectoryPlanner` 僅維持無 ROS2 依賴的純邏輯測試；現行局部軌跡規劃應修改
  `src/ego_swarm/` 的 `EgoSwarmPlanner`，並保持 deterministic candidate baseline 定義。
- 現行 EGO 距離參數是機體中心到中心，使用 `formation_min_distance_m` 與
  `peer_collision_min_distance_m`；不把舊 `safety_margin_m` 視為現行 runtime 契約。
- 同儕軌跡資料處理與有限值檢查由現行 EGO planner 負責；禁止讓舊
  `TrajectoryController` 重新成為 PX4 控制資料流。
- 新增輸出型別或 Topic 時，同步更新 `msg/`、`docs/taiphoon-agent-interface.md` 與測試。

## 變更與驗證

- 純邏輯變更以 `test/trajectory_planner_test.cpp` 與 EGO-Swarm 對應測試驗證；不要
  以啟動 `trajectory_controller` 作為 runtime 驗證方式。現行 runtime 應驗證
  `ego_swarm_planner`、`basic_controller` 與選配 `obstacle_cloud_simulator`。
