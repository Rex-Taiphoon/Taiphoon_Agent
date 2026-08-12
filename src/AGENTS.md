# AGENTS.md — src（原始碼總覽）

## 目的

`src/` 是 Taiphoon_Agent 的核心原始碼目錄，依模組資料夾組織。每個子資料夾都有自己的 AGENTS.md，本檔只說明整體結構與跨模組慣例。

## 模組地圖

| 資料夾 | 職責 |
|--------|------|
| `core/` | 介面契約層（純虛擬基底類別） |
| `controllers/` | 實作 core 介面，轉譯 PX4 飛行控制 |
| `nodes/` | ROS2 節點組裝層（topic、參數、控制迴圈） |
| `ego_swarm/` | 現行機群局部規劃核心：共同座標、B-spline、peer future trajectory、編隊、planner backend、trajectory-level safety。現有 deterministic candidate planner 保留作 fallback / regression，允許後續加入 optimized EGO backend |
| `perception/` | 障礙物幾何採樣與 PointCloud2 Adapter（純邏輯） |
| `safety/` | 安全監控與狀態轉譯（純邏輯） |
| `formation/` | 舊 MissionPlanner 與編隊純邏輯元件；不再提供現行 runtime Node |
| `trajectory/` | 舊 TrajectoryController 與 waypoint 純邏輯元件；不再提供現行 runtime Node |
| `utils/` | 命令解析與數值驗證工具 |

現行 ROS2 runtime executable 由 CMake／Launch 啟動：

- `basic_controller`：PX4 控制與唯一 `TrajectorySetpoint` 輸出端。
- `agent_supervisor`：Agent shutdown service。
- `ego_swarm_planner`：EGO-Swarm 編隊、局部候選軌跡與同儕避碰。
- `obstacle_cloud_simulator`：選配 YAML PointCloud2 障礙物模擬。

`MissionPlanner`／`TrajectoryController` 及其進入點不再由 CMake 或 Launch 啟動；
舊檔案僅保留純邏輯測試與歷史相容用途，不得視為現行控制資料流。

## 跨模組慣例

- include 一律以 `src/` 為根：`#include "core/IBasicFlightController.hpp"`、`#include "utils/CommandTypes.hpp"`。
- 命名空間對應：節點類別統一放在 `taiphoon::nodes`（即使檔案位於 `formation/`、`trajectory/`），純邏輯類別使用模組命名空間（`taiphoon::formation`、`taiphoon::trajectory`、`taiphoon::safety`、`taiphoon::utils`）。
- 依賴方向：`core` ← `controllers` ← `nodes`；`utils` 與純邏輯類別不依賴 ROS2；節點類別不包含演算法。
- `src/main.cpp` 是 `BasicControllerNode` 進入點；`src/main_ego_swarm_planner.cpp` 與
  `src/main_obstacle_cloud_simulator.cpp` 是現行 EGO／PointCloud Node 進入點。
- `formation/main_mission_planner.cpp` 與 `trajectory/main_trajectory_controller.cpp`
  是舊進入點，未列入現行 CMake／Launch runtime。
- 現行 trajectory planning 與 formation runtime 皆以 `src/ego_swarm/` 為主要 domain；legacy `src/trajectory/`、`src/formation/` 不重新接回 runtime。
- obstacle map / sensor-independent obstacle representation 放 `src/perception/`。
- 單元測試統一放在專案根目錄 `test/` 並使用 gtest（`ament_add_gtest`）；`src/` 內禁止放置測試原始碼。
- 修改任何模組前，先閱讀該模組的 AGENTS.md 與 `CODE_CONSTRAINT.md`。
