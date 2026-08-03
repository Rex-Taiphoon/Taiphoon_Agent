# AGENTS.md — src/trajectory（軌跡控制層）

## 目的

`src/trajectory/` 實作軌跡控制層：接收目標 NED 位置與同儕狀態，規劃無碰撞 waypoint 序列並發布。包含一個 ROS2 節點（`TrajectoryController`）與一個純邏輯規劃器。

## 內容

| 檔案 | 角色 |
|------|------|
| `TrajectoryController.hpp/.cpp` | ROS2 節點（`taiphoon::nodes`）：訂閱 `/{namespace}/ned_target` 與同儕 AgentState，發布 `/{namespace}/waypoints`；預留 `/{namespace}/obstacles` 介面；同儕發現複用 `formation/PeerMatcher` |
| `TrajectoryPlanner.hpp` | 純邏輯（`taiphoon::trajectory`）：直線 waypoint 生成、到達時間計算、與同儕位置的安全距離檢查 |
| `main_trajectory_controller.cpp` | `TrajectoryController` 進入點 |

## 核心原則

- 路徑規劃演算法一律放在 `TrajectoryPlanner`（無 ROS2 依賴、可單元測試），`TrajectoryController` 只負責資料收發與驅動。
- 安全邊界（`safety_margin_m`，預設 ≥ 3 公尺）是兩機路徑間的最小允許距離；修改參數或演算法時必須維持此保證並更新測試。
- 同儕位置資料可能過期或離線：規劃前需過濾離線 / 無效資料，禁止把 NaN 或不完整資料送入規劃器。
- 新增輸出型別或 topic 時，同步更新 `msg/`、`docs/taiphoon-agent-interface.md` 與測試。

## 變更與驗證

- 修改後執行 `colcon build --packages-select taiphoon_agent`，並執行 `colcon test --packages-select taiphoon_agent` 確認單元測試（對應測試：`test/trajectory_planner_test.cpp`，gtest）。
