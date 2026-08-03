# AGENTS.md — src/safety（安全監控與狀態轉譯）

## 目的

`src/safety/` 提供兩類純邏輯元件：飛行安全監控（位置偏差）與 Agent 對外狀態轉譯（PX4 資料 → AgentState / AgentAttitude）。設計目標是「可獨立單元測試」，由節點在控制迴圈中呼叫。

## 內容

| 檔案 | 角色 |
|------|------|
| `PositionSafetyMonitor.hpp` | 純邏輯：NED 位置偏差（歐氏距離）與閾值比較，不依賴 ROS2 |
| `StatePublisher.hpp` | 純邏輯靜態方法：把 PX4 odometry / GPS / battery 轉譯為 AgentState / AgentAttitude，不持有 publisher |

## 核心原則

- 本模組元件不得依賴 `rclcpp` 或持有 publisher / subscriber；資料轉譯必須對缺漏資料（GPS 或電池為空）給出明確結果（NaN 或預設值）。
- 安全相關邏輯（閾值、偏差判斷）必須保持純函式，以單元測試驗證；加入 ROS2 依賴前先確認是否有必要。
- 飛行模式列舉（`FlightState`，定義於 `controllers/BasicController.hpp`）與 `AgentState.flight_mode` 的對應由本模組負責，狀態機變更時需同步。
- 新增對外狀態欄位時，必須同步更新 `msg/` 訊息定義、`docs/taiphoon-agent-interface.md` 與對應測試。

## 變更與驗證

- 修改後執行 `colcon build --packages-select taiphoon_agent`，並執行 `colcon test --packages-select taiphoon_agent` 確認單元測試（對應測試：`test/position_safety_monitor_test.cpp`、`test/test_state_publisher.cpp`，gtest）。
