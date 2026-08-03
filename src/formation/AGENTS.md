# AGENTS.md — src/formation（任務規劃與編隊）

## 目的

`src/formation/` 實作任務規劃層：同儕發現（Peer Discovery）、編隊計算與編隊目標發布。包含一個 ROS2 節點（`MissionPlanner`）與多個純邏輯元件。

## 內容

| 檔案 | 角色 |
|------|------|
| `MissionPlanner.hpp/.cpp` | ROS2 節點（命名空間 `taiphoon::nodes`）：動態掃描同儕 Agent、訂閱 `/fmu_all_agent/command`、計算並發布 `/{namespace}/ned_target` |
| `FormationCalculator.hpp` | 純邏輯（`taiphoon::formation`）：質心計算與編隊目標位置（CIRCLE / HORI_LINEAR / VERT_LINEAR） |
| `PeerMatcher.hpp` | 純邏輯（`taiphoon::formation`）：從 node / topic 名稱解析 agent 索引與過濾同儕 |
| `main_mission_planner.cpp` | `MissionPlanner` 進入點 |

## 核心原則

- 節點與純邏輯分離：`MissionPlanner` 屬節點（`taiphoon::nodes`），編隊數學與同儕比對屬純邏輯（`taiphoon::formation`）；新增演算法時放純邏輯類別並寫單元測試。
- 職責劃分：同儕發現解析在 `PeerMatcher`，狀態追蹤在 `MissionPlanner`（`peers_` map），編隊計算在 `FormationCalculator`；不要混在一起。
- `effective_self_index` 與「任意 Agent 離線重編隊」是既有設計：編隊以**線上** Agent 動態計算，不假設編號連續。
- 新增編隊類型時需同步：`FormationType` 列舉、`FormationCalculator` 計算、`utils/CommandTypes.hpp` 的 `FORMATION_*` 命令解析、`docs/taiphoon-agent-interface.md` 命令協定。

## 變更與驗證

- 修改後執行 `colcon build --packages-select taiphoon_agent`，並執行 `colcon test --packages-select taiphoon_agent` 確認單元測試（對應測試：`test/formation_calculator_test.cpp`、`test/peer_matcher_test.cpp`，gtest）。
