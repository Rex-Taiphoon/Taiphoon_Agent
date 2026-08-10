# AGENTS.md — src/formation（任務規劃與編隊）

## 目的

`src/formation/` 是舊任務規劃與編隊純邏輯模組。現行 runtime 已由
`ego_swarm_planner` 取代；`MissionPlanner` 不再由 CMake 或 Launch 啟動，舊檔案僅
保留純邏輯測試與歷史相容用途。

## 內容

| 檔案 | 角色 |
|------|------|
| `MissionPlanner.hpp/.cpp` | 舊 ROS2 節點實作；不在現行 CMake／Launch runtime，僅保留測試與歷史參考 |
| `FormationCalculator.hpp` | 純邏輯（`taiphoon::formation`）：質心計算與編隊目標位置（CIRCLE / HORI_LINEAR / VERT_LINEAR） |
| `PeerMatcher.hpp` | 純邏輯（`taiphoon::formation`）：從 node / topic 名稱解析 Agent 索引與過濾同儕；可供現行 EGO planner 共用 |
| `main_mission_planner.cpp` | 舊 `MissionPlanner` 進入點；不在現行 CMake／Launch runtime |

## 核心原則

- 現行節點與演算法分離：`ego_swarm_planner` 位於 `src/nodes/`，EGO 核心位於
  `src/ego_swarm/`；本資料夾的 `MissionPlanner` 不得重新接回 runtime。
- 舊編隊數學與同儕解析仍可作純邏輯測試；若由 EGO planner 共用 `PeerMatcher`，不得
  將 `MissionPlanner` 的 ROS2 控制責任一併帶回。
- 現行編隊槽位由 `ego_swarm::FormationSlotPlanner` 負責；修改 EGO 編隊行為時應同步
  更新 `docs/taiphoon-agent-interface.md` 與對應測試。
- 新增編隊類型時需同步：`FormationType` 列舉、`FormationCalculator` 計算、`utils/CommandTypes.hpp` 的 `FORMATION_*` 命令解析、`docs/taiphoon-agent-interface.md` 命令協定。

## 變更與驗證

- 純邏輯變更以 `test/formation_calculator_test.cpp`、`test/peer_matcher_test.cpp` 與
  EGO-Swarm 對應測試驗證；不要以啟動 `mission_planner` 作為 runtime 驗證方式。
