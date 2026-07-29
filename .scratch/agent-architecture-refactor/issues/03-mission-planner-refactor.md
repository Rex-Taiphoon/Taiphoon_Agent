# 03 — 任務規劃層重構（FormationNode → MissionPlanner + 同儕發現）

**What to build:** 將既有 `FormationNode` 重構為 `MissionPlanner`，移除對 PX4 `vehicle_odometry` 的直接訂閱，改為透過同儕發現機制動態訂閱其他 Agent 的 `/fmu*_agent/state`。接收 `/fmu_all_agent/command` 編隊命令後，根據線上 Agent 位置計算自身目標 NED。

**Blocked by:** 01 — 自訂訊息定義與 CMake 更新、02 — Agent 狀態發布 + 位置安全監控

**Status:** ready-for-agent

- [ ] 同儕發現：`MissionPlanner` 建構時呼叫 `get_topic_names_and_types()`，匹配 `/fmu*_agent/state` 正則，排除自身 namespace
- [ ] 對每個發現的同儕，動態建立 `rclcpp::Subscription<AgentState>`，存入 map
- [ ] 以 3 秒週期定時器重新掃描 topic，處理新機加入與離線
- [ ] 從 `AgentState` 提取各機位置快取，取代既有 `DroneState` + `drone_states_` 結構
- [ ] 既有 `compute_centroid()` 與 `compute_formation_target()` 邏輯移植至 `MissionPlanner`
- [ ] `/fmu_all_agent/command` 訂閱維持不變
- [ ] 編隊目標發布至 `/{agent_namespace}/ned_target` 維持不變
- [ ] **移除**對 `/fmu{N}/fmu/out/vehicle_odometry` 的所有訂閱
- [ ] **移除** `drone_count` 靜態參數
- [ ] **移除** `FormationNode` 相關檔案（formation_node.cpp/hpp, main_formation.cpp）
- [ ] 更新 `CMakeLists.txt`：移除 formation_node 執行檔，新增 mission_planner 執行檔
- [ ] 測試：`ros2 topic pub` 模擬兩台同儕 AgentState → `ros2 topic pub /fmu_all_agent/command` 發送 FORMATION_CIRCLE → 驗證 `/fmu1_agent/ned_target` 與手算一致
- [ ] 測試紀錄留存於 `AutoTest/`

## Comments
