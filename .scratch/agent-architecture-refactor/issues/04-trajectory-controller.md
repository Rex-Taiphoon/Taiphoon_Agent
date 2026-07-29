# 04 — 軌跡控制層新增（TrajectoryController）

**What to build:** 新增 `TrajectoryController` 節點，接收目標 NED 位置與所有同儕當前位置，規劃無碰撞路徑（任兩機 ≥3m），輸出帶時間戳的 waypoint 序列。預留外部避障節點訂閱介面。

**Blocked by:** 01 — 自訂訊息定義與 CMake 更新、02 — Agent 狀態發布 + 位置安全監控

**Status:** ready-for-agent

- [ ] 定義 `Waypoint` 結構：NED 位置 (x,y,z) + 到達時間戳
- [ ] `TrajectoryController` 訂閱自身目標 NED（`/{agent_namespace}/ned_target`，來自 MissionPlanner）
- [ ] `TrajectoryController` 訂閱所有同儕 `AgentState`（可複用 MissionPlanner 的同儕發現邏輯，或共用同儕清單）
- [ ] 路徑規劃：從目前位置到目標 NED，生成直線 waypoint 序列，檢查每個 waypoint 與所有同儕位置距離
- [ ] 安全邊界：若有同儕在 3m 內，延遲或繞行該 waypoint（初期可用時間偏移策略：延遲抵達時間直到同儕離開安全範圍）
- [ ] 安全邊界參數化：`safety_margin_m` 預設 3.0
- [ ] 輸出 waypoint 序列至 `/{agent_namespace}/waypoints`，訊息格式為 `TrajectoryWaypoint` 陣列（需定義新訊息或複用 NedTarget 序列）
- [ ] 預留障礙物訂閱介面：`/{agent_namespace}/obstacles` topic，訂閱但不實作處理邏輯（未來外部節點接入）
- [ ] 更新 `CMakeLists.txt`：新增 trajectory_controller 執行檔
- [ ] 測試：注入目標 NED + 兩組同儕位置（一組在路徑上） → 確認 waypoint 序列中所有點與同儕位置 ≥3m
- [ ] 測試紀錄留存於 `AutoTest/`

## Comments
