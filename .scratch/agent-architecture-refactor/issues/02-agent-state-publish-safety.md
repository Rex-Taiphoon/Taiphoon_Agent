# 02 — Agent 狀態發布 + 位置安全監控

**What to build:** `BasicControllerNode` 訂閱自家 PX4 `vehicle_odometry` 後，將資料轉譯為 Agent 對外狀態，發布至 `/fmu{N}_agent/state`（10Hz）與 `/fmu{N}_agent/attitude`（10Hz）。同節點內新增位置安全監控：當 PX4 實際位置與目標偏差 ≥2m 時，自動觸發 hover 介入。

**Blocked by:** 01 — 自訂訊息定義與 CMake 更新

**Status:** ready-for-agent

- [ ] `BasicControllerNode` 新增 `rclcpp::Publisher<AgentState>` 成員，發布至 `/{agent_namespace}/state`
- [ ] `BasicControllerNode` 新增 `rclcpp::Publisher<AgentAttitude>` 成員，發布至 `/{agent_namespace}/attitude`
- [ ] `odometry_cb` 中將 PX4 `vehicle_odometry` 轉譯為 AgentState（位置 NED）與 AgentAttitude（姿態 q）
- [ ] flight_mode 欄位對應 BasicController 內部 FlightState 枚舉
- [ ] battery_remaining 預設為 1.0（PX4 odometry 不含電量，待後續擴充）
- [ ] 發布頻率透過 ROS2 參數 `state_publish_rate_hz` 控制，預設 10Hz
- [ ] `control_loop_cb` 中新增偏差檢查：比較 current_position_ned 與 target_position_ned，歐氏距離 > 2m 時觸發 hover
- [ ] 偏差閾值透過 ROS2 參數 `position_deviation_threshold_m` 控制，預設 2.0
- [ ] 單機 Docker 環境測試：`ros2 topic echo /fmu1_agent/state` 確認內容與頻率
- [ ] 單機 Docker 環境測試：`ros2 topic echo /fmu1_agent/attitude` 確認內容與頻率

## Comments
