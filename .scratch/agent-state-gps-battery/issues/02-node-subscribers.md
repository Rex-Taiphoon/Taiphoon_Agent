# 02 — BasicControllerNode 訂閱 GPS/Battery + 上線

**What to build:** `BasicControllerNode` 新增 `VehicleGlobalPosition` 與 `BatteryStatus` 兩個 PX4 topic 訂閱，`state_publish_cb` 開始發布真實 GPS 座標與電池電壓電流。`ros2 topic echo /fmu1_agent/state` 即可看到新欄位有真實資料。

**Blocked by:** 01 — AgentState 擴充 + StatePublisher 轉譯 + 單元測試

**Status:** ready-for-agent

- [ ] 訂閱 `fmu/out/vehicle_global_position`（QoS 比照 odometry）
- [ ] 訂閱 `fmu/out/battery_status`（QoS 比照 odometry）
- [ ] Subscriber 為類別成員（L1-12），回呼檢查 nullptr（L1-03）
- [ ] `state_publish_cb` 將快取 GPS/Battery 傳入 `fill_agent_state`
- [ ] 編譯無警告（含 float64→float32 narrowing cast）
