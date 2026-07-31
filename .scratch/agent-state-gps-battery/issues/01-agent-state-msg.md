# 01 — AgentState 擴充 + StatePublisher 轉譯 + 單元測試

**What to build:** `AgentState.msg` 新增 `position_global[3]`（經緯度海拔）、`battery_voltage_v`、`battery_current_a` 三個欄位。`StatePublisher::fill_agent_state` 擴充簽名，能從 PX4 `VehicleGlobalPosition` 與 `BatteryStatus` 正確轉譯至 AgentState，nullptr 時填入 NaN/1.0（保守預設）。`gtest` 6 案例全綠。

**Blocked by:** None — can start immediately.

**Status:** ready-for-agent

- [ ] `AgentState.msg` 新增 `position_global`、`battery_voltage_v`、`battery_current_a` 欄位
- [ ] `fill_agent_state` 簽名新增 `const VehicleGlobalPosition*`、`const BatteryStatus*`
- [ ] GPS 有效時 `position_global` 正確複製 lat/lon/alt
- [ ] Battery 有效時 `battery_remaining` 真實取值，`voltage_v`/`current_a` 正確複製
- [ ] GPS null → `position_global` 填 `NaN`
- [ ] Battery null → `battery_remaining=1.0`，`voltage_v`/`current_a` 填 `NaN`
- [ ] 既有欄位（NED/flight_mode/online）行為不受影響
- [ ] 6 個新 gtest + 所有舊測試通過
