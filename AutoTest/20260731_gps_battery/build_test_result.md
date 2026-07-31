# AgentState GPS/Battery 擴充 — 建置與單元測試結果

**日期**：2026-07-31  
**分支**：`codex/agent-state-gps-battery`（若有切分支）

## 建置

- Docker 映像：`taiphoon-agent:dev`
- 編譯指令：`colcon build --symlink-install`
- 結果：`Finished <<< taiphoon_agent`，無錯誤、無警告

## 單元測試

| 測試套件 | 案例數 | 結果 |
|----------|--------|------|
| `test_state_publisher` | 14 | ✅ 14/14 Passed |
| `test_position_safety_monitor` | — | ✅ Passed |
| `test_formation_calculator` | — | ✅ Passed |
| `test_peer_matcher` | — | ✅ Passed |
| `test_trajectory_planner` | 22 | ✅ 22/22 Passed |

**StatePublisher 新增測試案例：**
- `FillAgentState_GpsFromGlobalPosition` — GPS → position_global 正確複製
- `FillAgentState_GpsNullptr` — GPS nullptr → NaN
- `FillAgentState_BatteryFromBatteryStatus` — Battery → voltage/current/remaining 正確複製
- `FillAgentState_BatteryNullptr` — Battery nullptr → 1.0/NaN/NaN
- `FillAgentState_AllPresent` — 所有資料齊全時欄位全對
- `FillAgentState_BackwardCompat` — 舊簽名（4 args）向下相容

## SITL 驗證

- SITL 驗證腳本：`AutoTest/20260731_gps_battery/verify_gps_battery.sh`
- 執行方式：`bash AutoTest/20260731_gps_battery/verify_gps_battery.sh [namespace]`
- 驗證項目：
  - [ ] `position_global` lat 約 25.04、lon 約 121.51
  - [ ] `battery_voltage_v` > 0
  - [ ] `battery_remaining` 非固定 1.0
