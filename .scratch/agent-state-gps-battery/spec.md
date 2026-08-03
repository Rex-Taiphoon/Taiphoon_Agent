# AgentState 擴充：GPS 座標與電池狀態

Status: ready-for-agent

## Problem Statement

GCS 開發團隊需要透過 ROS2 topic 取得無人機的完整狀態資訊以建立地面監控儀表板。目前 `/fmu{N}_agent/state` 僅包含 NED 位置、飛行模式、電量比例與在線旗標，**缺少兩項關鍵資料**：

1. **GPS 座標**（經度、緯度、海拔）—— GCS 無法在地圖上定位無人機，也無法與 MAVLink 遙測或 PX4 地面站交叉比對位置。
2. **電池電壓與電流**—— 現有 `battery_remaining` 欄位始終回傳 `1.0`（硬編碼預設值），GCS 無法取得真實電量、低電壓警告或估算剩餘飛行時間。

PX4 已透過 `px4_msgs` 在 ROS2 網路中發布 `VehicleGlobalPosition`（經緯度海拔）與 `BatteryStatus`（電壓電流電量），但 Taiphoon_Agent 未曾訂閱這些 topic 或將其轉譯至 AgentState。

## Solution

擴充 `AgentState` 自訂訊息，新增三個欄位：

- `position_global`（`float32[3]`）：`[lat, lon, alt]`，來自 PX4 `VehicleGlobalPosition`
- `battery_voltage_v`（`float32`）：電池電壓，來自 PX4 `BatteryStatus`
- `battery_current_a`（`float32`）：電池電流，來自 PX4 `BatteryStatus`

`BasicControllerNode` 新增兩個 PX4 topic 訂閱，`StatePublisher` 擴充轉譯邏輯，既有 `battery_remaining` 欄位改為從 `BatteryStatus.remaining` 真實取值。

對外 Topic 數量不變（仍只有 `/fmu{N}_agent/state`），欄位向下相容（僅新增、不刪不改型別），不影響既有 GCS 與其他 Agent 的訂閱。

## User Stories

1. As a GCS developer, I want to receive lat/lon/alt for each drone, so that I can plot drone positions on a map and correlate them with MAVLink telemetry.
2. As a GCS developer, I want to see battery voltage for each drone, so that I can display a voltage gauge and trigger low-voltage alerts in the ground station UI.
3. As a GCS developer, I want to see battery current draw for each drone, so that I can estimate remaining flight time and detect abnormal power consumption.
4. As a GCS developer, I want `battery_remaining` to reflect the actual PX4 battery estimate, so that the fuel gauge in the GCS shows accurate state of charge instead of a dummy 100% value.
5. As a formation/mission planner developer, I want the AgentState message to remain backward-compatible with existing subscriptions, so that the formation and trajectory nodes continue to work without code changes.
6. As an operator, I want to detect when GPS data is invalid (e.g. during indoor flight or GPS loss), so that I don't rely on stale or incorrect lat/lon values.
7. As a test engineer, I want `StatePublisher::fill_agent_state` to have unit-testable GPS and battery mapping logic, so that data translation correctness is verified independently of SITL.
8. As a test engineer, I want to verify GPS and battery fields appear on the actual ROS2 topic during SITL, so that end-to-end integration is confirmed before deployment.

## Implementation Decisions

### 訊息變更

`AgentState.msg` 新增三個欄位，不修改既有欄位名稱或型別，確保下游相容。

```ros2
# 既有欄位 (不變)
builtin_interfaces/Time stamp
float32[3] position_ned
uint8 flight_mode
float32 battery_remaining
bool online

# 新增欄位
float32[3] position_global    # [lat(deg), lon(deg), alt(m AMSL)]
float32 battery_voltage_v     # 電池電壓 (V)，無資料時為 NaN
float32 battery_current_a     # 電池電流 (A)，無資料時為 NaN
```

`position_global` 使用 `float32` 而非 PX4 原生的 `float64 lat/lon`。經緯度 float32 精度約 2.4 公尺——足夠 GCS 地圖顯示與多機監控；若未來編隊計算需要 float64，可再新增 `position_global_64` 欄位或獨立訊息。

### 模組變更

**StatePublisher**：
- `fill_agent_state` 簽名從 `(AgentState&, VehicleOdometry&, FlightState, bool)` 擴充為 `(AgentState&, VehicleOdometry&, FlightState, bool, const VehicleGlobalPosition*, const BatteryStatus*)`
- GPS 與 Battery 參數為指標，nullptr 表示未收到資料（保持 `online=true` 時容許部分欄位無資料）
- `position_global`：直接複製 `lat`, `lon`, `alt`
- `battery_voltage_v` / `battery_current_a`：直接複製
- `battery_remaining`：改為從 `BatteryStatus.remaining` 取值，無資料時維持 `1.0`（保守預設）

**BasicControllerNode**：
- 新增兩個 subscriber 成員變數：
  - `px4_msgs::msg::VehicleGlobalPosition` 訂閱 `fmu/out/vehicle_global_position`
  - `px4_msgs::msg::BatteryStatus` 訂閱 `fmu/out/battery_status`
- 新增兩個回呼，快取最新訊息至成員變數
- `state_publish_cb` 中將快取的 GPS 與 Battery 訊息傳入 `StatePublisher::fill_agent_state`
- 兩個 subscriber 使用既有的 `rclcpp::QoS(10).best_effort()` 模式（與 odometry subscriber 一致）

### 架構決策

- AgentState 維持為 **唯一對外狀態 topic**，不拆出額外的 GPS 或 Battery 專屬 topic，符合「外部可見 topic 應盡量精簡」的專案約束。
- GPS 與電池資料缺失時不影響 Agent 的 `online` 旗標與 NED 位置發布——這兩個 subscriber 僅是補充資料源，非關鍵路徑。
- 編隊計算仍使用 NED 座標——`position_global` 目前僅供 GCS 顯示，不參與 FormationCalculator。真實多機 GPS 編隊留待後續 spec 處理。
- `battery_remaining` 現有欄位的語義不變（0.0~1.0），僅資料源從硬編碼改為 PX4，對下游完全透明。

## Testing Decisions

### 測試原則

- 只測外部行為（輸入→輸出），不測內部實作細節。
- 單元測試覆蓋所有欄位的轉譯正確性（含 nullptr 容錯路徑）。
- 整合測試確認 SITL 環境中 topic 欄位非零且合理。

### 單元測試（test/test_state_publisher.cpp）

延續既有 gtest 模式，新增測試案例：

1. `FillAgentState_GpsFromGlobalPosition`：GPS 資料正確複製至 `position_global`
2. `FillAgentState_BatteryFromBatteryStatus`：電壓/電流/剩餘比例正確複製
3. `FillAgentState_GpsNullptr`：GPS 為 nullptr 時 `position_global` 填 NaN
4. `FillAgentState_BatteryNullptr`：Battery 為 nullptr 時 `battery_remaining=1.0`、電壓電流填 NaN
5. `FillAgentState_AllPresent`：所有資料齊全時的完整輸出
6. `FillAgentState_BackwardCompat`：新簽名下既有欄位（NED/flight_mode/online）行為不變

### 整合測試（AutoTest/）

在既有 `quick_takeoff.sh` 基礎上新增驗證：
- 啟動 SITL → 訂閱 `/fmu1_agent/state` → 確認 `position_global` 三欄位非 NaN 且在合理範圍（lat 25°±3°, lon 121°±3°, alt 0~500m）
- 確認 `battery_voltage_v` > 0 且 `battery_remaining` < 1.0（SITL 會模擬放電）

### 文件更新

- `docs/taiphoon-agent-interface.md`：更新 AgentState 欄位表，加入 `position_global`、`battery_voltage_v`、`battery_current_a`
- 版本號從 1.0 → 1.1

## Out of Scope

- GPS 座標用於編隊位置計算（FormationCalculator 維持 NED）
- WGS84 ↔ NED 轉換工具函式
- GPS 訊號品質欄位（eph/epv/satellite count）
- 電池溫度、cell voltage、cycle count 等 BatteryStatus 中較細節的欄位
- 電池低電壓自動 RTL 邏輯（應由 PX4 內部 failsafe 處理，非 Agent 責任）

## Further Notes

- `VehicleGlobalPosition` 的 `lat`/`lon` 是 `float64`，存入 `float32` 時在編譯層會觸發 narrowing conversion warning，需顯式 `static_cast<float>` 並加註說明精度取捨。
- SITL 中 `battery_status` topic 預設發布頻率約 1Hz，低於 AgentState 的 10Hz。`state_publish_cb` 每次發布 AgentState 時使用最新快取值，電池欄位可能在數個週期內不變——這是預期行為，GCS 應搭配 `stamp` 判斷資料新鮮度。
- docker-compose.yml 中每台無人機的 `fmu/out/vehicle_global_position` launch 時已存在（PX4 SITL 自動輸出），不需額外設定 uXRCE-DDS bridge。
