# Taiphoon Agent ROS2 地面控制站 (GCS) 介面規格書

> **版本**：1.4
> **更新日期**：2026-08-10
> **適用對象**：ROS2 地面控制站 (GCS) 開發者
>
> 本文件完整列出 Taiphoon_Agent 專案中所有 ROS2 節點、Topic、Service、自訂訊息格式與命令協定，
> 供 GCS 開發團隊進行訂閱、發布與監控整合。

---

## 目錄

1. [系統架構總覽](#1-系統架構總覽)
2. [節點 (Node) 清單](#2-節點-node-清單)
3. [自訂訊息 (Custom Message) 定義](#3-自訂訊息-custom-message-定義)
4. [Topic 完整清單](#4-topic-完整清單)
5. [Service 完整清單](#5-service-完整清單)
6. [命令協定 (Command Protocol)](#6-命令協定-command-protocol)
7. [命名空間規則](#7-命名空間規則)
8. [QoS 設定摘要](#8-qos-設定摘要)
9. [多機部署注意事項](#9-多機部署注意事項)
10. [Launch 參數](#10-launch-參數)

---

## 1. 系統架構總覽

```
MissionPlanner ── NedTarget ──┬──▶ TrajectoryController
                              │             │
                              │             └──▶ /waypoints
                              ▼                  （僅規劃／觀測）
其他 AgentState ──▶ [防撞關卡] ──▶ BasicController ──▶ PX4
                         ▲
GCS 移動命令 ─────────────┘
```

> `/{agent_namespace}/waypoints` 是軌跡規劃與觀測輸出，目前沒有接入
> `BasicController` 或 PX4 控制鏈。BasicController 執行控制目標前的防撞關卡，才是
> NED 目標送往 PX4 前的最終控制關卡。

每一台無人機對應一個 Taiphoon_Agent 實例，內含四個節點：

| 節點 | 功能 | 必有 |
|------|------|------|
| `basic_controller_node` | 飛控橋接、狀態發布、命令轉譯 | ✅ |
| `mission_planner` | 編隊計算、同儕發現、NED 目標輸出 | 多機時 |
| `trajectory_controller` | 路徑規劃、碰撞檢查、waypoint 發布 | ✅ |
| `agent_supervisor` | Agent 容器生命週期管理，提供失效模擬服務 | ✅ |

---

## 2. 節點 (Node) 清單

### 2.1 basic_controller_node

| 屬性 | 值 |
|------|-----|
| **執行檔** | `basic_controller` |
| **節點名稱** | `basic_controller_node` |
| **命名空間** | `fmu{N}`（例如 `fmu1`） |
| **描述** | 核心飛控橋接節點。連接 PX4 並提供 Agent 對外狀態。 |

**內部組件**：
- `BasicController`：PX4 Offboard 控制（arm/disarm/takeoff/land/move）
- `AgentCommandHandler`：接收並解析高階命令字串
- 防撞關卡：以同儕 AgentState 檢查三維安全距離，決定移動目標是否可交給 BasicController 執行
- `PositionSafetyMonitor`：位置偏差安全監控
- `StatePublisher`：填充 AgentState / AgentAttitude 訊息

### 2.2 mission_planner

| 屬性 | 值 |
|------|-----|
| **執行檔** | `mission_planner` |
| **節點名稱** | `mission_planner` |
| **命名空間** | `fmu{N}` |
| **描述** | 任務規劃層。動態同儕發現、編隊計算、NED 目標發布。 |
| **啟動條件** | `drone_count > 1` |

**內部組件**：
- `FormationCalculator`：編隊位置計算
- `PeerMatcher`：同儕比對與匹配

### 2.3 trajectory_controller

| 屬性 | 值 |
|------|-----|
| **執行檔** | `trajectory_controller` |
| **節點名稱** | `trajectory_controller` |
| **命名空間** | `fmu{N}` |
| **描述** | 軌跡控制層。路徑生成、碰撞檢查、waypoint 序列發布；輸出目前不直接控制 PX4。 |

**內部組件**：
- `TrajectoryPlanner`：直線路徑生成與到達時間計算
- `PeerMatcher`：同儕位置蒐集

### 2.4 agent_supervisor

| 屬性 | 值 |
|------|-----|
| **執行檔** | `agent_supervisor` |
| **節點名稱** | `agent_supervisor` |
| **命名空間** | `fmu{N}_agent`（例如 `fmu1_agent`） |
| **描述** | Agent 容器生命週期管理節點，提供刻意模擬飛行電腦失效的 shutdown service。 |

**對外介面**：
- `/{agent_namespace}/shutdown`：詳見 [第 5 節](#5-service-完整清單)。

---

## 3. 自訂訊息 (Custom Message) 定義

所有自訂訊息位於套件 `taiphoon_agent` 的 `msg/` 目錄。

### 3.1 AgentState (`taiphoon_agent/msg/AgentState`)

Agent 對外狀態，由 `basic_controller_node` 發布。`flight_mode` 是經 PX4 實際觀測結果校正後的
任務階段，不代表最近收到的命令意圖。位置資訊由既有的 `position_ned` 與
`position_global` 提供，不另外複製 PX4 Position 欄位。`position_global` 唯一取自
`/fmu{N}/fmu/out/vehicle_global_position`；AgentState 名目發布頻率至少為 10 Hz，若 PX4
原始資料更新較慢，期間會重複最近一筆有效值，不進行插值。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `position_ned` | `float32[3]` | NED 座標位置（前向/右向/向下，公尺） |
| `position_global` | `float32[3]` | 全域 GPS 座標 [lat(deg), lon(deg), alt(m AMSL)]，來自 PX4 VehicleGlobalPosition |
| `flight_mode` | `uint8` | 經 PX4 觀測校正後的任務階段（對應 FlightState 枚舉，見下表），不是命令意圖 |
| `battery_remaining` | `float32` | 剩餘電量比例（0.0 ~ 1.0），來自 PX4 BatteryStatus |
| `battery_voltage_v` | `float32` | 電池電壓（V），來自 PX4 BatteryStatus |
| `battery_current_a` | `float32` | 電池電流（A），來自 PX4 BatteryStatus |
| `online` | `bool` | Agent 是否在線 |
| `armed` | `bool` | PX4 實際解鎖狀態 |
| `landed` | `bool` | PX4 實際落地狀態 |
| `px4_nav_state` | `uint8` | PX4 `VehicleStatus.nav_state` 原始值；Manual、Position、Offboard 等 PX4 原生模式由此欄位表示 |
| `px4_state_valid` | `bool` | PX4 狀態欄位是否有效；只有此值為 `true` 時，`armed`、`landed` 與 `px4_nav_state` 才可信 |

> **PX4 模式判讀**：`px4_nav_state` 不經 Agent 重新編碼，原樣對應 PX4
> `VehicleStatus.nav_state`。GCS 應先確認 `px4_state_valid=true`，再依相同版本
> `px4_msgs` 的 `VehicleStatus` 常數判讀 Manual、Position、Offboard 等原生導航模式。

**FlightState 枚舉對照表**：

| 值 | 名稱 | 說明 |
|----|------|------|
| 0 | `IDLE` | 閒置 |
| 1 | `OFFBOARD_INIT` | 等待 Offboard 模式啟用 |
| 2 | `OFFBOARD_ACTIVE` | Offboard 已啟用，等待解鎖 |
| 3 | `ARMED` | 已解鎖 |
| 4 | `TAKEOFF` | 起飛中 |
| 5 | `HOVER` | 懸停中 |
| 6 | `RTL` | 返航中 |
| 7 | `LAND` | 降落中 |
| 8 | `MOVING` | 水平移動中 |
| 9 | `POST_RTL` | RTL 降落後 |
| 10 | `ERROR` | 錯誤狀態 |

### 3.2 AgentAttitude (`taiphoon_agent/msg/AgentAttitude`)

Agent 姿態訊息，由 `basic_controller_node` 發布。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `q` | `float32[4]` | 四元數姿態（w, x, y, z） |

### 3.3 NedTarget (`taiphoon_agent/msg/NedTarget`)

NED 絕對目標位置，由 `mission_planner` → `basic_controller_node` 與 `trajectory_controller`。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `x` | `float32` | NED X 軸位置（前向為正，公尺） |
| `y` | `float32` | NED Y 軸位置（右向為正，公尺） |
| `z` | `float32` | NED Z 軸位置（向下為正，公尺） |
| `yaw` | `float32` | 目標航向角（度，順時針為正），`NaN` 表示不控制偏航 |

### 3.4 Waypoint (`taiphoon_agent/msg/Waypoint`)

單一軌跡 waypoint。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `position_ned` | `float32[3]` | NED 位置（前向/右向/向下，公尺） |
| `arrival_time` | `builtin_interfaces/Time` | 預定到達時間 |

### 3.5 WaypointSequence (`taiphoon_agent/msg/WaypointSequence`)

Waypoint 序列，由 `trajectory_controller` 發布。

| 欄位 | 型別 | 說明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 訊息時間戳 |
| `waypoints` | `Waypoint[]` | waypoint 序列 |

---

## 4. Topic 完整清單

### 4.1 PX4 飛控橋接 Topic（命名空間：`fmu{N}`）

這些 Topic 是 Taiphoon_Agent 與 PX4 SITL 之間的內部通訊。**GCS 通常不需直接操作**，但理解這些 Topic 有助於除錯。

| Topic | 方向 | 型別 | QoS |
|-------|------|------|-----|
| `fmu/in/offboard_control_mode` | 發布 → PX4 | `px4_msgs/OffboardControlMode` | best_effort (10) |
| `fmu/in/trajectory_setpoint` | 發布 → PX4 | `px4_msgs/TrajectorySetpoint` | best_effort (10) |
| `fmu/in/vehicle_command` | 發布 → PX4 | `px4_msgs/VehicleCommand` | best_effort (10) |
| `fmu/out/vehicle_odometry` | 訂閱 ← PX4 | `px4_msgs/VehicleOdometry` | best_effort (10) |
| `fmu/out/vehicle_global_position` | 訂閱 ← PX4 | `px4_msgs/VehicleGlobalPosition` | best_effort (10) |
| `fmu/out/vehicle_command_ack_v1` | 訂閱 ← PX4 | `px4_msgs/VehicleCommandAck` | best_effort (10) |
| `fmu/out/vehicle_status_v4` | 訂閱 ← PX4 | `px4_msgs/VehicleStatus` | best_effort (10) |

> **注意**：Topic 名稱為相對路徑，實際解析後會加上節點命名空間，例如 `/fmu1/fmu/in/offboard_control_mode`。

### 4.2 Agent 對外 Topic（GCS 關注重點）

這些是 **GCS 應直接訂閱與發布** 的 Topic。

#### 4.2.1 狀態發布（Agent → GCS）

| Topic 模板 | 說明 | 型別 | 頻率 | QoS |
|------------|------|------|------|-----|
| `/{agent_namespace}/state` | Agent 狀態（位置、飛行模式、電量） | `taiphoon_agent/AgentState` | ≥ 10 Hz（名目） | reliable (10) |
| `/{agent_namespace}/attitude` | Agent 姿態（四元數） | `taiphoon_agent/AgentAttitude` | ≥ 10 Hz（名目） | reliable (10) |

**實際範例**（4 機部署）：

| 無人機 | state Topic | attitude Topic |
|--------|------------|----------------|
| fmu1 | `/fmu1_agent/state` | `/fmu1_agent/attitude` |
| fmu2 | `/fmu2_agent/state` | `/fmu2_agent/attitude` |
| fmu3 | `/fmu3_agent/state` | `/fmu3_agent/attitude` |
| fmu4 | `/fmu4_agent/state` | `/fmu4_agent/attitude` |

#### 4.2.2 命令下達（GCS → Agent）

| Topic | 說明 | 型別 | QoS |
|-------|------|------|-----|
| `/{agent_namespace}/command` | 發送命令給**指定** Agent | `std_msgs/String` | reliable (10) |
| `/fmu_all_agent/command` | 廣播命令給**所有** Agent | `std_msgs/String` | reliable (10) |

> **GCS 設計建議**：
> - 使用 `/{agent_namespace}/command` 對單一無人機下達移動、模式切換命令。
> - 使用 `/fmu_all_agent/command` 發送編隊命令（FORMATION_*）或全體同步命令（如全體 RTL）。
> - 命令格式詳見 [第 6 節](#6-命令協定-command-protocol)。

#### 4.2.3 內部控制流（Agent 內部通訊，GCS 可監控）

| Topic 模板 | 說明 | 型別 | QoS |
|------------|------|------|-----|
| `/{agent_namespace}/ned_target` | 編隊計算出的 NED 目標 | `taiphoon_agent/NedTarget` | reliable (10) |
| `/{agent_namespace}/waypoints` | 軌跡控制器輸出的規劃／觀測 waypoint 序列；目前不是 PX4 控制來源 | `taiphoon_agent/WaypointSequence` | reliable (10) |
| `/{agent_namespace}/obstacles` | 障礙物資訊（預留介面，尚未實作） | `std_msgs/String` | best_effort (10) |

#### 4.2.4 跨 Agent 通訊（同儕發現）

`mission_planner` 與 `trajectory_controller` 會動態訂閱其他 Agent 的 `/state` topic：

| 訂閱者 | 訂閱 Topic | 用途 |
|--------|-----------|------|
| `mission_planner` | `/{peer_namespace}/state` | 取得同儕位置以計算編隊目標 |
| `trajectory_controller` | `/{peer_namespace}/state` | 取得同儕位置以檢查路徑碰撞 |
| `basic_controller_node` | `/{peer_namespace}/state` | 在移動目標送往 PX4 前執行最終防撞判斷 |

> **同儕發現與離線判定機制**：透過 ROS2 node 名稱掃描自動發現線上 Agent，
> 只辨識 `/fmu<N>_agent/...` 節點（相容模式另含 `/fmu<N>/basic_controller_node`）。
> 離線判定以 `/fmu<N>_agent` 節點存在性為主軸，並以「超過
> `peer_timeout_s`（預設 2 秒）未收到該 Agent 的 `/fmu<N>_agent/state`」作為
> 加速器（避免等待 DDS 節點清除延遲）。掃描週期為 3 秒（`peer_scan_interval_s`）。
> Agent 從 ROS2 discovery 消失時，MissionPlanner、TrajectoryController 與
> BasicController 會將它移出活動同儕及防撞集合；該機後續由 PX4 failsafe
> 處理，其餘 Agent 直接重新編隊。Agent 再次出現時會重新建立訂閱。
>
> 機群之間一律只訂閱 `/fmu<N>_agent/state` 等 Agent 對外 topic，
> 不直接訂閱其他無人機的 PX4 原生 topic（`/fmu<N>/fmu/out/*`）。

### 4.2.5 第一版觀測式防撞契約

- 安全距離使用三維直線距離，任兩機必須維持至少 3 公尺。
- BasicController 執行控制目標前的防撞關卡是送往 PX4 前的最終控制關卡，適用於編隊與一般移動目標。
- 發現路徑衝突、仍存在的同儕狀態無效、位置資料無效，或各機 GPS 無法轉換成可比較的共同座標時，Agent 原地保持並保留待執行目標。
- Agent 從 ROS2 discovery 消失後，不再參與防撞判斷；後續飛行交由該機 PX4 failsafe 處理。
- 當資料恢復有效且目標重新符合安全距離後，Agent 自動續行，不要求 GCS 重新發布命令。
- `/{agent_namespace}/waypoints` 目前只表示 TrajectoryController 的規劃結果，不會送入 PX4，也不能作為 PX4 已執行防撞路徑的證據。
- 第一版只依賴最近觀測位置，不使用速度、加速度或同儕未來軌跡；高速相向、通訊延遲與制動距離不在預測模型內。
- 本功能沿用既有 `AgentState` 與 `NedTarget`，不新增 Topic、自訂訊息或 GCS 命令。
- `collision_safety_margin_m` 預設為 `3.0` 公尺，低於 `3.0` 的設定會被限制為
  `3.0`；`collision_peer_timeout_s` 預設為 `2.0` 秒。參數由每個
  `basic_controller_node` 個別載入。
- TrajectoryController 的 `safety_margin_m` 與 `peer_timeout_s` 預設同樣為
  `3.0` 公尺與 `2.0` 秒；`safety_margin_m` 低於 `3.0` 時也會被限制為 `3.0`。

---

## 5. Service 完整清單

### 5.1 Agent shutdown service

| Service | 提供節點 | 型別 | 說明 |
|---------|----------|------|------|
| `/{agent_namespace}/shutdown` | `agent_supervisor` | `std_srvs/srv/SetBool` | 僅接受 `request.data = true`；回應送出後以 SIGTERM 觸發 Agent 容器 PID 1 立即退出 |

**契約與安全限制**：

- `request.data` 必須為 `true`；`false` 不會觸發終止。
- 接受請求後，服務回應先送出，接著以 SIGTERM 觸發 PID 1 的立即退出，刻意模擬飛行電腦失效。
- 此服務不檢查飛行狀態、不執行 RTL 或 LAND；後續行為完全由 PX4 failsafe 決定。
- 不提供也不允許 `/fmu_all_agent/shutdown` 廣播；shutdown 必須指定單一 Agent namespace。
- 若 Compose 服務使用 `restart: no`，PID 1 終止後容器會保持停止，不會自動重新啟動。若使用自動重啟策略，容器可能重新啟動，服務也會再次出現，將無法保持失效狀態。

**呼叫範例**：

```bash
ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

**驗證服務與容器狀態**：

```bash
# 在 Agent 容器內確認服務存在與型別
ros2 service list | grep -F /fmu1_agent/shutdown
ros2 service type /fmu1_agent/shutdown
```

```bash
# 在 Agent 容器內呼叫；回應送出後容器 PID 1 應立即終止
ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

```bash
# 回到宿主機確認容器狀態與退出碼
docker compose ps fmu1_agent
docker inspect --format '{{.State.Status}} exit={{.State.ExitCode}} restart={{.RestartCount}}' fmu1_agent
```

## 6. 命令協定 (Command Protocol)

### 6.1 傳輸格式

- **Topic**：`/{agent_namespace}/command` 或 `/fmu_all_agent/command`
- **型別**：`std_msgs/msg/String`
- **編碼**：純文字 ASCII，不區分大小寫
- **格式**：`<COMMAND> [param1] [param2] [param3] [param4]`

### 6.2 支援的命令

#### 模式切換命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MODE_RTL` | 無 | 返航 (Return to Launch) |
| `MODE_LAND` | 無 | 降落 |
| `MODE_OFFBOARD` | 無 | 切換至 Offboard 模式 |
| `HOLD` | 無 | 位置保持 (Position Hold) |

#### 移動命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MOVE_FORWARD <x>` | 距離 (公尺) | 向前移動 |
| `MOVE_BACKWARD <x>` | 距離 (公尺) | 向後移動 |
| `MOVE_LEFT <x>` | 距離 (公尺) | 向左移動 |
| `MOVE_RIGHT <x>` | 距離 (公尺) | 向右移動 |
| `MOVE_UP <x>` | 距離 (公尺) | 向上移動 |
| `MOVE_DOWN <x>` | 距離 (公尺) | 向下移動 |
| `YAW <angle>` | 角度 (度) | 順時針旋轉 |
| `MOVE_POINT <x> <y> <z> <yaw>` | NED 座標 + 偏航 | 移動至絕對位置 |

#### 基礎控制命令

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `ARM` | 無 | 解鎖 |
| `DISARM` | 無 | 上鎖 |
| `TAKEOFF [altitude]` | 高度 (公尺，預設 3.0) | 起飛 |

#### 編隊命令（僅透過 `/fmu_all_agent/command` 廣播）

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `FORMATION_CIRCLE <radius> <altitude>` | 半徑 (公尺), 高度 (公尺) | 環形編隊 |
| `FORMATION_HORI_LINEAR <spacing> <altitude>` | 間距 (公尺), 高度 (公尺) | 水平一字編隊 |
| `FORMATION_VERT_LINEAR <spacing>` | 間距 (公尺) | 垂直一字編隊 |
| `FORMATION_CANCEL` | 無 | 取消編隊 |

> **編隊基準點**：收到 `FORMATION_*` 指令當下，以 GPS 將所有線上機體位置換算到
> 共同參考系後計算質心並固定為編隊圓心；線上成員增減時以目前位置重算。
> 若線上機體少於 2 台，編隊自動取消（發布一次目前位置目標後停止發布），
> 需重新廣播編隊指令才能恢復。

#### 相容性簡寫

| 命令 | 簡寫 |
|------|------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

### 6.3 命令使用範例

```bash
# 讓 fmu1 起飛至 5 公尺
ros2 topic pub /fmu1_agent/command std_msgs/msg/String "data: 'TAKEOFF 5'"

# 讓所有無人機返航
ros2 topic pub /fmu_all_agent/command std_msgs/msg/String "data: 'MODE_RTL'"

# 環形編隊（半徑 10 公尺，高度 20 公尺）
ros2 topic pub /fmu_all_agent/command std_msgs/msg/String \
  "data: 'FORMATION_CIRCLE 10 20'"

# fmu2 向前移動 3 公尺
ros2 topic pub /fmu2_agent/command std_msgs/msg/String "data: 'MOVE_FORWARD 3'"
```

---

## 7. 命名空間規則

| 命名空間 | 用途 | 範例 |
|----------|------|------|
| `fmu{N}` | 節點命名空間，對應 PX4 實例 | `fmu1`, `fmu2`, `fmu3` |
| `fmu{N}_agent` | Agent 對外 Topic 前綴 | `fmu1_agent`, `fmu2_agent` |
| `fmu_all_agent` | 全體 Agent 廣播 Topic | `/fmu_all_agent/command` |

**規則**：
- 一個 Taiphoon_Agent 只控制一台 PX4（`fmu{N}`）。
- 所有 Agent 對外 Topic 使用 `/{agent_namespace}/...` 前綴。
- 廣播命令固定使用 `/fmu_all_agent/command`。
- Shutdown service 僅使用 `/{agent_namespace}/shutdown`，不提供 `/fmu_all_agent/shutdown`。

---

## 8. QoS 設定摘要

| 介面類型 | Reliability | Durability | Depth |
|-----------|-------------|------------|-------|
| PX4 飛控 Topic | best_effort | volatile | 10 |
| Agent 狀態 Topic (`/state`, `/attitude`) | reliable | volatile | 10 |
| 命令 Topic (`/command`) | reliable | volatile | 10 |
| Shutdown Service (`/shutdown`) | reliable | volatile | 10 |
| 控制流 Topic (`/ned_target`, `/waypoints`) | reliable (pub) / best_effort (sub) | volatile | 10 |
| 障礙物 Topic (`/obstacles`) | best_effort | volatile | 10 |

---

> **文件維護**：此文件由 Taiphoon_Agent 團隊維護，每次新增或修改 Topic/Service/Node 時應同步更新。
