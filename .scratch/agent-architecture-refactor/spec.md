# Agent 架構重構：分層設計與 Agent-to-Agent 狀態發布

Status: ready-for-agent

## Problem Statement

目前 Taiphoon_Agent 的編隊節點（`FormationNode`）直接訂閱所有飛控的 PX4 原生 topic（`/fmu{N}/fmu/out/vehicle_odometry`），此做法有三個問題：

1. **違反一 Agent 一機架構邊界**：一個 Agent 跨越邊界讀取其他飛控的內部資料。
2. **耦合 PX4 實作細節**：PX4 升級改 topic 名稱或訊息格式時，所有 Agent 都需修改。
3. **缺乏清楚的分層**：飛控橋接、編隊計算、路徑規劃混在少數節點中，難以獨立開發與測試。

## Solution

重構為四層架構，引入 Agent-to-Agent 狀態發布機制：

```
任務規劃層 (MissionPlanner)  →  接收編隊命令，計算自身目標位置
        ↓ 目標 NED
軌跡控制層 (TrajectoryController)  →  規劃無碰撞路徑，輸出 waypoint 序列
        ↓ waypoints
位置控制層 (BasicControllerNode)  →  安全監控 + 餵入 PX4 TrajectorySetpoint
        ↓
飛控橋接層 (BasicController)  →  PX4 Offboard mode
```

每個 Agent 發布自身狀態至 `/fmu{N}_agent/state` 與 `/fmu{N}_agent/attitude`，其他 Agent 透過同儕發現機制動態訂閱，不再直接存取任何 PX4 原生 topic。

## User Stories

1. 作為開發者，我希望每個 Agent 只控制一台飛控，使架構邊界清晰，避免跨機耦合。
2. 作為任務規劃層，我希望自動發現線上有哪些同儕 Agent，無需手動設定 drone_count。
3. 作為任務規劃層，我希望訂閱其他 Agent 的 `/fmu*_agent/state` 來獲取位置資訊，而非直接存取 PX4 odometry。
4. 作為飛控橋接層，我希望將 PX4 的里程計資料轉譯為 Agent 狀態對外發布，隔離 PX4 內部格式變更的影響。
5. 作為軌跡控制層，我希望接收目標 NED 位置，規劃一條與其他無人機保持 ≥3m 安全邊界的路徑。
6. 作為軌跡控制層，我希望輸出帶時間戳的 waypoint 序列，供位置控制層執行。
7. 作為位置控制層，我希望監控 PX4 飛行狀態，當偏差過大時介入（強制懸停或模式切換）。
8. 作為操作員，我希望透過 `/fmu_all_agent/command` 傳送編隊命令（如 FORMATION_CIRCLE），所有 Agent 同步響應。
9. 作為操作員，我希望在 Docker 容器內執行任何 ROS2 指令時，無需手動 source workspace。
10. 作為開發者，我希望 Agent state topic 的更新頻率可調整（預設 10Hz）。
11. 作為開發者，我希望姿態資訊有獨立 topic（`/fmu{N}_agent/attitude`），與其他狀態分離。
12. 作為軌跡控制層，我希望預留外部避障節點的介面，未來可接入而不需大幅修改。

## Implementation Decisions

### 模組拆分

| 現有模組 | 重構後 | 說明 |
|----------|--------|------|
| `BasicControllerNode` | 維持 + 擴充 | 新增 `/fmu{N}_agent/state` 與 `/attitude` 發布；保留安全監控責任 |
| `BasicController` | 維持 | 不變，繼續處理 PX4 Offboard 橋接 |
| `AgentCommandHandler` | 維持 | 不變，繼續解析字串命令 |
| `FormationNode` | 重構為 `MissionPlanner` | 移除對 PX4 odometry 的直接訂閱；新增同儕發現；計算編隊目標 NED |
| （無） | 新增 `TrajectoryController` | 接收目標 NED、規劃無碰撞路徑、輸出 waypoint 序列 |

### 自訂訊息定義

**AgentState.msg** — 以 10Hz 發布至 `/fmu{N}_agent/state`

```
float32[3] position_ned     # NED 位置 (x, y, z)，公尺
uint8 flight_mode            # 飛行模式 (枚舉: IDLE=0, OFFBOARD=1, ...)
float32 battery_remaining    # 剩餘電量 (0.0 ~ 1.0)
bool online                  # Agent 是否在線
builtin_interfaces/Time stamp
```

**AgentAttitude.msg** — 以 10Hz 發布至 `/fmu{N}_agent/attitude`

```
float32[4] q                 # 姿態四元數 (w, x, y, z)
builtin_interfaces/Time stamp
```

### 同儕發現機制

- Agent 啟動時呼叫 `get_topic_names_and_types()`，掃描符合 `/fmu*_agent/state` 模式的 topic。
- 排除自身 namespace（如 `/fmu1_agent`），對其餘 topic 動態建立 subscription。
- 以 3 秒為週期重新掃描，偵測新機加入或離線。
- 不使用靜態參數 `drone_count`。

### 編隊目標計算（任務規劃層責任）

- 任務規劃層訂閱所有已發現的 `/fmu*_agent/state`，維護各機位置快取與線上狀態。
- 收到編隊命令時，根據線上機數與編隊類型計算自身目標 NED。
- 編隊類型：`FORMATION_CIRCLE R alt`（環形）、`FORMATION_HORI_LINEAR s alt`（水平直列）、`FORMATION_VERT_LINEAR s`（垂直直列）。
- 邏輯沿用現有 `FormationNode::compute_centroid()` 與 `compute_formation_target()`。

### 軌跡控制層介面

- 輸入：目標 NED 位置 `(x, y, z)` + 所有其他機的當前位置
- 輸出：帶時間戳的 waypoint 序列（`vector<Waypoint>`，每個 waypoint 含 `NED` + `timestamp`）
- 安全邊界：路徑上任一點與其他機位置 ≥ 3m
- 預留避障介面：接收外部節點的障礙物位置更新

### 位置控制層（BasicControllerNode 內）

- 接收軌跡層的 waypoint 序列，逐一餵入 `BasicController::set_ned_target()`
- 監控 PX4 實際位置與目標的偏差
- 偏差超過閾值（參數化，預設 2m）時強制發送 hover 命令

### Docker 環境修復

- 在 Dockerfile 的 builder 與 runtime 階段新增 `/etc/profile.d/ros2.sh`，自動 source ROS2 + px4_msgs + Taiphoon_Agent workspace。
- 這使 `docker compose exec taiphoon-agt -- ros2 <cmd>` 在非互動式 shell 也能正常執行。
- `skills/ros2-tools/SKILL.md` 已簡化，移除所有 `$TAIPHOON` 路徑變數與手動 source 指令。

## Testing Decisions

### 測試接縫

最高測試接縫為 **Agent-to-Agent state topic**——`/fmu{N}_agent/state` 與 `/fmu{N}_agent/attitude`。此接縫之上的邏輯可完全獨立測試。

### 測試層級

1. **Agent state 發布驗證**（無需 PX4 SITL）
   - 啟動單一 Agent，驗證 `/fmu{N}_agent/state` 與 `/attitude` topic 存在且訊息格式正確。
   - 驗證 topic 發布頻率符合設定（預設 10Hz）。

2. **同儕發現驗證**（無需 PX4 SITL）
   - 使用 `ros2 topic pub` 模擬其他 Agent 的 state。
   - 驗證任務規劃層正確發現並訂閱模擬 topic。

3. **編隊目標計算驗證**（無需 PX4 SITL）
   - 注入三組已知 Agent state，發送 FORMATION_CIRCLE 命令。
   - 驗證輸出的自身目標 NED 與手算結果一致（沿用既有 `AutoTest/20260728_formation/test_log.md` 的期望值）。

4. **軌跡防撞驗證**（需 PX4 SITL）
   - 三機模擬一字型起飛後切換環形編隊。
   - 驗證路徑上任兩機距離始終 ≥ 3m。

### 測試紀錄

所有測試結果存放於 `AutoTest/` 目錄，格式遵循既有慣例。多機測試需啟動 3 台 PX4 SITL。

## Out of Scope

- 外部避障節點的具體實作（僅預留介面）
- Gazebo 模擬（沿用 SIH 內建模擬器）
- WiFi AP multicast 相容性驗證（屬實機部署階段）
- 4G/5G 或 SiK radio 資料鏈測試
- `uxrce_dds_agent-tools` 與 `px4-sitl-launcher` Skill 修改（此兩 Skill 不受本次重構影響）

## Further Notes

- 參閱 `CONTEXT.md` 取得領域詞彙定義。
- 參閱 `docs/adr/0001-agent-state-pub-sub.md` 取得架構決策背景。
- 參閱 `docs/development-workflow.md` 取得開發→審查→測試迭代流程。
- Git commit message 使用 `模組: 更改項目` 格式。
- 分支命名：`arch-refactor-dev`。
