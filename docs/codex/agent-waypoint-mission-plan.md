# Taiphoon_Agent 修改規劃：航點任務、錯誤回報與重新啟動飛控

> 對象：Taiphoon_Agent 開發者（人員或 AI 開發工具）
> 來源：Taiphoon_QGCS 1.2.1 的改版需求與 2026-10-30 試飛問題
> 範圍：只描述 Taiphoon_Agent 端；介面契約確定後需同步到 `../Taiphoon_Agent_Interfaces/`
> 動工前請依 Agent 的 `AGENTS.md` 閱讀 `CONTEXT.md`、`docs/taiphoon-agent-interface.md`、
> `src/controllers/AGENTS.md`、`src/ego_swarm/AGENTS.md`、`src/nodes/AGENTS.md` 與 `CODE_CONSTRAINT.md`

本次包含三項功能，共用同一次介面改版與發布：

| 部分 | 功能 | 主要節點 |
|------|------|----------|
| A | 航點任務改由 Agent 執行 | `ego_swarm_planner` |
| B | 錯誤事件與 PX4 健康狀態回報給 GCS | `basic_controller_node`、`ego_swarm_planner` |
| C | 重新啟動飛控（`REBOOT_FC`）命令 | `basic_controller_node` |

**相容性**：QGCS 與 Agent 會同時升級，不需要與舊版互通，也不保留舊的逐段 `MOVE_POINT` 任務流程。

---

## 0. 已定案的決策（2026-10-30，人類開發者確認）

| 項目 | 決定 |
|------|------|
| 舊版相容 | 不考慮；QGCS 與 Agent 一起升級 |
| QGCS 斷線或關閉時 | 航點任務照樣繼續執行到完成 |
| 任務中收到 `HOLD` | 中止任務並停在目前位置；任務以外的 `HOLD` 行為不變 |
| QGCS 端 | 航點上傳與新命令都經 QGCS 的 GuardModule 驗證 |
| 飛控狀態不明時重新啟動 | 允許 `REBOOT_FC 1`；QGCS 以滑桿（L2）確認，前提是 PX4 解鎖時會拒絕（D3 情境 15 必須通過） |
| PX4 訊息版本（D6 第 0 步） | 維持目前版本：`PX4_MSGS_REF=main` 與現行 topic 名稱不變（見 E2 #12） |

---

# Part A：航點任務改由 Agent 執行

## A1. 背景與目標

### 現況（QGCS 逐段下令）

QGCS 的 MissionModule 依序送出單點 `MOVE_POINT x y z yaw`，自己判斷到達、停留 3 秒、再送下一段。
實作與 SITL 實測發現以下限制：

| 問題 | 原因 |
|------|------|
| 航點會整體偏移 | `MOVE_POINT` 座標是 planner-local NED，原點（planner 第一筆有效 WGS84）不對外發布；QGCS 只能假設它等於 PX4 local 原點，Agent 在空中重啟就不成立 |
| 到達判斷只能猜 | 執行 `MOVE_POINT` 時 `flight_mode` 維持 HOVER、不轉 MOVING；QGCS 只能用「接近 1 公尺持續 1 秒」或「離開起點後停止 1.5 秒」推測 |
| 停止任務要繞路 | `HOLD` 只鎖住 BasicController 的 setpoint，planner 仍發布原軌跡；QGCS 改送「`MOVE_POINT` 目前位置」 |
| 與 QGCS 連線綁死 | QGCS 斷線或關閉時任務停在當段；每段之間有網路往返延遲 |

### 目標

- QGCS **一次上傳**整條航線（WGS84 經緯度 + 高度 + 停留時間），Agent 自行換算座標、依序飛行、判斷到達、停留並回報進度。
- QGCS 只送「開始」與「停止」兩個命令，並訂閱任務狀態顯示進度。
- 航點飛行沿用 EGO-Swarm 既有目標流程，自動取得障礙物迴避與同儕避碰。

### 不在本版範圍

- 機群（全體）航點任務、編隊中的航點任務。
- 逐點航向（`yaw_deg` 先保留欄位，只接受 NaN）。
- 不停留直接通過航點（每個航點都會停下，與目前行為相同）。
- 任務持久化：Agent 重啟後任務遺失，不自動恢復。
- GCS 心跳與失聯處置（已定案：斷線時任務繼續）。
- `HOLD` 在任務以外的語意（既有問題，另案處理）。

## A2. 介面契約

### A2.1 新增訊息（`msg/`）

`msg/MissionWaypoint.msg`

```
# 航點任務的單一航點
# 水平位置使用 WGS84；高度基準由 WaypointMission.altitude_frame 決定

float64 latitude_deg      # WGS84 緯度（度）
float64 longitude_deg     # WGS84 經度（度）
float32 altitude_m        # 高度（公尺，向上為正），基準見 WaypointMission.altitude_frame
float32 dwell_s           # 到達後停留時間（秒）；最後一個航點不使用，到達即完成並懸停
float32 yaw_deg           # 保留欄位：NaN 表示維持目前目標航向；本版只接受 NaN
```

`msg/WaypointMission.msg`

```
# 單機航點任務上傳，GCS 發布於 /{agent_namespace}/mission/plan
# 上傳只載入任務、不會移動；需另送 MISSION_START <mission_id> 才開始

uint8 ALTITUDE_PX4_LOCAL=1        # 高度與 -AgentState.position_ned[2] 同基準（PX4 local 原點起算）

builtin_interfaces/Time stamp     # 發布時間
uint32 mission_id                 # GCS 指定的任務識別碼，不得為 0
uint8 altitude_frame              # 高度基準；本版只接受 ALTITUDE_PX4_LOCAL（0 視為未設定而拒絕）
MissionWaypoint[] waypoints       # 依序執行的航點
```

`msg/WaypointMissionStatus.msg`

```
# 單機航點任務狀態，Agent 發布於 /{agent_namespace}/mission/status
# 狀態改變時立即發布，並以 mission_status_rate_hz 週期重送目前狀態

uint8 IDLE=0          # 沒有任務
uint8 LOADED=1        # 已接受上傳，等待 MISSION_START
uint8 RUNNING=2       # 前往 current_index 航點
uint8 DWELLING=3      # 已到達 current_index 航點，停留中
uint8 COMPLETED=4     # 已到達最後一個航點，懸停中
uint8 ABORTED=5       # 執行中被中止
uint8 REJECTED=6      # 上傳或開始請求被拒絕；屬於事件，只對該 mission_id 發布一次

uint8 REASON_NONE=0
uint8 REASON_INVALID_PLAN=1        # 航點數量、座標、高度、停留、航向或高度基準不合法
uint8 REASON_OUT_OF_RANGE=2        # 航點與目前位置的水平距離超過上限
uint8 REASON_NOT_READY=3           # 尚無有效狀態，或 Home／PX4 local 換算尚未建立
uint8 REASON_NOT_HOVERING=4        # 未解鎖、已落地或不在懸停
uint8 REASON_BUSY=5                # 已有任務執行中，或編隊中
uint8 REASON_ID_MISMATCH=6         # MISSION_START 的識別碼與已載入任務不符，或載入已逾時
uint8 REASON_OPERATOR_STOP=10      # 收到 MISSION_STOP
uint8 REASON_COMMAND_OVERRIDE=11   # 收到其他命令（操作人員介入）
uint8 REASON_FLIGHT_STATE=12       # 飛行狀態離開懸停／移動（RTL、LAND、ERROR 等）
uint8 REASON_LANDED=13             # 已落地或上鎖
uint8 REASON_LEG_TIMEOUT=14        # 單段逾時未到達

builtin_interfaces/Time stamp
uint32 mission_id          # 狀態所屬任務；IDLE 時為 0
uint8 state
uint8 reason               # REJECTED／ABORTED 的原因，其他狀態為 REASON_NONE
uint16 current_index       # 目前航點（0 起算）；INVALID_PLAN／OUT_OF_RANGE 時指向第一個不合法航點
uint16 waypoint_count
string detail              # 英文補充說明，供日誌使用（例如觸發中止的命令名稱）
```

> 命名避開既有的 `Waypoint.msg`／`WaypointSequence.msg`（舊 NED 介面，僅純邏輯測試用途）。
> 欄位一次定好：ROS2 訊息增減欄位會改變 CDR 排列，QGCS 與 Agent 必須同時更新，因此 `yaw_deg`、
> `altitude_frame`、`reason` 先保留。

### A2.2 Topic 與 QoS

| Topic | 方向 | 型別 | QoS |
|-------|------|------|-----|
| `/{agent_namespace}/mission/plan` | GCS → Agent | `taiphoon_agent/WaypointMission` | reliable、volatile、depth 10（發布與訂閱相同） |
| `/{agent_namespace}/mission/status` | Agent → GCS | `taiphoon_agent/WaypointMissionStatus` | reliable、**transient_local**、depth 1 |

- `mission/plan` 不用 transient_local：避免 Agent 重啟後收到舊航線。QGCS 發布前會確認已有訂閱者，並以狀態回報確認收到。
- `mission/status` 用 transient_local：QGCS 晚加入或重新啟動時，立即取得最後狀態。QGCS 的 reader 也用 transient_local，
  Agent writer 若改成 volatile 會**無法配對**，兩邊必須一致。
- 只有 `ego_swarm_planner` 訂閱 `mission/plan`、發布 `mission/status`。
- DDS 名稱：`rt/uav_N_agent/mission/plan`（`taiphoon_agent::msg::dds_::WaypointMission_`）、
  `rt/uav_N_agent/mission/status`（`taiphoon_agent::msg::dds_::WaypointMissionStatus_`）。

### A2.3 新增命令（`std_msgs/String`，只接受 `/{agent_namespace}/command`）

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `MISSION_START <mission_id>` | 已載入任務的識別碼（1～4294967295 的整數） | 開始已載入的航點任務 |
| `MISSION_STOP` | 無 | 停止任務並停在目前位置；沒有任務時只清除已載入的任務 |

- 出現在 `/uav_all_agent/command` 的 `MISSION_*` 一律忽略，並發布 `COMMAND_REJECTED` 事件（Part B）。
- QGCS 輸出參數時整數一律以十進位整數表示（例如 `MISSION_START 4000000000`），不會出現小數點或指數；
  Agent 仍應以整數解析並拒絕非整數或超出範圍的值。

### A2.4 座標換算

- 水平：`planner_frame_.to_local_ned()` 直接把航點 WGS84 換成 planner-local NED 的 North／East，
  不再依賴「planner 原點 = PX4 local 原點」。
- 垂直（`ALTITUDE_PX4_LOCAL`）：航點的 PX4 local z = `-altitude_m`，再以既有的 `planner_to_px4_ned_offset_`
  轉回 planner-local：`z_planner = -altitude_m - offset.z`。這與 QGCS 機群面板顯示的高度（`-position_ned[2]`）一致，
  Agent 在空中重啟時也正確。
- 換算在**上傳時**完成並保存 planner-local 目標，執行中不重算。換算失敗即 `REJECTED / REASON_NOT_READY`。
- 注意 Part C：PX4 重新啟動後 EKF 原點可能改變，`planner_to_px4_ned_offset_` 必須重建。

## A3. 行為規格

### A3.1 狀態機

```
         上傳成功                MISSION_START(id 相符)
 IDLE ──────────────▶ LOADED ─────────────────────────▶ RUNNING ◀──────┐
  ▲  ◀── 載入逾時 ─────┘ │                                │  到達非最後點   │ 停留結束
  │      MISSION_STOP    │ 新上傳取代                      ▼              │
  │                      ▼                              DWELLING ───────┘
  │                    LOADED(新 id)                       │
  │                                                   到達最後點 ▼
  └──── 新上傳時才離開 ── COMPLETED／ABORTED ◀── 中止條件（任一狀態 RUNNING／DWELLING）

 REJECTED：上傳或 START 被拒時，對該 mission_id 發布一次事件，目前狀態不變。
```

- COMPLETED、ABORTED 保留到下一次上傳或 Agent 重啟，週期發布照常（晚加入的 GCS 看得到結果）。
- LOADED 超過 `mission_load_timeout_s`（預設 10 秒）未開始即回到 IDLE。

### A3.2 上傳驗證（不符即 `REJECTED`，不改變目前狀態）

| 檢查 | 原因碼 |
|------|--------|
| 任務執行中（RUNNING／DWELLING）或編隊中（`has_formation_`） | `REASON_BUSY` |
| `mission_id == 0`、`altitude_frame` 不是 `ALTITUDE_PX4_LOCAL` | `REASON_INVALID_PLAN` |
| 航點 0 個或超過 `mission_max_waypoints`（預設 50） | `REASON_INVALID_PLAN` |
| 經緯度非有限值或超出 ±90／±180 | `REASON_INVALID_PLAN` |
| 高度不在 `mission_min_altitude_m`～`mission_max_altitude_m`（預設 1～120） | `REASON_INVALID_PLAN` |
| `dwell_s` 非有限值、小於 0 或大於 `mission_max_dwell_s`（預設 60） | `REASON_INVALID_PLAN` |
| `yaw_deg` 不是 NaN | `REASON_INVALID_PLAN` |
| 任一航點與目前位置水平距離超過 `mission_max_range_m`（預設 1000） | `REASON_OUT_OF_RANGE` |
| 沒有有效 AgentState、planner Home 或 PX4 local 換算未建立 | `REASON_NOT_READY` |

驗證通過：保存換算後的目標，狀態 LOADED（覆蓋先前已載入但未開始的任務）。上傳本身不需在懸停，但必須可換算座標。

### A3.3 開始條件（`MISSION_START <id>`）

全部成立才轉 RUNNING，否則 `REJECTED`：

- 狀態為 LOADED 且識別碼相符（否則 `REASON_ID_MISMATCH`）。
- `flight_mission_lifecycle_.can_plan()`，且 AgentState `flight_mode == HOVER`、`px4_state_valid`（否則 `REASON_NOT_HOVERING`）。
- 不在編隊中（否則 `REASON_BUSY`）。
- 再檢查一次水平距離上限（上傳到開始之間可能已移動）。

開始時不改變 target yaw，BasicController 維持目前航向（與現行 QGCS 送出起始航向的效果相同）。

### A3.4 執行

- 每段把目前航點設為 Final Target，流程與單機 `MOVE_POINT` 的位置部分相同（`has_goal_`、`has_position_target_`、
  清除 `has_formation_`／`preserve_altitude_`、`invalidate_active_plan()`），但**不**改 target yaw。
- 到達判斷沿用既有 `evaluate_target_arrival()`（PX4 local 與 planner-local 都在容許誤差內），
  且只在 Final Target 與本段目標相同時才算本段到達（避免誤用其他來源的目標）。
- 到達後沿用既有 terminal hold；非最後一點轉 DWELLING，停留 `dwell_s` 後設定下一段目標。
  最後一點轉 COMPLETED，維持 terminal hold 懸停。
- 單段逾時：`mission_leg_base_timeout_s`（預設 30）+ 段長 ÷ `mission_min_expected_speed_mps`（預設 0.3），
  從該段開始計算；planner 因同儕仲裁、煞車或規劃失敗而等待的時間也算在內。逾時即中止並停在目前位置。
- GCS 斷線不影響執行（已定案）。

### A3.5 中止條件與處置

| 觸發 | 原因碼 | Agent 動作 |
|------|--------|------------|
| `MISSION_STOP` | `REASON_OPERATOR_STOP` | 停在目前位置 |
| 單機或全體 topic 收到任何非 `MISSION_*` 的有效命令（含 `HOLD`、`YAW`）；舊 `ned_target`、`mission_intent` 收到目標 | `REASON_COMMAND_OVERRIDE` | 先停在目前位置，再照常處理該命令（MOVE_*、MOVE_POINT、FORMATION_* 會覆寫目標） |
| AgentState `flight_mode` 離開 HOVER／MOVING（RTL、LAND、ERROR…） | `REASON_FLIGHT_STATE` | 不另建停止目標，交由 BasicController 既有流程 |
| `clear_grounded_mission()`（確認落地或上鎖） | `REASON_LANDED` | 隨既有流程清除 |
| 單段逾時 | `REASON_LEG_TIMEOUT` | 停在目前位置 |

- 「停在目前位置」：不得繼續飛往原航點，也不得飛回收到命令時的位置。建議以既有 `EmergencyBrakePlanner`
  依動力學煞停，再於停止點建立 terminal hold；QGCS 現行做法（Final Target 改成目前位置）在 SITL 驗證可行，可作為退路。
- 無法解析的命令字串不算介入，不中止任務（但會發布 `COMMAND_UNKNOWN` 事件）。
- 中止與完成後都不自動執行 RTL、LAND 或其他命令。

### A3.6 狀態發布

- 狀態、`current_index` 或原因改變時立即發布；另以 `mission_status_rate_hz`（預設 1 Hz）重送目前狀態。
- REJECTED 只針對被拒的 `mission_id` 發布一次，下一次週期發布回到目前實際狀態（可能是另一個任務的 RUNNING）。
- planner 啟動後即開始發布 IDLE，不需等待 Home 建立。

### A3.7 與既有功能的互動

| 功能 | 規則 |
|------|------|
| 編隊 | 編隊中拒絕上傳與開始；任務中收到 `FORMATION_*` 即中止（COMMAND_OVERRIDE），編隊照常建立 |
| 同儕避碰、障礙物 | 航點目標走一般 Final Target 流程，Local Goal、時空 A*、reservation、safety validator 全部照舊 |
| 同儕 terminal hold heartbeat | 停留與完成時沿用既有 terminal hold，其他 Agent 會把該點視為佔位 |
| 落地生命週期 | `clear_grounded_mission()` 一併清除任務並回報 ABORTED／LANDED |
| BasicController | `MISSION_*` 交由 planner，BasicController 只記錄已轉交（與 FORMATION_* 相同） |
| Agent 重啟 | 任務遺失，重啟後發布 IDLE；QGCS 以狀態不符判定任務中止 |

## A4. 實作規劃

### A4.1 檔案與修改點

| 檔案 | 修改 |
|------|------|
| `msg/MissionWaypoint.msg`、`msg/WaypointMission.msg`、`msg/WaypointMissionStatus.msg` | 新增 |
| `CMakeLists.txt` | `rosidl_generate_interfaces` 加入新訊息；新增測試目標 |
| `src/utils/CommandTypes.hpp` | `AgentCommand` 新增 `MISSION_START`、`MISSION_STOP` 與解析；`MISSION_START` 參數需為範圍內整數 |
| `src/ego_swarm/WaypointMissionExecutor.hpp`（新增，純邏輯、不依賴 ROS2） | 任務資料、驗證、狀態機、停留與逾時計時；輸入時間與到達事件，輸出目前目標與狀態快照 |
| `src/nodes/ego_swarm_planner_node.hpp/.cpp` | 新增 `mission/plan` 訂閱與 `mission/status` 發布、參數、座標換算、各 callback 與 `planner_loop_cb` 的掛載點（A4.2） |
| `src/nodes/basic_controller_node.cpp` | `MISSION_START`／`MISSION_STOP` 轉交 planner，回報成功 |
| `launch/`、`docker/entrypoint.sh`、`config/` | 若參數需由部署設定，依既有方式傳入；預設值可直接用 Node 預設 |
| `src/ego_swarm/AGENTS.md`、`src/nodes/AGENTS.md` | 補充新元件與資料流 |

`WaypointMissionExecutor` 建議介面（實際命名依 `CODE_CONSTRAINT.md`）：

```cpp
struct MissionLimits { std::size_t max_waypoints; double min_altitude_m, max_altitude_m,
                       max_range_m, max_dwell_s, leg_base_timeout_s, min_expected_speed_mps,
                       load_timeout_s; };
struct MissionLeg { Vector3 target_planner_ned; double dwell_s; };
struct MissionStatusSnapshot { uint32_t mission_id; uint8_t state, reason;
                               uint16_t current_index, waypoint_count; std::string detail; };

class WaypointMissionExecutor {
  // 驗證與換算後的航段由 node 傳入；executor 不處理 WGS84 換算，方便單元測試
  RejectResult load(uint32_t id, std::vector<MissionLeg> legs, double now_s);
  RejectResult start(uint32_t id, const StartConditions& conditions, double now_s);
  void stop(uint8_t reason, std::string detail);            // 是否需停在目前位置由呼叫端依原因決定
  void on_target_reached(const Vector3& final_target, double now_s);
  std::optional<Vector3> due_target(double now_s);          // 開始或停留結束時回傳下一段目標
  bool is_leg_timed_out(double now_s) const;
  bool is_active() const;                                    // LOADED／RUNNING／DWELLING
  MissionStatusSnapshot snapshot() const;
};
```

### A4.2 `ego_swarm_planner_node` 掛載點

1. **`mission_plan_cb`（新增）**：驗證格式 → 以 `planner_frame_` 與 `planner_to_px4_ned_offset_` 換算 → `executor.load()` → 發布狀態。
2. **`agent_command_cb`**：`MISSION_START` → 檢查 A3.3 條件 → `executor.start()`；`MISSION_STOP` → 中止並停在目前位置。
   其他有效命令且任務 active → 先中止（COMMAND_OVERRIDE）並停在目前位置，再照原流程處理該命令。
3. **`all_agent_command_cb`**：忽略 `MISSION_*`；其他有效命令且任務 active → 同上。注意這裡在 `can_plan()` 為 false 時
   會提早 return，中止判斷要放在它之前或另行處理。
4. **`ned_target_cb`、`mission_intent_cb`**：任務 active 時先中止（COMMAND_OVERRIDE）。
5. **`own_state_cb`**：任務 RUNNING／DWELLING 且 `px4_state_valid` 時，`flight_mode` 不是 HOVER／MOVING → 中止（FLIGHT_STATE）。
6. **`clear_grounded_mission()`**：任務 active 時回報 ABORTED／LANDED 並清除。
7. **`planner_loop_cb` 開頭**（Home 已建立之後、取得 Final Target 之前）：
   ```
   if (auto target = executor.due_target(now_s)) set_mission_goal(*target);   // 開始或停留結束
   if (executor.is_leg_timed_out(now_s)) { abort(LEG_TIMEOUT); stop_in_place(); }
   publish_mission_status_if_due(now);
   ```
8. **`planner_loop_cb` 到達分支**（`arrival.reached` 且非編隊、在 `has_goal_ = false` 之前）：
   `executor.on_target_reached(final_target, now_s)`。
9. **`set_mission_goal()`（新增）**：與 `set_goal_from_command()` 的 `MOVE_POINT` 位置分支相同，但不改 target yaw；
   建議把兩者共用的位置設定抽成同一個私有函式，避免日後只改一邊。

---

# Part B：錯誤事件與 PX4 健康狀態回報

## B1. 背景

2026-10-30 試飛時無法解鎖，GCS 上也無法判斷飛控狀態。查看目前程式碼：

- PX4 拒絕 ARM、Offboard、Hold 等命令時，`BasicController::vehicle_command_ack_cb()` 與
  `handle_arm_rejection()` 只寫 `RCLCPP_WARN`（例如 `PX4 rejected ARM (result=%u)`），**沒有任何 topic 告知 GCS**。
- VehicleStatus 逾時或間隔過大只寫日誌；AgentState 只有 `px4_state_valid` 一個布林，看不出是「從未收到」、
  「已逾時」還是「收到但預檢未通過」。
- planner 拒絕命令（編隊成員不足、沒有新鮮姿態、Home 未建立）也只寫日誌。
- QGCS 的 Guard 在 `px4_state_valid == false` 時拒絕所有命令，操作人員只看到「狀態無效」。

### 試飛問題的可能根因：PX4 訊息版本不一致（需先處理，不屬於本次新功能）

`Dockerfile` 以 `PX4_MSGS_REF=main` 建置 `px4_msgs`（未鎖定版本），程式依 main 的訊息版本訂閱帶後綴的 topic。
PX4 自 v1.16 起，uXRCE-DDS 對 `MESSAGE_VERSION > 0` 的訊息自動在 topic 名稱加上 `_v<版本>`，版本 0 則不加。
對照 PX4-Autopilot `v1.17.0` 標籤的原始碼（2026-10-30 查閱）：

| PX4 訊息 | v1.17.0 版本 | v1.17.0 實際 topic | Agent 目前訂閱／發布 | 是否相符 |
|----------|--------------|--------------------|----------------------|----------|
| VehicleStatus | 1 | `fmu/out/vehicle_status_v1` | `fmu/out/vehicle_status_v4`（main 為 v4） | ✗ |
| VehicleCommandAck | 0 | `fmu/out/vehicle_command_ack` | `fmu/out/vehicle_command_ack_v1` | ✗ |
| BatteryStatus | 1 | `fmu/out/battery_status_v1` | `fmu/out/battery_status_v1` | ✓ |
| VehicleCommand | 0 | `fmu/in/vehicle_command` | `fmu/in/vehicle_command` | ✓ |

若實機韌體（Morakot v1.17.0）使用 PX4 v1.17.0 的預設 DDS 設定，Agent 會**收不到 VehicleStatus 與 VehicleCommandAck**：
`px4_state_valid` 一直是 false（無法判定飛控狀態），ARM 流程等不到 Offboard 確認與 ack（無法解鎖），
而位置類 topic（版本 0、無後綴）照常有資料。這與試飛現象一致。SITL 若使用較新的 PX4，就不會出現此問題。

請先在實機的 Agent 容器內確認：

```bash
ros2 topic list | grep /uav_1/fmu/
ros2 topic hz /uav_1/fmu/out/vehicle_status_v1
ros2 topic hz /uav_1/fmu/out/vehicle_status_v4
```

修正方向（擇一，需 Agent 開發者決定）：

1. **鎖定版本**：`PX4_MSGS_REF` 改為與實機韌體相同的版本（例如 px4_msgs 的 1.17 分支），SITL 也改用同一版 PX4，
   topic 後綴依該版本調整。最單純、可重現。
2. **使用 PX4 ROS 2 訊息轉換節點**（message translation node）在不同版本之間轉換，Agent 維持單一版本的 `px4_msgs`。
3. 不論選哪一種，`PX4_MSGS_REF=main` 都應改成固定版本，否則不同時間建置的映像會訂閱不同的 topic。

Part B 的健康狀態訊息會把各 PX4 topic 的新鮮度直接顯示在 GCS，之後同類問題不需進容器就能判斷。

## B2. 介面契約

### B2.1 `msg/AgentEvent.msg`（事件，發生時發布一次）

```
# Agent 事件（錯誤、警告、資訊），發布於 /{agent_namespace}/events
# basic_controller_node 與 ego_swarm_planner 都會發布到同一個 topic

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_ERROR=2

uint8 SOURCE_BASIC_CONTROLLER=1
uint8 SOURCE_PLANNER=2
uint8 SOURCE_SUPERVISOR=3

builtin_interfaces/Time stamp
uint32 sequence           # 各來源節點各自單調遞增，供 GCS 偵測遺失與去重
uint8 severity
uint8 source
uint16 code               # 事件代碼，見 B2.3
string command            # 相關命令名稱（例如 ARM、REBOOT_FC），沒有時為空字串
int16 px4_result          # 相關的 PX4 VehicleCommandAck.result；沒有時為 -1
string detail             # 英文補充說明，供日誌與未知代碼顯示
```

### B2.2 `msg/AgentHealth.msg`（狀態，週期發布）

```
# Agent 與 PX4 連線健康狀態，basic_controller_node 以 health_publish_rate_hz（預設 1 Hz）發布於 /{agent_namespace}/health
# *_age_s 為距離最後一筆訊息的秒數；從未收到為 -1

uint32 FAILSAFE_LOCAL_POSITION_INVALID=1
uint32 FAILSAFE_GLOBAL_POSITION_INVALID=2
uint32 FAILSAFE_HOME_POSITION_INVALID=4
uint32 FAILSAFE_ATTITUDE_INVALID=8
uint32 FAILSAFE_MANUAL_CONTROL_LOST=16
uint32 FAILSAFE_OFFBOARD_SIGNAL_LOST=32
uint32 FAILSAFE_BATTERY_WARNING=64
uint32 FAILSAFE_BATTERY_UNHEALTHY=128
uint32 FAILSAFE_GEOFENCE_BREACHED=256
uint32 FAILSAFE_FD_CRITICAL_FAILURE=512
uint32 FAILSAFE_FD_ESC_ARMING_FAILURE=1024
uint32 FAILSAFE_LOCAL_POSITION_ACCURACY_LOW=2048

builtin_interfaces/Time stamp
float32 px4_status_age_s              # VehicleStatus
float32 px4_odometry_age_s            # VehicleOdometry
float32 px4_global_position_age_s     # VehicleGlobalPosition
float32 px4_land_detected_age_s       # VehicleLandDetected
float32 px4_battery_age_s             # BatteryStatus
float32 px4_command_ack_age_s         # VehicleCommandAck；平常沒有命令時會很大，-1 表示從未收到
bool px4_pre_flight_checks_pass       # 以下 px4_* 欄位只在 VehicleStatus 新鮮時可信
bool px4_failsafe
uint8 px4_arming_state
uint8 px4_latest_arming_reason
bool failsafe_flags_valid             # 有收到新鮮的 fmu/out/failsafe_flags 才為 true
uint32 failsafe_flags                 # 依上方 FAILSAFE_* 位元整理
bool offboard_session_active
```

- 欄位只放「判斷能不能飛」需要的內容；完整 PX4 資訊仍以 PX4 log 為準。
- 只使用 PX4 v1.17 與 main 的 VehicleStatus 都有的欄位（`failure_detector_status` 在 main 已移除，故不列入）。
- PX4 v1.17.0 預設 `dds_topics.yaml` 已輸出 `/fmu/out/failsafe_flags`，topic 後綴同樣依鎖定的訊息版本決定；
  沒有收到時 `failsafe_flags_valid = false`，其他欄位照常。
- PX4 的逐項解鎖失敗原因（events／`health_report`）需另行解碼，不在本版範圍；本版以 ack 結果、預檢旗標與 failsafe 位元提供判斷依據。

### B2.3 事件代碼（初版）

| 代碼 | 名稱 | 嚴重度 | 觸發 |
|------|------|--------|------|
| 100 | `COMMAND_UNKNOWN` | WARNING | 無法解析的命令字串 |
| 101 | `COMMAND_INVALID_PARAMS` | WARNING | 參數缺少或不合法（例如 `TAKEOFF abc`、`YAW` 無參數） |
| 102 | `COMMAND_REJECTED` | WARNING | Agent 狀態不允許（未初始化、重新啟動冷卻中、全體 topic 的 `MISSION_*`…），`detail` 寫明原因 |
| 103 | `COMMAND_FAILED` | ERROR | 命令處理失敗（現行 `Command '%s' execution failed`） |
| 200 | `PX4_COMMAND_REJECTED` | ERROR | VehicleCommandAck 不是 ACCEPTED／IN_PROGRESS；`command` 填 ARM、DISARM、MODE_OFFBOARD、HOLD、MODE_RTL、MODE_LAND、REBOOT_FC，`px4_result` 填結果 |
| 201 | `PX4_COMMAND_TIMEOUT` | ERROR | 送出後 `px4_command_ack_timeout_s`（預設 3 秒）內沒有 ack，或 Offboard 等待確認逾時 |
| 202 | `PX4_COMMAND_ACCEPTED` | INFO | 只用於 ARM、DISARM、REBOOT_FC，讓 GCS 確認真正生效 |
| 203 | `TAKEOFF_CANCELLED` | WARNING | 排隊中的 TAKEOFF 因等待解鎖逾時取消 |
| 204 | `OFFBOARD_LOST` | WARNING | PX4 離開 Offboard（failsafe 或遙控器切換） |
| 300 | `PX4_STATUS_STALE` | ERROR | VehicleStatus 從未收到或超過 `PX4_STATE_TIMEOUT_MS`（狀態轉換時發布一次） |
| 301 | `PX4_STATUS_RECOVERED` | INFO | VehicleStatus 恢復 |
| 302 | `PX4_POSITION_STALE` | WARNING | odometry 或 global position 逾時（`detail` 標明哪一個） |
| 303 | `PX4_PREFLIGHT_FAILED` | WARNING | `pre_flight_checks_pass` 由 true 轉 false，或收到 ARM 時為 false |
| 304 | `PX4_FAILSAFE` | WARNING | `failsafe` 由 false 轉 true |
| 305 | `PX4_RESTART_DETECTED` | INFO | 偵測到 PX4 重新連線或重新啟動（見 C3） |
| 400 | `PLANNER_COMMAND_REJECTED` | WARNING | 編隊成員不足、幾何不安全、沒有新鮮姿態、Home 未建立等 |
| 401 | `PLANNER_EMERGENCY_BRAKE` | WARNING | planner 啟動緊急煞停 |
| 402 | `PLANNER_FAILURE_PERSISTENT` | ERROR | 連續規劃失敗超過 `planner_failure_event_s`（預設 3 秒） |
| 500 | `SAFETY_POSITION_DEVIATION` | ERROR | PositionSafetyMonitor 觸發 |

- 代碼只增不改；GCS 對未知代碼顯示 `detail`。
- 同一代碼與命令 5 秒內只發布一次；狀態型事件（300／301、303、304）只在轉換時發布。
- 現有對應的 `RCLCPP_WARN`／`RCLCPP_ERROR` 保留，事件是額外發布。

### B2.4 Topic 與 QoS

| Topic | 方向 | 型別 | QoS |
|-------|------|------|-----|
| `/{agent_namespace}/events` | Agent → GCS | `taiphoon_agent/AgentEvent` | reliable、volatile、depth 20 |
| `/{agent_namespace}/health` | Agent → GCS | `taiphoon_agent/AgentHealth` | reliable、volatile、depth 1 |

- 事件不用 transient_local：GCS 連上前的舊事件不重播，避免操作人員把舊錯誤當成新錯誤；持續性的問題由 `health` 週期呈現。
- `health` 在 PX4 完全沒有資料時也要照常發布（age 為 -1），這正是試飛時需要看到的資訊。

## B3. 實作規劃

| 檔案 | 修改 |
|------|------|
| `msg/AgentEvent.msg`、`msg/AgentHealth.msg` | 新增 |
| `src/safety/AgentEventCodes.hpp`（新增，純邏輯） | 代碼、嚴重度、節流規則（同代碼 5 秒）、狀態轉換偵測 |
| `src/controllers/BasicController.hpp/.cpp` | 在 ack callback、`handle_arm_rejection()`、Offboard 逾時、排隊起飛取消、VehicleStatus 逾時等處呼叫事件回呼；保存各 PX4 topic 最後接收時間與 VehicleStatus 摘要 |
| `src/controllers/BasicController.cpp` | 新增 `fmu/out/failsafe_flags` 訂閱（best_effort）；topic 不存在時不影響其他功能 |
| `src/nodes/basic_controller_node.cpp` | `events`、`health` 發布者；命令解析失敗與執行失敗發布事件；`health` 定時器 |
| `src/nodes/ego_swarm_planner_node.cpp` | 現有 `Rejecting ...` 與緊急煞停處發布事件 |
| `src/nodes/AgentSupervisor.cpp` | shutdown 請求時發布 INFO 事件（選配） |

- BasicController 是純邏輯控制器，事件以注入的回呼（`std::function`）送出，不在 controller 內直接建立 ROS2 publisher，
  維持 `controllers` 與 `nodes` 的依賴方向。
- 事件發布必須在 ROS2 callback／timer 執行緒，不在 PX4 callback 中做耗時工作。

---

# Part C：重新啟動飛控命令（`REBOOT_FC`）

## C1. 介面

| 命令字串 | 參數 | 說明 |
|----------|------|------|
| `REBOOT_FC [force]` | 選填，`0` 或 `1`（預設 0） | 送出 PX4 `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`（246），`param1 = 1` 重新啟動飛控、`param2 = 0` 不影響機上電腦 |

- 只接受 `/{agent_namespace}/command`；`/uav_all_agent/command` 的 `REBOOT_FC` 忽略並發布 `COMMAND_REJECTED`（與 shutdown service 不提供廣播相同）。
- 這是重新啟動**飛控**；Agent 容器的 shutdown service（`/{agent_namespace}/shutdown`）用途不同，兩者不可混用。
- 結果以 Part B 事件回報：`PX4_COMMAND_ACCEPTED`／`PX4_COMMAND_REJECTED`／`PX4_COMMAND_TIMEOUT`（`command = REBOOT_FC`）。

## C2. 安全條件（BasicController）

| PX4 狀態 | `force = 0` | `force = 1` |
|----------|-------------|-------------|
| VehicleStatus 新鮮、已上鎖、land detected 新鮮且已落地 | 送出 | 送出 |
| VehicleStatus 新鮮、已解鎖 | 拒絕 | **拒絕** |
| VehicleStatus 新鮮、已上鎖但未確認落地 | 拒絕 | 拒絕 |
| VehicleStatus 從未收到或逾時（狀態不明） | 拒絕，`detail` 提示可用 force | 送出（依賴 PX4 自身在解鎖時拒絕重新啟動） |

- 任何拒絕都發布 `COMMAND_REJECTED`，`detail` 說明是哪一條。
- 送出前停止 Offboard session、清除排隊中的 ARM／TAKEOFF 與 timed trajectory，flight state 回到 IDLE。
- 冷卻：送出後 `reboot_cooldown_s`（預設 15 秒）內的 `REBOOT_FC` 一律拒絕，避免重複送出。
- 依 PX4 Commander 的行為，飛控在解鎖時會以 DENIED 拒絕重新啟動；這是 `force` 的安全前提，**必須在 SITL 與實機各驗證一次**，
  若實機韌體不拒絕，`force` 改為不提供。
- ack 逾時可能代表 PX4 已開始重新啟動來不及回覆，或連線本來就斷了；事件 `detail` 要寫明「未收到 ack，請觀察 health 是否恢復」。

## C3. 飛控重新啟動後的 Agent 狀態

PX4 重新啟動（不論是 `REBOOT_FC` 或斷電）後 EKF 原點與 local NED 可能改變，Agent 需重建依賴它的狀態：

- 偵測：VehicleStatus 逾時後恢復且 PX4 為上鎖，或 `REBOOT_FC` 已 ACCEPTED 後狀態恢復 → 發布 `PX4_RESTART_DETECTED`。
- BasicController：重設 Offboard session、鎖定的 setpoint 與航向、排隊命令，flight state 回到 IDLE。
- planner：清除任務與編隊（沿用 `clear_grounded_mission()`），並**重建 `planner_to_px4_ned_offset_`**；
  目前這個平移只在第一次建立後就不再更新，PX4 原點改變後到達判斷與航點高度換算都會錯。
  是否同時重新凍結 planner Home 由 Agent 開發者評估（飛機若未移動，保留 Home 即可）。

---

# 共同部分

## D1. 介面總表（本次新增）

| 類別 | 名稱 | 方向 | QoS |
|------|------|------|-----|
| Topic | `/{ns}/mission/plan`（`WaypointMission`） | GCS → Agent | reliable、volatile、10 |
| Topic | `/{ns}/mission/status`（`WaypointMissionStatus`） | Agent → GCS | reliable、transient_local、1 |
| Topic | `/{ns}/events`（`AgentEvent`） | Agent → GCS | reliable、volatile、20 |
| Topic | `/{ns}/health`（`AgentHealth`） | Agent → GCS | reliable、volatile、1 |
| 命令 | `MISSION_START <id>`、`MISSION_STOP`、`REBOOT_FC [force]` | GCS → Agent | 既有 command topic |
| 訊息 | `MissionWaypoint`、`WaypointMission`、`WaypointMissionStatus`、`AgentEvent`、`AgentHealth` | — | — |

## D2. 單元測試（`test/`，gtest）

- `test_waypoint_mission_executor.cpp`
  - 驗證：航點數 0／1／50／51、高度 1／120 邊界、`dwell_s` 邊界、非 NaN `yaw_deg`、`mission_id == 0`、錯誤 `altitude_frame`、超出距離上限；
    `current_index` 指向第一個不合法航點。
  - 識別碼：未載入就 START、id 不符、載入逾時後 START → `REASON_ID_MISMATCH`；新上傳取代 LOADED。
  - 執行中上傳 → `REASON_BUSY`，原任務狀態不變。
  - 進度：到達 → DWELLING → 停留結束才給下一段 → … → COMPLETED；`dwell_s = 0` 立即下一段；最後一點不停留。
  - 到達事件的目標與本段不同時忽略。
  - 逾時、STOP、COMMAND_OVERRIDE、FLIGHT_STATE、LANDED 的原因碼與 active 狀態。
- 座標換算（抽成可測的純函式）：planner 原點與 PX4 local 原點**不同**時，換算後的 PX4 local 目標水平與高度都正確；
  `offset` 為 0 時與現行 QGCS 換算一致；`offset` 重建後換算跟著更新。
- `test_agent_event_codes.cpp`：節流（同代碼 5 秒）、狀態轉換只發布一次、未知 ack 結果值的處理。
- `REBOOT_FC` 安全條件：C2 表格每一格（狀態新鮮／逾時／從未收到 × 解鎖／上鎖 × 落地／未落地 × force 0／1），以及冷卻期間。
- `CommandTypes`：`MISSION_START 123`、`MISSION_START 4294967295`、`MISSION_START 1.5`、`MISSION_START 0`、缺參數、`MISSION_STOP`、
  `REBOOT_FC`、`REBOOT_FC 1`、`REBOOT_FC 2`（拒絕）。
- 若有 node 層測試基礎：全體 topic 的 `MISSION_*`、`REBOOT_FC` 被忽略並發布事件。

## D3. SITL 與實機驗證

| # | 情境 | 通過條件 |
|---|------|----------|
| 1 | 單機起飛 8 m，上傳 3 航點後 START | 依序到達（PX4 local 誤差 ≤ 0.5 m）、各停留約 3 秒、COMPLETED 後懸停於最後一點 |
| 2 | 任務中途 `MISSION_STOP` | 不再飛往原航點、不飛回，停下懸停；ABORTED／OPERATOR_STOP |
| 3 | 任務中途 `HOLD` | 同上，原因 COMMAND_OVERRIDE |
| 4 | 任務中途 `MOVE_FORWARD 3` | 任務中止，並執行該移動 |
| 5 | 任務中途 `MODE_RTL` | ABORTED／FLIGHT_STATE，RTL 正常完成 |
| 6 | 空中重啟 Agent 後再上傳並執行 | 到達的 WGS84 位置與航點一致 |
| 7 | 兩機航線交叉 | 兩機避碰後都完成，或逾時中止且停在原地 |
| 8 | `east_wall` 障礙物場景中的航點 | 繞過障礙物到達 |
| 9 | 落地時上傳、編隊中上傳、距離 1.2 km 的航點 | 對應 REJECTED 原因碼，無人機不移動 |
| 10 | 任務中關閉 QGCS（或切斷網路）再重開 | 任務繼續完成；QGCS 重開後立即從 transient_local 取得狀態 |
| 11 | SITL 以參數製造預檢失敗（例如要求遙控器連線）後 ARM | 收到 `PX4_PREFLIGHT_FAILED` 與 `PX4_COMMAND_REJECTED`（ARM，含結果碼）；health 的 `px4_pre_flight_checks_pass = false` |
| 12 | 停止 PX4（或擋住 uXRCE-DDS） | 5 秒內 `PX4_STATUS_STALE`，health 的 age 持續增加；恢復後 `PX4_STATUS_RECOVERED` |
| 13 | 落地上鎖時 `REBOOT_FC` | `PX4_COMMAND_ACCEPTED`，PX4 重新啟動後 `PX4_RESTART_DETECTED`，可再次解鎖起飛，且航點任務高度正確 |
| 14 | 懸停時 `REBOOT_FC` 與 `REBOOT_FC 1` | 兩者都被 Agent 拒絕，飛行不受影響 |
| 15 | 解鎖狀態下擋住 VehicleStatus 後送 `REBOOT_FC 1`（SITL） | PX4 回 DENIED，飛控不重新啟動；確認 force 的安全前提成立 |
| 16 | 實機：對試飛時的機體重現無法解鎖 | GCS 能看到原因（topic 缺漏、預檢未通過或 ack 結果） |

## D4. 文件與發布

- `docs/taiphoon-agent-interface.md`：第 3 節新增五個訊息、第 4.2 節新增四個 Topic 與資料流、第 6 節新增航點任務與
  `REBOOT_FC` 命令、第 8 節 QoS 表、新增事件代碼表；版本號遞增。
- `docs/agent-commands.md`：新增命令與範例：
  ```bash
  ros2 topic pub --once /uav_1_agent/mission/plan taiphoon_agent/msg/WaypointMission \
    "{mission_id: 7, altitude_frame: 1, waypoints: [{latitude_deg: 24.1, longitude_deg: 120.6, altitude_m: 8.0, dwell_s: 3.0, yaw_deg: .nan}]}"
  ros2 topic pub --once /uav_1_agent/command std_msgs/msg/String "data: 'MISSION_START 7'"
  ros2 topic echo /uav_1_agent/mission/status
  ros2 topic echo /uav_1_agent/events
  ros2 topic echo /uav_1_agent/health
  ros2 topic pub --once /uav_1_agent/command std_msgs/msg/String "data: 'REBOOT_FC'"
  ```
- `docs/basic_controller_manual.md`：事件、health 與重新啟動流程。
- 依 Agent `AGENTS.md`：`msg/` 與介面文件同步到 `../Taiphoon_Agent_Interfaces/`。
- README（使用者）：航點任務由 Agent 執行且 GCS 斷線時會繼續、錯誤會顯示在 GCS、如何重新啟動飛控。
- 版本：新功能，建議 v1.4.0；發布說明註明需搭配 QGCS 1.3.0。

## D5. QGCS 端對應行為（供 Agent 端理解，不需實作）

1. 航點任務：操作人員確認後，QGCS 產生隨機非 0 的 `mission_id`，所有航點 `dwell_s = 3`、`yaw_deg = NaN`、
   高度基準 `ALTITUDE_PX4_LOCAL`；發布 `mission/plan`，3 秒內等到同一 `mission_id` 的 LOADED 後，經 GuardModule 送出
   `MISSION_START <id>`。進度完全依 `mission/status` 顯示；停止送 `MISSION_STOP`。
2. 事件：WARNING 以上顯示提示並寫入 QGCS 日誌；各機保留最近的事件清單。
3. health：機群面板顯示飛控連線與預檢狀態（例如「VehicleStatus 未收到」、「預檢未通過」、「Failsafe」）。
4. `REBOOT_FC`：狀態有效時只允許在上鎖且落地時送出（兩步驟確認）；狀態不明時只能以 `force = 1` 送出，
   且需操作人員以滑桿確認。

## D6. 建議實作順序

0. **先修正 B1 的 PX4 訊息版本不一致**（鎖定 `px4_msgs` 與 SITL 版本或加入轉換節點），並在實機確認能解鎖。
1. 與 QGCS 開發者定稿 D1 的介面契約，提交 `msg/` 與介面文件，同步到 `Taiphoon_Agent_Interfaces`。
2. **Part B 優先**：事件與 health（直接解決試飛無法判斷的問題，也方便後續兩部分除錯）。
3. Part C：`REBOOT_FC` 與 PX4 重新啟動後的狀態重建。
4. Part A：`CommandTypes`、`WaypointMissionExecutor`、planner 掛載。
5. SITL 情境 1～15，實機情境 16，日誌留在 `AutoTest/Log/`。
6. 文件、README、發布說明。

QGCS 端可在第 1 步定稿後以 FakeAgentLink 平行開發，第 5 步時一起做端對端驗證。

## D7. 仍待確認的事項

| 項目 | 本文件的預設 | 替代做法 |
|------|--------------|----------|
| PX4 訊息版本 | 鎖定 `px4_msgs` 與 SITL 為實機韌體版本 | 使用 PX4 訊息轉換節點 |
| 實機韌體輸出 `failsafe_flags` | 若沒有，`failsafe_flags_valid = false` | 請韌體在 `dds_topics.yaml` 加入 |
| 高度基準 | PX4 local（與 QGCS 面板高度一致） | WGS84 AMSL：全球唯一，但受 GPS 高度雜訊影響，室內 UWB 場地不適用 |
| 航點停下或通過 | 每點都停下 | 不停留通過需調整到達判斷與 Local Goal，另案處理 |
| 載入逾時 | 10 秒 | 不逾時，保留到下一次上傳 |

---

# Agent 端實作決策（2026-10-30）

> 以下為 Agent 開發時依程式碼現況補充的決策，與上文衝突時以本節為準；涉及安全行為者已在交付時告知人類開發者。

## E1. 資料流

```
GCS ──mission/plan──▶ ego_swarm_planner ── 驗證／WGS84 換算 ──▶ WaypointMissionExecutor（純邏輯）
                         │  ▲                                     │ due_target／逾時
                         │  └── planner_loop_cb：到達 → on_target_reached
                         ├──mission/status（transient_local）──▶ GCS
                         └──events（SOURCE_PLANNER）──────────▶ GCS

GCS ──command──▶ AgentCommandHandler ──▶ basic_controller_node ──▶ BasicController ──▶ PX4
                                              │  ▲ 事件回呼（控制迴圈執行緒）
                                              ├──events（SOURCE_BASIC_CONTROLLER）──▶ GCS、planner（只取 PX4_RESTART_DETECTED）
                                              └──health（1 Hz）────────────────────▶ GCS
```

## E2. 決策

| # | 項目 | 決定與理由 |
|---|------|------------|
| 1 | 其他命令對 LOADED 任務的影響 | 只有 RUNNING／DWELLING 會因介入命令、飛行狀態或逾時而中止；LOADED 只會因載入逾時、`MISSION_STOP`、新上傳或落地清除，避免 QGCS 上傳到開始之間的其他命令誤刪任務 |
| 2 | PX4 離開 Offboard | 開始條件與執行中都要求 `px4_nav_state == OFFBOARD`；遙控器接手（例如切 Position）時 AgentState 仍可能顯示 HOVER，因此以 `REASON_FLIGHT_STATE` 中止 |
| 3 | FLIGHT_STATE 中止 | 不發布本機停止軌跡（控制權已在 PX4），但清除 planner 目標，控制權回到 Agent 時不會再飛往原航點 |
| 4 | 停在目前位置 | 以 `EmergencyBrakePlanner` 依目前速度產生煞停軌跡並發布到本機與同儕；煞停終點作為之後相對 `MOVE_*` 的累加基準 |
| 5 | 經 command topic 的 `MODE_RTL`／`MODE_LAND` | 依 A3.5 回報 `COMMAND_OVERRIDE`（`detail` 為命令名稱）；D3 情境 5 的 `FLIGHT_STATE` 只在 PX4 自行切換（failsafe、遙控器）時出現 |
| 6 | 參數不合法的命令 | 例如 `TAKEOFF abc`、`YAW` 無參數：BasicController 不執行並發布 `COMMAND_INVALID_PARAMS`（舊版 `TAKEOFF abc` 會以預設高度起飛）；planner 也不視為介入 |
| 7 | 事件的唯一發布者 | 命令類事件（100～103、全體 topic 的 `MISSION_*`／`REBOOT_FC` 拒絕）由 basic_controller_node 發布；planner 只發布 400～402，避免同一命令產生兩筆事件 |
| 8 | PX4 重新啟動 | BasicController 偵測後發布 `PX4_RESTART_DETECTED`；planner 訂閱同一 `events` topic，收到後清除任務與編隊並重建 `planner_to_px4_ned_offset_`，保留 planner Home；basic_controller_node 也重建 WGS84→PX4 local 的軌跡平移（同一問題，指引未列出） |
| 9 | `SAFETY_POSITION_DEVIATION`（500） | PositionSafetyMonitor 的介入目前在控制迴圈停用，代碼保留但暫不發布 |
| 10 | `PX4_POSITION_STALE` 門檻 | 1 秒，與 `CONTEXT.md` 的沉默失效定義一致；PX4 啟動後 5 秒內未收到資料才視為從未收到 |
| 11 | AgentSupervisor shutdown 事件（選配） | 本版不做 |
| 12 | PX4 訊息版本（D6 第 0 步） | 人類開發者決定維持目前版本（2026-10-30）：`PX4_MSGS_REF=main` 與現行 topic 名稱不變，不鎖定版本也不加轉換節點；B1 的版本不一致改由 `health` 的資料時間與事件 300 判斷 |
| 13 | SITL 的 `REBOOT_FC` | PX4 v1.17.0 Commander 的重新啟動分支只在定義 `CONFIG_BOARDCTL_RESET`（NuttX）時編入，且要求未解鎖；POSIX SITL 一律回 DENIED。D3 #13 的實際重新啟動與 #15 改為只在實機驗證，SITL 以重新啟動 PX4 容器驗證 301／305（2026-10-30 QGCS 整合測試回報） |
| 14 | 沒有 odometry 時的 AgentState | 維持不發布：AgentState 的 `position_ned` 會被 planner（Home、目前位置）、同儕避碰與障礙物模擬直接使用，發布未定義位置的風險大於收益；GCS 改以固定發布的 `health` 判斷 Agent 在線，`health.px4_arming_state`（0=從未收到）判斷最後已知的解鎖狀態 |
| 15 | `PX4_PREFLIGHT_FAILED`（303）去抖動 | 人類開發者決定維持現狀（2026-10-30）：不去抖動。SITL 驗證時 PX4 降落自動上鎖的瞬間 `pre_flight_checks_pass` 短暫為 false，會出現一筆 303；GCS 以 `health.px4_pre_flight_checks_pass` 確認是否持續 |
