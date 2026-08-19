# Step 1 — 現行 EGO-Swarm 資料流分析

## 文件定位

本文件是 `Local Goal + Generic Trajectory Reservation` 第一版開發的 Step 1
分析結果，提供後續實作查閱目前架構與變更接入位置。

本文件是目前程式碼的架構快照，不取代正式開發規格。功能需求、禁止修改項目
與最終驗收條件仍以
[`docs/taiphoon-agent-development-spec.md`](../taiphoon-agent-development-spec.md)
為準。

分析基準：

- Branch：`EGO-Swarm-dynamic-dev`
- Commit：`bfdaa8a`
- 分析日期：2026-08-19
- 工作樹：乾淨
- 本次結果：只新增本參考文件，尚未修改 runtime 程式碼

## 一、分析範圍

Step 1 規定的核心分析對象：

- `EgoSwarmPlannerNode::planner_loop_cb()`
- `get_current_target()`
- `get_formation_target_for_agent()`
- `update_formation_assignment()`
- `evaluate_target_arrival()`
- `evaluate_formation_convergence()`
- `FormationReservationPlanner`
- `TrajectoryPriority`
- `ContinuousSafetyValidator`
- `OptimizedEgoPlanner`

同時追蹤以下執行邊界：

- `PeerTrajectoryTable`
- `BasicControllerNode::agent_trajectory_cb()`
- `BasicController`
- `launch/basic_controller.launch.py`
- `docs/taiphoon-agent-interface.md`

## 二、現行架構結論

目前的 runtime 資料流是：

```text
MOVE / MOVE_POINT / mission intent
                  │
                  ▼
       EgoSwarmPlannerNode::set_goal_from_command()
                  │
                  ├── MOVE：goal_position_
                  └── Formation：FormationEpoch → assignment → slot
                                      │
                                      ▼
                  get_current_target() → target
                                      │
                                      ▼
                     PlanningSnapshot.local_goal = target
                                      │
                                      ▼
                         OptimizedEgoPlanner
                                      │
                                      ▼
                         candidate B-spline
                                      │
                    Formation only: Reservation
                                      │
                                      ▼
                       ContinuousSafetyValidator
                                      │
                                      ▼
                             active_plan_
                                      │
                                      ▼
                            AgentTrajectory
                                      │
                                      ▼
                             BasicController
                                      │
                                      ▼
                          PX4 TrajectorySetpoint
```

目前尚未存在：

- `LocalGoalSelector`
- `planning_target` 狀態變數
- `active_plan_final_target_`
- Local Goal 或 Generic Reservation feature flag
- progress-based segment replan policy

因此目前 `get_current_target()` 回傳的 `target` 同時被當成：

1. 任務真正終點。
2. 本輪 planner 的 trajectory endpoint。
3. Active plan 的 target 比較基準。
4. Arrival 與 Terminal Hold 的目標。

這正是後續 Step 2～Step 7 必須拆開的核心耦合點。

## 三、對外資料流與執行元件

### 3.1 輸入來源

| 來源 | Topic／觸發 | 目前用途 | 主要接收位置 |
|---|---|---|---|
| 本機 Agent 狀態 | `/{agent_namespace}/state` | WGS84 位置、飛行狀態與自身定位 | `own_state_cb()` |
| 本機姿態 | `/{agent_namespace}/attitude` | 初始化及保存水平 MOVE 的 target yaw | `own_attitude_cb()` |
| 本機命令 | `/{agent_namespace}/command` | MOVE、MOVE_POINT、YAW、FORMATION_CANCEL | `agent_command_cb()` |
| 任務意圖 | `/{agent_namespace}/planning/mission_intent` | 以文字輸入 planner 任務 | `mission_intent_cb()` |
| 舊目標介面 | `/{agent_namespace}/ned_target` | 轉換為 MOVE_POINT 相容輸入 | `ned_target_cb()` |
| 全體命令 | `/fmu_all_agent/command` | Formation 與全體取消編隊 | `all_agent_command_cb()` |
| Formation epoch | `/fmu_all_agent/swarm/formation_epochs` | proposal、adopted 與成員 snapshot | `formation_epoch_cb()` |
| 同儕狀態 | `/{peer_namespace}/state` | 同儕 WGS84 位置、discovery 與 live distance | `peer_state_cb()` |
| 同儕軌跡 | `/fmu_all_agent/swarm/agent_trajectories` | 同儕 future occupancy 與 reservation／safety | `trajectory_cb()` |
| 障礙物 | `/{agent_namespace}/perception/obstacles` | `RawPointObstacleMap` 更新 | `obstacle_cb()` |
| planner timer | 目前預設 10 Hz | 驅動資料檢查、規劃、發布與 recovery | `planner_loop_cb()` |
| peer scan timer | 目前每 3 秒 | 動態發現與移除同儕 state subscription | `scan_peers_cb()` |

`basic_controller_node` 是唯一允許發布 PX4
`TrajectorySetpoint` 的 runtime 節點；EGO-Swarm planner 只發布時間化
`AgentTrajectory`。

### 3.2 輸出來源

`publish_plan()` 將純 C++ B-spline 轉換成 WGS84 `AgentTrajectory`，並同步發布：

- `/fmu_all_agent/swarm/agent_trajectories`：供其他 Agent 使用。
- `/{agent_namespace}/planning/trajectory`：供本機 BasicController 使用。
- `/{agent_namespace}/planning/status`：供診斷與驗證使用。

發布內容包含：

- `start_time`
- `trajectory_id`
- `frame_revision`
- `duration_s`
- `terminal_hold`
- WGS84 control points

Reservation 不新增 ROS2 message；目前透過修改候選 trajectory 的 accepted
start time，再由 `publish_plan()` 寫入訊息。

## 四、任務目標資料流

### 4.1 MOVE 與 MOVE_POINT

`agent_command_cb()`、`mission_intent_cb()` 與 `ned_target_cb()` 最終都會進入
`set_goal_from_command()`。

目前行為：

1. `MOVE_POINT` 直接設定 `goal_position_`，並標記 `has_goal_` 與
   `has_position_target_`。
2. 相對水平命令以保存的 `target_yaw_rad_` 將 body-frame 位移轉成 NED，
   再累加到 `goal_position_`。
3. `MOVE_UP`／`MOVE_DOWN` 直接調整 `goal_position_.z`。
4. 新目標建立後呼叫 `invalidate_active_plan()`。
5. `preserve_altitude_` 記錄水平 MOVE 的 `FIXED_Z` 語意。

因此 MOVE 的 Final Target 目前實際上是：

```text
goal_position_
```

但程式中尚未使用 `final_target` 這個明確名稱。

### 4.2 Formation

Formation 的資料流分成兩層：

```text
FORMATION_* 命令
      ↓
formation_config_
      ↓
FormationEpoch proposal / adopted
      ↓
update_formation_assignment()
      ↓
formation_assignment_ + formation_center_
      ↓
get_formation_target_for_agent(agent_index_)
      ↓
assigned slot target
```

目前 `update_formation_assignment()` 會：

- 讀取 accepted `FormationEpoch` 的成員與 WGS84 snapshot。
- 將成員位置轉換至共同 `swarm_ned`。
- 使用既有 assignment、slot switch penalty 與 crossing penalty。
- 更新 `formation_assignment_` 與 `formation_center_`。

後續 Local Goal 不得插入此流程中間。正確位置是：

```text
FormationEpoch
    ↓
Slot Assignment
    ↓
Final Target = assigned slot
    ↓
Local Goal
```

### 4.3 Formation Cancel

`FORMATION_CANCEL` 會先嘗試保存目前 assigned slot target 到
`goal_position_`，再清除 Formation 狀態並 invalidate active plan。

這代表取消編隊後的 MOVE 相對命令具有「沿用原 slot target」的相容語意，後續
不可因導入 Local Goal 而改成沿用中間 Local Goal。

## 五、目前 `planner_loop_cb()` 的資料流

### 5.1 前置檢查與同步

每個 planner cycle 目前依序執行：

1. 取得 `now`／`now_s`；時間無效時發布 HOLD。
2. 週期發布候選 FormationEpoch heartbeat。
3. 以 dynamic discovery 結果建立 `expected_peer_ids`。
4. 呼叫 `PeerTrajectoryTable::all_fresh()`，確認預期同儕均有 ACTIVE 或
   TERMINAL_HOLD 資料。
5. 確認本機 AgentState 與 WGS84 local Home 可用。
6. 若 Formation 命令早於 local Home 到達，保留 pending command 並重試 proposal。

這層的原則是 fail-closed：已發現但尚未提供有效 occupancy 的 peer 不可被當成
「沒有同儕」。

### 5.2 無任務狀態

當 `!has_goal_ && !has_formation_` 時：

- 若 emergency brake 尚未完成，持續發布同一條 brake trajectory。
- brake 完成後建立 terminal hold。
- 待命中的 Agent 也建立並發布 terminal hold heartbeat。
- terminal hold 會以目前實際位置或已完成目標位置維持 occupancy。

此分支不能被 Local Goal 改寫，因為它處理的是實際控制狀態與 peer occupancy，
不是任務規劃目標。

### 5.3 取得目前任務目標

目前流程為：

```text
get_position_in_swarm_frame(own_state_)
        ↓
make_planning_state()
        ↓
get_current_target(target)
```

`get_current_target()` 的目前規則：

- `has_formation_ == true`：回傳本機 assigned formation slot。
- 否則：回傳有限的 `goal_position_`。
- 無有效目標：回傳 false，進入 emergency brake／recovery。

在後續實作中，這個 `target` 必須重新命名或明確標記為 `final_target`；
不能直接把 Local Goal 塞回同一個語意。

### 5.4 Live peer safety gate

planner 不只依賴 peer future trajectory，也會用即時 WGS84 state 計算
`live_peer_min_distance()`。

目前會：

- 低於 `max(formation_min_distance_m_, peer_collision_min_distance_m)` 時
  立即停止既有 route，並依速度進入 waiting hold 或 emergency brake。
- 進入 live warning buffer 且沒有有效 active route 時，可能先 emergency brake。

這是即時資料的 fail-closed safety gate，後續 Local Goal 不能取代或繞過它。

### 5.5 Formation membership、arrival 與 Terminal Hold

Formation 成員集合變更尚未穩定時：

- invalidate active plan。
- 在目前位置建立 hold。
- 等待新成員集合穩定後再繼續。

接著以 `evaluate_target_arrival(current, target)` 判斷到達：

- 一般 MOVE 要求 local/global arrival contract。
- Formation 可使用共同 global NED 的 arrival tolerance。
- 到達 Formation slot 後仍保留 `has_formation_`。
- Terminal hold 位置鎖定目前 `target`，也就是 assigned slot。
- Formation convergence 另外由 `evaluate_formation_convergence()` 判斷。

後續必須維持：

```text
Local Goal reached ≠ Final Target reached
```

因此 Local Goal 不得觸發這一段的 task completion 或 Terminal Hold。

### 5.6 Peer freshness 與 obstacle freshness

若 peer 資料不是 fresh：

- Formation initial plan 使用 waiting hold。
- 其他情況進入 emergency brake。

若 obstacle map 不是 `Fresh`：

- 進入 emergency brake。

後續 candidate planning 仍必須使用相同 freshness contract；Local Goal 只改變
planner 的 endpoint，不降低輸入資料有效性要求。

### 5.7 PathConstraint

目前依任務狀態建立：

| 條件 | PathConstraint | Z 語意 |
|---|---|---|
| 水平 MOVE 或非 VERT Formation | `FIXED_Z` | 以 Final Target Z 固定規劃高度 |
| VERT_LINEAR Formation | `LINEAR_Z` | 由起點至目標沿路徑線性變化 |
| 其他自由 3D 任務 | `FREE_3D` | 允許 planner 原有 3D 行為 |

在 `FIXED_Z` 下目前會先將 planning state 的 current Z 設為 target Z，再送入
planner。這個順序是 Local Goal 演算法需要保留的語意：應先進入 planning space，
再計算 horizon。

### 5.8 Active plan reuse

目前 active plan reuse 條件為：

```text
has_active_plan_
∧ distance(active_plan_target_, target) <= 1e-3
∧ trajectory valid
∧ active elapsed <= duration
```

若條件成立，會：

1. 用 `ContinuousSafetyValidator` 重新檢查整條 active trajectory。
2. 安全時保留原本 `start_time` 與 `trajectory_id` 發布，不重設 BasicController 進度。
3. 若 peer／obstacle collision，invalidate 後重新規劃。
4. 其他 safety failure 則進 emergency brake。

這表示目前已具備「不要每個 planner tick 重建同一條 trajectory」的基本機制，
但它比較的是單一 `active_plan_target_`，尚未能表達：

```text
active task final target 不變，但下一個 local segment 已可規劃
```

後續需加入 progress threshold 與 active final target 比較，不能只把
`active_plan_target_` 改成新的 local goal 就結束。

### 5.9 建立 PlanningSnapshot

目前的 snapshot 建立方式：

```text
plan_start_delay_s = has_formation_ ? reservation_lead_time_s_ : 0.0
plan_start_time = now + plan_start_delay_s

snapshot.self_state = planning_state
snapshot.local_goal = target
snapshot.planning_time_s = now_s
snapshot.trajectory_start_time_s = plan_start_time_s
snapshot.frame_revision = frame_revision_
snapshot.self_agent_id = agent_index_
snapshot.path_constraint = path_constraint
snapshot.fixed_z_m = target.z
snapshot.obstacle_map = &obstacle_map_
snapshot.peers_are_fresh = peer_data_valid
snapshot.peers = peer_trajectories_.get_active(...)
```

後續唯一預期的 endpoint 改動是：

```text
snapshot.local_goal = planning_target
snapshot.fixed_z_m = planning_target.z 或明確保留 Final Target Z 語意
```

其中 `OptimizedEgoPlanner` 本身不應知道 MOVE、Formation 或 Final Target。

## 六、Planner、Reservation、Safety 的實際邊界

### 6.1 OptimizedEgoPlanner

`OptimizedEgoPlanner::plan()` 只接收 `PlanningSnapshot`，目前直接使用：

- `snapshot.self_state.position` 作為 trajectory 起點。
- `snapshot.local_goal` 作為 trajectory 終點。
- `snapshot.path_constraint` 控制 control point 的 Z 行為。
- peer 與 obstacle map 作為候選路徑成本及 hard feasibility 資料。

目前 optimizer 明確保證：

```text
points.front() = self_state.position
points.back()  = snapshot.local_goal
```

因此 Local Goal 應在 Node／Planning Layer 產生，不應修改 optimizer 的任務語意。

### 6.2 FormationReservationPlanner

目前 `FormationReservationPlanner::reserve()` 是純 C++ helper，責任是：

```text
candidate trajectory
+ PlanningSnapshot peers
+ minimum distance
+ lead time / step / horizon
→ earliest safe start time
```

它會：

- 從 `max(snapshot.trajectory_start_time_s,
  snapshot.planning_time_s + lead_time_s)` 開始搜尋。
- 以固定 step 延後 start time。
- 逐點取樣 candidate 與 peer trajectory。
- 對 terminal hold peer 持續檢查 occupancy。
- 對 moving peer 使用 `TrajectoryPriority` 判斷時間窗優先權。
- 不修改 control points。

目前 Node 只在以下條件呼叫 reservation：

```cpp
if (has_formation_)
{
    FormationReservationPlanner::reserve(...);
}
```

所以目前 MOVE candidate 沒有經過這一層，也沒有使用 Formation 的
`reservation_lead_time_s_`。

### 6.3 TrajectoryPriority

目前規則：

1. `start_time` 較早者優先。
2. start time 相同時，較小 Agent ID 優先。
3. Terminal hold peer 永遠視為 occupancy，不因 priority 略過。
4. reservation 尚未開始前的 moving peer 仍視為佔位。

第一版不得修改 `TrajectoryPriority.hpp`。Local segment 可能造成 priority
在不同 segment 間交替，這是需要 diagnostics 與 SITL 觀察的風險，不可先改 priority
掩蓋問題。

### 6.4 ContinuousSafetyValidator

Safety validator 是所有 planner backend 共用的最後安全閘門，會取樣整條
B-spline，檢查：

- trajectory finite。
- frame revision 一致。
- start time、duration 與 future delay 合法。
- velocity、acceleration、jerk 上限。
- obstacle clearance。
- peer center-to-center separation。
- peer freshness 與 occupancy priority。

Node 目前在 reservation 後以：

```text
validate_plan(result.trajectory, ..., accepted_start_time_s)
```

驗證；這個時間參數不可改回未 reservation 前的 start time。

## 七、Peer Occupancy 與資料有效性

`PeerTrajectoryTable` 以 Agent ID 保存最新有效 trajectory，並拒絕：

- invalid trajectory。
- frame 或時間欄位無效。
- duration 與 end time 不一致。
- message／receive time 超出 skew。
- future start 超出允許延遲。
- trajectory ID 倒退。

Planner 可使用的 peer 狀態為：

- `ACTIVE`
- `TERMINAL_HOLD`

`STALE`、`MISSING`、`INVALID` 不可靜默當成空資料。

此資料流使 terminal hold 成為真正的空間 occupancy，而非單純任務完成訊號。
Generic Reservation 必須沿用此資料 contract。

## 八、BasicController 執行邊界

`BasicControllerNode::agent_trajectory_cb()` 只接受屬於本機 Agent 的
`AgentTrajectory`，並執行：

1. 驗證 `agent_index`、valid、WGS84 frame、duration 與 control points。
2. 以本機 GPS Home 將 WGS84 control points 轉為 local NED。
3. 呼叫 `BasicController::update_timed_trajectory()`。
4. BasicController 控制迴圈依 trajectory start time 取樣 p/v/a。
5. 由唯一的 BasicController PX4 publisher 發布 `TrajectorySetpoint`。

因此後續開發：

- 不修改 `BasicController`。
- 不新增第二個 PX4 setpoint publisher。
- 不新增 AgentTrajectory metadata 來承載 Local Goal。
- 必須避免無必要的 `trajectory_id`、`start_time` 重設，否則會重置執行進度。

## 九、後續開發的接入位置

### Step 2 — LocalGoalSelector

預定新增：

```text
src/ego_swarm/LocalGoalSelector.hpp
test/local_goal_selector_test.cpp
```

接入位置：

```text
get_current_target(final_target)
        ↓
決定 PathConstraint 對應的 planning space
        ↓
compute_local_goal(current, final_target, path_constraint, config)
        ↓
planning_target
        ↓
PlanningSnapshot.local_goal
```

不得放入 command callback、Formation Assignment 或 OptimizedEgoPlanner。

### Step 3 — Feature Flags

預定在 `EgoSwarmPlannerNode` 與 launch 增加：

```text
enable_local_goal_planning = false
local_goal_horizon_m = 10.0
local_goal_replan_ratio = 0.70
minimum_replan_interval_s = 0.5
enable_generic_trajectory_reservation = false
```

兩個功能初期都必須保持 backward-compatible。

### Step 4 — Generic Reservation

預定將 reservation 的「是否呼叫」從：

```text
has_formation_
```

改為：

```text
candidate 是 moving trajectory
且 generic reservation feature 已啟用
```

Formation 與 MOVE 必須使用相同 lead time 規則。minimum separation policy：

```text
MOVE      = peer_collision_min_distance_m
Formation = max(peer_collision_min_distance_m, formation_min_distance_m)
```

演算法本體與 priority 規則維持不變。

### Step 5 — Split Final / Planning / Active Target

Node 內需明確具備以下語意：

```text
final_target
planning_target
active_plan_target_
active_plan_final_target_
```

所有 arrival、Formation convergence、Terminal Hold、relative command 累積與
recovery 目標仍使用 `final_target`。

### Step 6 — 將 Local Goal 接入 snapshot

Feature disabled 時必須保持：

```text
planning_target == final_target
snapshot.local_goal == final_target
```

Feature enabled 時才讓 snapshot endpoint 使用 Local Goal。

### Step 7 — Replan Trigger

Active plan 必須同時檢查：

- Final Target 是否改變。
- Active trajectory 是否仍在 duration 內。
- Active trajectory 是否安全。
- Segment progress 是否達到 threshold。
- Normal replan 是否已通過 minimum interval。

不可只用「重新計算出的 Local Goal 不同」作為 invalidate 條件。

### Step 8 — Acceptance Chain

保持唯一接受路徑：

```text
OptimizedEgoPlanner
        ↓ success
Reservation
        ↓ accepted start time
ContinuousSafetyValidator
        ↓ safe
active_plan_ / active target state 更新
        ↓
publish_plan()
```

candidate 在 reservation 或 safety 失敗時，不可覆蓋仍安全的舊 active plan。

## 十、不可改變的架構邊界

本次第一版分析後確認以下項目是保護邊界：

- 不重新啟用 legacy `MissionPlanner`。
- 不重新啟用 legacy `TrajectoryController`。
- 不修改 FormationEpoch 語意。
- 不修改 FormationAssignment、slot ownership 或 convergence lifecycle。
- 不修改 FormationReservation 的核心演算法。
- 不修改 `TrajectoryPriority.hpp`。
- 不修改 `SwarmFrame`、`MovementFrame`、`EmergencyBrakePlanner`。
- 不修改 `ContinuousSafetyValidator` 的安全 contract。
- 不修改 `BasicController`。
- 不修改 `AgentTrajectory.msg` 或 `FormationEpoch.msg`。
- 不把 ROS2 callback、topic 或 lifecycle 放入 `src/ego_swarm/` 純邏輯類別。
- 不把 Local Goal 當作到達或 Terminal Hold。
- 不讓 MOVE 繞過 Generic Reservation。

## 十一、Step 1 發現的主要風險

| 風險 | 目前原因 | 後續處理方向 |
|---|---|---|
| Final Target 與 segment endpoint 混用 | `target` 直接填入 `snapshot.local_goal` | Step 5／6 拆分語意 |
| MOVE 未經 reservation | Node 目前只在 `has_formation_` 時呼叫 | Step 4 擴展共用 reservation |
| Local Goal 可能造成 target churn | 目前 active plan 只比較單一 target | Step 7 加入 progress 與 minimum interval |
| Local Goal 被誤判完成 | arrival／hold 目前直接使用 planner target | 保持 arrival／hold 使用 Final Target |
| Terminal hold 被誤當無 occupancy | Peer table 目前已支援 terminal hold，但新流程不可繞過 | Generic Reservation 繼續納入 terminal hold |
| segment 間 priority oscillation | priority 依 start time，每段可能重新競爭 | 保持 priority，增加 diagnostics 與 SITL 觀察 |
| candidate 失敗覆蓋安全舊路線 | 新流程若太早更新 active state 會破壞 recovery | 僅在 reservation + safety 通過後更新 |
| FIXED_Z 計算順序錯誤 | Local Goal 若先算 3D distance 再固定 Z 會改變 horizon 語意 | 先進 planning space，再計算 Local Goal |

## 十二、後續驗證基線

Step 1 本身沒有程式變更，因此不執行 Docker build 或 ROS2 測試。

後續每次實作變更必須遵守：

1. 先在 Docker 開發容器內完成 workspace build。
2. 再執行 `taiphoon_agent` 相關 unit tests。
3. 若修改共用核心或介面，再執行完整 test result。
4. Local Goal 與 Generic Reservation 完成後，依開發規格順序執行 SITL。
5. 記錄 trajectory ID、start time、priority、waiting、brake、formation
   convergence 與 terminal occupancy 診斷。

## 十三、Step 1 完成判定

Step 1 完成，因為目前已明確記錄：

- 輸入 topic 與 callback 邊界。
- MOVE 與 Formation 產生目標的完整路徑。
- `get_current_target()` 的現行語意。
- planner loop 的各個 safety／state gate。
- Optimized planner 的 endpoint contract。
- Reservation 的現行呼叫條件與責任。
- TrajectoryPriority 與 terminal occupancy 規則。
- ContinuousSafetyValidator 的 accepted start time contract。
- Active plan、arrival、convergence、Terminal Hold 與 BasicController 的關聯。
- Step 2～Step 8 的預定接入位置與禁止修改邊界。

後續實作應以本文件的「現行架構」段落作為舊架構參考，以正式開發規格的
「後續開發接入位置」段落作為變更前檢查清單。
