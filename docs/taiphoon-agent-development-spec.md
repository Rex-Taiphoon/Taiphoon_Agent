# Taiphoon Agent Master — Local Goal + Generic Trajectory Reservation 第一版開發計畫

## 1. 本次開發目標

以目前架構為唯一基準，在**不破壞既有 MOVE、Formation、Slot Assignment、FormationEpoch、Convergence、Terminal Hold、BasicController 與既有安全機制**的前提下，完成兩項彼此一致的功能：

1. 將目前「直接規劃到 Final Target」改為有限距離的 Local Goal / segmented trajectory planning。
2. 將目前只套用於 Formation 的 trajectory reservation，擴展成所有 moving trajectory 共用的時間仲裁機制。

本次不大幅重構 Formation lifecycle，也不修改現有 `TrajectoryPriority` 規則。

---

## 2. 最終希望建立的架構

整體應分成三層：

```text
                         TASK LAYER

       MOVE -------------------------------┐
                                            │
       Formation                            │
           ↓                                │
       FormationEpoch                       │
           ↓                                │
       Slot Assignment ---------------------┤
                                            ▼
                                       Final Target

────────────────────────────────────────────────────────

                      PLANNING LAYER

                                       Final Target
                                            │
                                            ▼
                                   Local Goal Selector
                                            │
                                      Planning Target
                                            │
                                            ▼
                                  OptimizedEgoPlanner
                                            │
                                    Candidate Trajectory
                                            │
                                            ▼
                                 Trajectory Reservation
                                            │
                                  Accepted Start Time
                                            │
                                            ▼
                              ContinuousSafetyValidator
                                            │
                                            ▼
                                   Accepted Trajectory

────────────────────────────────────────────────────────

                     EXECUTION LAYER

                                    AgentTrajectory
                                            │
                                            ▼
                                    BasicController
                                            │
                                            ▼
                                           PX4
```

核心原則：

> MOVE 與 Formation 只負責決定「最後要去哪裡」。

從 `Final Target` 以下，兩者應使用相同的 trajectory planning pipeline。

---

## 3. 本次最重要的資料語意

必須明確區分三種 target。

### 3.1 Final Target

代表真正任務終點。

MOVE 時：

```text
final_target = goal_position_
```

Formation 時：

```text
final_target = assigned formation slot
```

Final Target 必須繼續用於：

- MOVE 最終到達判斷
- Formation Slot 到達判斷
- Formation convergence
- Formation terminal hold
- Formation slot occupancy
- Formation cancel target preservation
- relative command target 累積
- recovery 後重新規劃原任務

Local Goal 絕對不得覆寫 Final Target。

---

### 3.2 Planning Target / Local Goal

代表：

> 這一輪 trajectory planner 要規劃到的局部終點。

傳入：

```cpp
PlanningSnapshot::local_goal
```

例如：

```text
current      = 0 m
final_target = 30 m
horizon      = 10 m

planning_target = 10 m
```

下一段：

```text
current      = 7 m
final_target = 30 m

planning_target = 17 m
```

接近終點時：

```text
planning_target = final_target
```

---

### 3.3 Active Plan Target

目前架構中已有：

```cpp
active_plan_target_
```

修改後其語意定義為：

> 目前已經正式接受並正在執行的 trajectory segment 所對應的 Planning Target。

例如：

```text
final_target        = 30
planning_target     = 17
active_plan_target_ = 10
```

代表目前還在執行前一段到 10 m 的 trajectory，而新的 17 m segment 尚未被接受。

---

## 4. 新增 Active Final Target

建議新增：

```cpp
ego_swarm::Vector3 active_plan_final_target_;
```

用途：

> 記錄目前 active trajectory 所屬任務真正的 Final Target。

接受新 trajectory 時：

```cpp
active_plan_target_ = planning_target;
active_plan_final_target_ = final_target;
```

這樣可以區分：

```text
Local segment 更新
```

與：

```text
真正任務目標改變
```

---

## 5. Final Target 改變與 Local Goal 改變必須分開處理

### Final Target 改變

例如：

- 新 MOVE_POINT
- 新 FORWARD / LEFT 指令
- 新 FormationEpoch
- Formation Slot reassignment
- Formation cancel
- Formation configuration change

應：

```text
invalidate active trajectory
→ 立即建立新 Local Goal
→ 重新規劃
```

---

### Local Goal 改變

Local Goal 會因 UAV 持續移動而自然改變。

不得因：

```text
newly_computed_local_goal
!=
active_plan_target_
```

就立即 invalidate active plan。

否則 planner 10 Hz 執行時會造成 trajectory churn。

---

## 6. Local Goal 第一版演算法

新增 pure C++ helper，例如：

```text
src/ego_swarm/LocalGoalSelector.hpp
```

建議介面：

```cpp
struct LocalGoalConfig
{
    double horizon_m{10.0};
};

struct LocalGoalResult
{
    bool is_valid{false};
    bool is_final_goal{false};
    Vector3 goal{0.0, 0.0, 0.0};
};

LocalGoalResult compute_local_goal(
    const Vector3& current,
    const Vector3& final_target,
    PathConstraint path_constraint,
    const LocalGoalConfig& config);
```

不得依賴 ROS2。

不得管理：

- Formation state
- Reservation
- Agent ID
- trajectory lifecycle
- terminal hold
- priority
- obstacle map

第一版只負責：

```text
current + final target + horizon
→ local goal
```

---

## 7. Local Goal 基本算法

Local Goal 必須先依 `PathConstraint` 進入對應的 planning space，再計算 horizon。

其中 `FIXED_Z` 必須先將規劃使用的 current Z 固定到 `final_target.z`，再以固定 Z 後的 current 與 final target 計算 Local Goal。不得先使用原始 3D 距離計算 horizon 後才修改 Local Goal 的 Z。

若：

```text
distance(planning_current, final_target)
<=
local_goal_horizon_m
```

則：

```text
planning_target = final_target
is_final_goal = true
```

否則：

```text
direction =
    normalize(final_target - planning_current)

planning_target =
    planning_current +
    direction * local_goal_horizon_m
```

---

## 8. PathConstraint 必須保持

Local Goal 不得改變目前的高度語意。

### FIXED_Z

例如：

- horizontal MOVE
- CIRCLE Formation
- HORI_LINEAR Formation

Local Goal 計算前先進入固定 Z 的 planning space：

```text
planning_current = current
planning_current.z = final_target.z
```

再使用 `planning_current` 與 `final_target` 計算 Local Goal。計算結果必須保持：

```text
planning_target.z = final_target.z
```

---

### LINEAR_Z

主要用於：

```text
VERT_LINEAR Formation
```

Local Goal Z 必須依據這一段沿完整 Final Target 路徑的進度比例做線性插值。

禁止額外產生：

```text
above
below
```

之類的 vertical bypass candidate。

---

### FREE_3D

沿用目前 planner 原有行為。

---

## 9. Local Goal 插入位置

Local Goal 必須放在：

```text
get_current_target()
        ↓
final_target
        ↓
compute_local_goal()
        ↓
planning_target
        ↓
PlanningSnapshot.local_goal
```

不要放進 MOVE callback。

也不要放進 Formation Assignment。

概念程式：

```cpp
ego_swarm::Vector3 final_target{};

if (!get_current_target(final_target))
{
    // preserve existing master recovery behavior
}

ego_swarm::Vector3 planning_target = final_target;

if (enable_local_goal_planning_)
{
    const auto local_goal_result =
        ego_swarm::compute_local_goal(
            current,
            final_target,
            path_constraint,
            local_goal_config_);

    if (!local_goal_result.is_valid)
    {
        // fail closed using existing behavior
    }

    planning_target = local_goal_result.goal;
}

snapshot.local_goal = planning_target;
```

---

## 10. OptimizedEgoPlanner 不修改

目前架構中已經有：

```cpp
points.front() = snapshot.self_state.position;
points.back() = snapshot.local_goal;
```

所以：

```text
planning_target
=
trajectory endpoint
```

這個 contract 保持不變。

本次不要修改 optimizer 的核心行為。

---

## 11. Formation Slot Assignment 完全保持原狀

禁止修改：

```text
FormationEpoch
FormationAssignment
FormationSlotPlanner
formation_assignment_
formation_center_
slot switch penalty
formation crossing penalty
membership consensus
epoch adoption
```

Formation：

```text
Slot Assignment
      ↓
Final Slot
      ↓
Local Goal
      ↓
Trajectory
```

Local Goal 永遠位於 Slot Assignment 之後。

因此：

```text
Formation Slot
!=
Local Goal
```

除非正在執行最後一段。

---

## 12. Formation 到達判斷必須繼續使用 Final Target

現有：

```cpp
evaluate_target_arrival(current, target);
```

修改時要把 `target` 的語意明確保留為：

```text
final_target
```

禁止：

```cpp
evaluate_target_arrival(
    current,
    planning_target);
```

只有：

```text
current ≈ assigned formation slot
```

才能判定 Formation Slot arrived。

---

## 13. Terminal Hold 必須鎖 Final Target

Formation 到達後：

```text
terminal_hold_position_
```

必須繼續設為：

```text
final_target
```

禁止 intermediate local goal 建立 terminal hold。

亦即：

```text
Local Goal reached
≠
task completed
```

---

## 14. Reservation 改成 Generic Planning Stage

目前 class 名稱暫時可以保留：

```cpp
FormationReservationPlanner
```

第一版不要為了 rename 製造大量 diff。

但是 Node 使用方式必須從：

```cpp
if (has_formation_)
{
    FormationReservationPlanner::reserve(...);
}
```

改成：

> 所有真正準備發布 moving trajectory 的 task 都要經過 reservation。

也就是 MOVE 和 Formation 都使用相同 reservation pipeline。

MOVE 與 Formation 必須使用相同的 reservation lead time 規則。即使 candidate trajectory 當下沒有 peer conflict，也必須保留既有 `reservation_lead_time_s`，不得讓 MOVE 因為不是 Formation 而繞過或取消 lead time。

---

## 15. Reservation 的責任

Reservation 只負責：

```text
Candidate Trajectory
+
Peer Occupancy
+
Candidate Start Time
↓
Earliest Safe Start Time
```

它不修改：

```text
control points
trajectory geometry
final target
formation slot
```

它只修改：

```text
start_time
```

---

## 16. 哪些 trajectory 要做 Reservation

只要 candidate 是：

```text
moving trajectory
```

都應執行 reservation。

例如：

- MOVE_POINT
- FORWARD
- BACKWARD
- LEFT
- RIGHT
- UP / DOWN
- Formation transition
- 未來其他自動移動任務

---

## 17. 哪些 peer 要被 Reservation 考慮

所有 peer occupancy。

包含：

### Moving Peer

```text
peer moving AgentTrajectory
```

### Terminal Hold Peer

```text
peer terminal_hold
```

Terminal Hold 代表實際空間佔位，永遠不能因 priority 被忽略。

因此系統原則應變成：

```text
Every UAV publishes occupancy

Every UAV that wants to move
reserves against all relevant peer occupancy
```

---

## 18. Reservation minimum separation

新增 helper，例如：

```cpp
double required_peer_separation() const;
```

規則第一版：

```cpp
double separation =
    planner_config_.peer_collision_min_distance_m;

if (has_formation_)
{
    separation = std::max(
        separation,
        formation_min_distance_m_);
}
```

即：

### MOVE

使用：

```text
peer_collision_min_distance_m
```

### Formation

使用：

```text
max(
    peer_collision_min_distance_m,
    formation_min_distance_m)
```

---

## 19. Reservation 的資料流位置不可改

保持：

```text
OptimizedEgoPlanner
        ↓
Candidate Trajectory
        ↓
Reservation
        ↓
Accepted Start Time
        ↓
ContinuousSafetyValidator
        ↓
Accept
```

不能移到 planner 前。

因為 Reservation 需要知道真正 trajectory：

```text
x(t), y(t), z(t)
```

不能移到 final safety 後。

因為 Reservation 可能改變：

```text
trajectory_start_time_s
```

Safety 必須驗證最後真正採用的時間。

---

## 20. TrajectoryPriority 第一版完全不修改

目前架構：

```text
earlier trajectory start time
→ higher priority

same start time
→ smaller Agent ID wins
```

第一版保持。

禁止修改：

```text
TrajectoryPriority.hpp
```

理由：

這次需要單獨觀察：

```text
Local segments
+
Generic reservation
+
current start-time priority
```

是否會產生 priority oscillation。

不能同時修改 Local Goal 與 priority，否則 regression 很難定位。

---

## 21. 已知 Priority Risk

Segment-based planning 後可能出現：

```text
A segment #1 start=10
B segment #1 start=11

A wins

A segment #2 start=12

B wins
```

下一輪：

```text
B segment #2 start=13

A wins
```

因此可能：

```text
A > B
↓
B > A
↓
A > B
```

這是第一版已知風險。

本次只做：

- 減少 segment 換段頻率
- 增加 diagnostics
- SITL 驗證

如果真的造成 deadlock / oscillation，再進第二階段設計 persistent route priority。

---

## 22. 不允許每 Planner Tick 重新規劃

Planner loop 可以維持 10 Hz。

但 trajectory generation 不得 10 Hz 重跑。

禁止：

```text
每 tick:
current changed
↓
local goal changed
↓
replan
```

否則會造成：

```text
trajectory_id churn
start_time churn
reservation churn
priority churn
BasicController trajectory reset risk
```

---

## 23. Segment Replan Policy

第一版只允許以下 trigger。

### Trigger A — Segment Progress

設定：

```text
local_goal_replan_ratio = 0.70
```

例如：

```text
0% ---------------- 70% -------- 100%
                       ↑
                    allow replan
```

---

### Trigger B — Active Trajectory Unsafe

沿用目前：

```text
ContinuousSafetyValidator
```

如果因：

- 新 peer trajectory
- obstacle change

而 unsafe：

```text
invalidate current active plan
→ immediately replan
```

---

### Trigger C — Segment Ended

如果：

```text
active trajectory ended
```

但是：

```text
Final Target not reached
```

必須重新規劃。

---

### Trigger D — Final Target Changed

立即重規劃。

例如：

```text
new MOVE command
new Formation Slot
new FormationEpoch
```

---

## 24. Minimum Replan Interval

新增：

```text
minimum_replan_interval_s
```

建議：

```text
0.5 s
```

適用於正常 progress-based replanning。

不要阻擋：

```text
hard safety / collision recovery
```

---

## 25. 建議第一版參數

新增：

```text
enable_local_goal_planning
local_goal_horizon_m
local_goal_replan_ratio
minimum_replan_interval_s
enable_generic_trajectory_reservation
```

建議：

```text
enable_local_goal_planning = false
local_goal_horizon_m = 10.0
local_goal_replan_ratio = 0.70
minimum_replan_interval_s = 0.5

enable_generic_trajectory_reservation = false
```

初期兩個 feature flag 都 default `false`。

方便做四種 A/B：

```text
A:
Local Goal OFF
Generic Reservation OFF
= pure master baseline

B:
Local Goal ON
Generic Reservation OFF

C:
Local Goal OFF
Generic Reservation ON

D:
Local Goal ON
Generic Reservation ON
```

這非常重要，可以快速分辨 regression 來源。

---

## 26. Active Plan 判斷邏輯調整

目前：

```cpp
target_unchanged =
    distance(active_plan_target_, target) <= epsilon;
```

Local Goal 加入後不能繼續直接用這個概念。

應改成：

```text
active task final target unchanged
+
active trajectory still within duration
+
active trajectory remains safe
+
replan threshold not reached
```

即可 KEEP ACTIVE。

---

## 27. Active Trajectory Acceptance Pipeline

任何 candidate 都必須：

```text
OptimizedEgoPlanner success
        ↓
Reservation success
        ↓
ContinuousSafetyValidator safe
        ↓
ACCEPT
```

之後才：

```cpp
active_plan_ = result;
active_plan_target_ = planning_target;
active_plan_final_target_ = final_target;
active_plan_start_time_ = accepted_start_time;
```

不能在 reservation / safety 前改 active state。

---

## 28. Candidate Failure 行為

新 segment 規劃失敗時，不要直接破壞安全的舊 active trajectory。

建議：

```text
candidate planning failed
        │
        ├─ old active trajectory still safe
        │        ↓
        │   continue old plan
        │
        └─ old active trajectory unsafe / ended
                 ↓
          existing wait / brake behavior
```

第一版盡量沿用現有 fail-closed recovery。

不要新增新的狀態機

---

## 29. 第一版禁止修改項目

除非 compile 需要，禁止修改：

```text
FormationAssignment.*
FormationConvergence.*
FormationSlotPlanner.*
FormationReservation.*   ← algorithm itself
TrajectoryPriority.hpp
SwarmFrame.*
MovementFrame.*
EmergencyBrakePlanner.*
ContinuousSafetyValidator.*
BasicController.*
AgentTrajectory.msg
FormationEpoch.msg
```

尤其：

```text
FormationEpoch semantic
slot ownership
terminal hold semantic
frame_revision
priority rule
```

全部保持。

---

## 30. 可以修改的檔案

主要：

```text
src/nodes/ego_swarm_planner_node.cpp
src/nodes/ego_swarm_planner_node.hpp

src/ego_swarm/LocalGoalSelector.hpp

test/local_goal_selector_test.cpp
planner/node regression tests

launch/basic_controller.launch.py
CMakeLists.txt
```

如果要加入 Generic Reservation helper，可新增：

```text
src/ego_swarm/TrajectoryReservationPolicy.hpp
```

但第一版盡量只是包 policy，不要複製 reservation 演算法。

---

## 31. Local Goal Unit Tests

至少：

### Final target within horizon

```text
current=(0,0,0)
target=(5,0,0)
horizon=10
```

預期：

```text
goal=(5,0,0)
is_final_goal=true
```

### Outside horizon

```text
current=(0,0,0)
target=(30,0,0)
horizon=10
```

預期：

```text
goal=(10,0,0)
```

### Arbitrary vector

```text
target=(30,40,0)
```

確認 local goal：

```text
distance = horizon
lies on target direction
```

### FIXED_Z

Z 保持。

### LINEAR_Z

Z 正確插值。

### Invalid NaN / Inf

fail closed。

---

## 32. MOVE Regression Tests

Feature OFF 時必須與 master 一致。

測：

```text
MOVE_FORWARD
MOVE_BACKWARD
MOVE_LEFT
MOVE_RIGHT
MOVE_UP
MOVE_DOWN
MOVE_POINT
```

確認：

- target 不變
- yaw 不變
- altitude semantics 不變
- arrival 不變
- hold 不變

---

## 33. Formation Regression Tests

必須測：

```text
FORMATION_CIRCLE
FORMATION_HORI_LINEAR
FORMATION_VERT_LINEAR
FORMATION_CANCEL
Formation → Formation
member leave
```

確認：

```text
FormationEpoch unchanged
slot index unchanged
formation center unchanged
slot switch penalty behavior unchanged
crossing penalty behavior unchanged
convergence unchanged
terminal hold unchanged
```

---

## 34. Local Goal + Formation 特殊驗證

例如：

```text
current = (0,0)
formation slot = (30,0)
horizon = 10
```

預期：

```text
final_target = (30,0)
planning_target = (10,0)
```

但是：

```text
formation_assignment_
```

仍指向原 Slot。

到 `(10,0)` 時：

```text
has_formation_ remains true
terminal hold not created
formation not considered arrived
```

直到 `(30,0)` 才到位。


---

## 35. SITL 驗收順序

Codex 必須依此順序，不要跳著測。

### Stage 1 — 基本指令功能

必須確認以下指令功能正常：
```text
ARM
TAKEOFF
MOVE_FORWARD
MOVE_BACKWARD
MOVE_UP
MOVE_DOWN
MOVE_LEFT
MOVE_RIGHT
```
並且移動到位時應該呈現懸停狀態，Setpoint不應該再調整，不允許飛機大幅度飄動

---

### Stage 2 — 長距離移動測試 - 無障礙物
啟動一台無人機模擬器起飛後，執行以下命令
```text
MOVE_FORWARD 30
MOVE_BACKWARD 30
MOVE_LEFT 30
MOVE_RIGHT 30
```

確認：
- Local Goal 規劃出來的空間幾何路徑應保持直線，不得產生額外的空間彎曲路徑
- 可以成功抵達目的地

---

### Stage 3 — Peer Occupancy 避碰測試

#### Case 1 — Moving UAV vs. Static Peer Occupancy

啟動多機模擬器並起飛，並且將 fmu2 移動到 fmu1 北方 10m。

fmu1 向北飛行，假設機頭向北：

```text
MOVE_FORWARD 30
```

確認：
- fmu1 是否可以成功避開 fmu2，保持 3m 以上的間隔
- fmu1 可以成功抵達目的地

#### Case 2 — Opposing Moving Peer Trajectories

啟動兩台無人機模擬器並起飛，安排 fmu1 與 fmu2 分別執行互相相向的水平移動：

```text
fmu1 向東飛行 10m
fmu2 向西飛行 10m
```

確認：
- 兩台飛機都可以安全抵達各自目的地
- 移動過程中兩台飛機必須實時保持安全距離

---


### Stage 4 — 編隊測試

啟動多機模擬器並起飛

執行以下編隊任務
```text
FORMATION_CIRCLE
FORMATION_HORI_LINEAR
FORMATION_VERT_LINEAR
```

確認：
- 編隊時Slot是否會持續變化
- 飛機是否可以成功成功編隊，如果無法成功編隊應該紀錄核心問題

---

## 36. 必須觀察的 Failure Modes

Codex 測試時特別記錄：

```text
trajectory_id growing every planner tick
start_time continuously resetting
same UAV repeatedly waiting
A/B priority alternating
formation never converging
local goal becoming terminal hold
slot assignment unexpectedly changing
repeated emergency brake
reservation starvation
```

任何一項發生都不得用「直接修改 Priority」解掉。

先記錄與定位。

---

## 37. 本次 Success Criteria

完成條件：

1. MOVE 與 Formation 都只負責產生 Final Target。
2. Local Goal 位於共用 planning layer。
3. Formation Slot 不受 Local Goal 影響。
4. Formation Assignment 不變。
5. FormationEpoch 不變。
6. Formation convergence 不變。
7. Terminal Hold 只使用 Final Target。
8. `PlanningSnapshot.local_goal` 使用 Planning Target。
9. OptimizedEgoPlanner 不知道 MOVE / Formation。
10. 所有 moving trajectory 都經過 reservation。
11. Terminal Hold peer 始終被當 occupancy。
12. Reservation 只改路徑起始時間、不改路徑。
13. ContinuousSafetyValidator 使用 reservation 後的 accepted start time。
14. TrajectoryPriority 第一版保持不變。
15. Active trajectory 不會每 10 Hz 被重建。
16. Local Goal segment 能正常逐段推進到 Final Target。
17. Formation + independent MOVE 能使用同一套 arbitration。
18. 沒有造成既有 Formation regression。

---

## 38. Codex 執行順序

### Step 1 — Analyze

先閱讀：

```text
planner_loop_cb()
get_current_target()
get_formation_target_for_agent()
update_formation_assignment()
evaluate_target_arrival()
evaluate_formation_convergence()

FormationReservationPlanner
TrajectoryPriority
ContinuousSafetyValidator
OptimizedEgoPlanner
```

輸出目前資料流分析。

先不要改程式。

---

### Step 2 — Add LocalGoalSelector

Pure C++。

完成 unit tests。

不要碰 Node lifecycle。

---

### Step 3 — Add Feature Flags

新增：

```text
enable_local_goal_planning
enable_generic_trajectory_reservation
local_goal_horizon_m
local_goal_replan_ratio
minimum_replan_interval_s
```

預設皆保持 backward-compatible。

---

### Step 4 — Generic Reservation

將 reservation 從：

```text
Formation only
```

擴展成：

```text
all moving trajectories
```

不要修改 reservation algorithm。

不要修改 priority。

完成 reservation-only regression tests。

---

### Step 5 — Split Final / Planning / Active Target

在 Node 明確區分：

```text
final_target
planning_target
active_plan_target_
active_plan_final_target_
```

所有 task completion 保持使用 `final_target`。

---

### Step 6 — Add Local Goal Planning

將：

```cpp
snapshot.local_goal = final_target;
```

改為：

```cpp
snapshot.local_goal = planning_target;
```

feature disabled 時仍完全等於原始行為。

---

### Step 7 — Replan Trigger

加入：

```text
progress >= ratio
active trajectory unsafe
segment ended
final target changed
```

避免 planner tick-based replanning。

---

### Step 8 — Preserve Acceptance Chain

保持：

```text
planner
→ reservation
→ safety
→ accept
```

並確保 MOVE 與 Formation 都一致。


---

## 39. Hard Requirements 給 Codex

請將以下視為不可違反的規則：

> Do not redesign the formation lifecycle.

> Do not modify FormationEpoch semantics.

> Do not modify FormationAssignment or slot ownership.

> Do not modify TrajectoryPriority in this phase.

> Do not modify BasicController.

> Do not treat a Local Goal as task completion.

> Do not publish terminal hold at an intermediate Local Goal.

> Do not replan every planner tick.

> Do not bypass reservation for MOVE trajectories.

> Do not ignore terminal-hold peer occupancy.

> Do not modify candidate geometry inside the reservation stage.

> Do not accept a trajectory before reservation and continuous safety validation both pass.

> Preserve original master behavior when both feature flags are disabled.

---

## 40. 本次第一版的邊界

本次刻意不做：

```text
persistent task priority
route_id
segment sequence ID
agent-id-only priority
new AgentTrajectory metadata
multi-candidate Local Goal
starvation recovery
new replanning hold state
new Formation lifecycle
frame revision redesign
```

這些留到第一版 SITL 結果明確之後再決定。

---

## 41. 第一版完成後應得到的系統

```text
MOVE ──────────────┐
                   │
Formation → Slot ──┤
                   ▼
              Final Target
                   │
                   ▼
             Local Goal
                   │
                   ▼
           Trajectory Planner
                   │
                   ▼
         Trajectory Reservation
                   │
                   ▼
            Safety Validator
                   │
                   ▼
          Active Trajectory
                   │
                   ▼
            BasicController
                   │
                   ▼
                  PX4
```

其中：

```text
Formation
```

只多了：

```text
FormationEpoch
Slot Assignment
Convergence
```

而「怎麼安全移動到目標」則由所有 task 共用同一套 Planning Layer。

這才是本次第一版最重要的架構目標。
