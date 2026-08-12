# EGO-Swarm 分階段實作指南

## Global Rule

Codex 一次只能實作一個 Phase。

每個 Phase 必須可以獨立：

```text
Build → Unit Test → SITL / Integration Validation → Review → Accept
```

未 Accepted 不進下一 Phase。

若發現後續工作：

```text
FOLLOW-UP:
```

記錄即可，不提前實作。

---

# Phase 0 — Architecture Baseline

## Goal

在改變 runtime behavior 前建立 characterization baseline。

## Tasks

檢查：

- `EgoSwarmPlanner`
- `BsplineTrajectory`
- `PeerTrajectoryTable`
- `SwarmFrame`
- `FormationSlotPlanner`
- `EgoSwarmPlannerNode`
- `BasicController`
- `BasicControllerNode`
- obstacle simulator / PointCloud adapter

新增：

- `PlanningSnapshot` 設計文件
- current behavior 文件
- simulation profile contract

補測試：

- B-spline endpoint
- invalid B-spline
- peer collision candidate rejection
- frame mismatch
- frame revision
- local trajectory expiration
- finite PX4 output
- current obstacle behavior

## Do Not

- 不改 planner algorithm
- 不改 PX4 behavior
- 不加入 optimizer
- 不改 collision threshold semantics

## Exit

- baseline build/test 狀態已知
- runtime behavior 有 characterization test
- docs 與 code 一致

---

# Phase 1 — Swarm Safety Foundation

## Goal

解決 frame、peer trajectory、freshness 與 clock contract。

## Implement

### Frame

```text
swarm_origin_agent_index
```

預設 1。

- 等待指定 origin GPS
- first valid origin 後 freeze
- origin Agent 移動不改 frame
- origin Agent 離線不改 frame
- origin 不可用時不可自選自己的 GPS

### Peer trajectory

加入：

- receive time
- end time
- erase
- prune
- freshness
- expiration

加入：

```text
peer_trajectory_timeout_s
```

狀態：

```text
ACTIVE
STALE
MISSING
INVALID
```

### Time

加入：

```text
max_peer_time_skew_s
```

不合理 future / old trajectory reject。

## Tests

- staggered startup
- Agent2 先啟動、Agent1 後啟動
- origin unavailable
- peer timeout
- peer disappear
- stale trajectory
- replay old trajectory
- future timestamp
- frame mismatch

## Do Not

- 不做 B-spline derivative
- 不改 obstacle architecture
- 不加入 optimizer

---

# Phase 2 — Obstacle Abstraction & Simulation Sources

## Goal

目前沒有真 LiDAR，也完整保留避障架構。

## Implement

建立：

```text
src/perception/IObstacleMap.hpp
src/perception/RawPointObstacleMap.hpp
```

Planner 不再永久直接耦合 raw point vector。

Simulation source：

```text
explicit_empty
ground_truth
synthetic_sensor
```

### explicit_empty

fresh valid empty map。

### ground_truth

YAML / world geometry → map。

### synthetic_sensor

使用 UAV 當前 pose：

- range
- FOV
- noise
- dropout

修正 sensor pose 固定於 origin 的問題。

加入：

```text
obstacle_cloud_timeout_s
require_obstacle_cloud
```

必須區分：

```text
fresh empty map
missing map
stale map
```

## Tests

- empty but valid
- missing map
- stale map
- wall
- box
- cylinder
- sensor range
- FOV
- noise finite
- dropout
- moving sensor pose

## Do Not

- 不加入 nonlinear optimizer

---

# Phase 3 — B-Spline State + PX4 Feed-Forward

## Goal

同一 trajectory 產生 p/v/a/j，PX4 使用 p/v/a。

## Implement

Trajectory state：

```text
position
velocity
acceleration
jerk
```

analytic derivative。

BasicController：

```text
TrajectorySetpoint.position
TrajectorySetpoint.velocity
TrajectorySetpoint.acceleration
```

保持唯一 setpoint publisher。

## Tests

- known derivative
- endpoint derivative
- continuity
- finite values
- expiration
- frame revision
- PX4 output finite

---

# Phase 4 — Dynamic Safety + Emergency Brake

## Goal

Planner 理解 UAV 不可瞬間停止。

## Implement

```cpp
VehiclePlanningState
{
    position
    velocity
    acceleration
    timestamp
}
```

Planner config：

```text
max_velocity_mps
max_acceleration_mps2
max_jerk_mps3
```

加入：

```text
ContinuousSafetyValidator
EmergencyBrakePlanner
```

Validator 至少：

- finite
- dynamic feasibility
- peer separation
- obstacle clearance
- frame/time/freshness

Brake：

根據 current v/a 與 limits 產生 braking trajectory。

UAV 尚有顯著速度時不得直接 repeated-current-position hold。

## Tests

- stationary hold
- 3 m/s brake
- high speed brake
- sudden peer conflict
- sudden obstacle
- stale peer
- stale map
- planner failure
- monotonic speed reduction
- finite p/v/a

---

# Phase 5 — Safe Formation

## Goal

至少 4 台 PX4 SITL 安全完成 circle formation。

## Implement

不要新增 Formation State Machine。

建立：

```text
FormationEpoch
FormationAssignment
```

保留：

```text
FormationSlotPlanner
```

支援：

- CIRCLE
- HORI_LINEAR
- VERT_LINEAR

Circle 第一優先。

新增 persistent slot assignment。

可使用 Hungarian assignment。

cost：

```text
distance
+ slot_switch_penalty
+ crossing_penalty
```

Formation command：

```text
freeze members
→ generate slots
→ assign
→ plan
→ exchange trajectory
→ safety validate
→ execute
```

Agent 暫時 missing 不立刻重排。

reconfiguration 透過新 FormationEpoch。

## Tests

- random start → circle
- circle convergence
- circle translation
- radius 10 → 5 → 15
- peer disconnect
- peer reconnect
- non-contiguous IDs
- formation + wall
- formation + cylinder
- crossing-risk assignment

---

# Phase 6 — Optimized EGO Backend

## Goal

保留 deterministic fallback，新增 optimized continuous planner。

## Implement

```text
src/ego_swarm/ITrajectoryPlanner.hpp
DeterministicPlanner
OptimizedEgoPlanner
```

不要放 `src/core/`。

backend parameter：

```text
planner_backend=deterministic
planner_backend=optimized
```

objective 至少：

```text
smoothness
terminal goal
dynamic feasibility
obstacle
peer swarm
```

optimizer failure：

```text
→ safety validator
→ emergency brake / deterministic fallback
```

視 safety contract 決定。

## Dependency Rule

如果要新增 optimizer dependency：

先回報：

- name
- license
- build impact
- runtime footprint
- alternative

未核准不要擅自加入大型 dependency。

不要直接 copy GPL upstream implementation。

---

# Phase 7 — Topological Initial Guesses

## Goal

解決 single initial path 的 local minimum。

產生：

```text
straight
left
right
above
below
```

但這些只作 initial guess。

流程：

```text
initial guess
→ optimized backend
→ validate
→ compare valid result
```

不要退回「固定 5 條 candidate 直接當最終 trajectory」的舊邏輯。

---

# Phase 8 — Multi-PX4 SITL Validation

## Goal

建立可重複的 swarm safety acceptance。

依序：

```text
1
2
4
8
```

台 UAV。

Scenario：

- head-on
- crossing
- circle
- translation
- resize
- wall
- cylinder
- peer trajectory loss
- peer process failure
- map loss
- timestamp fault
- planner failure
- emergency brake

收集：

```text
actual_min_peer_distance
planned_min_peer_distance
minimum_obstacle_clearance
tracking error
trajectory age
map age
planner time
CPU
memory
brake count
failure count
```

建立 machine-readable report。

---

# Phase 9 — Real Sensor Integration

## Goal

驗證 sensor-agnostic architecture。

建立：

```text
LiDARAdapter
DepthCameraAdapter
```

負責：

```text
sensor frame
→ local NED
→ swarm_ned
→ LocalObstacleMap
```

正常情況不得修改：

- planner backend
- formation
- continuous safety validator
- emergency brake
- BasicController

若實機 sensor integration 必須修改上述 core，先視為 architecture defect 並回報。
