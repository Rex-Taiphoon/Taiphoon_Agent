# Taiphoon Agent — Simulation-First EGO-Swarm Architecture

## 1. 目標

建立一套：

- Simulation-first
- Sensor-agnostic
- PX4-based
- decentralized multi-UAV

的機群軌跡規劃架構。

現階段使用 PX4 SITL，也必須能完整測試：

- 環形編隊
- 水平 / 垂直線型編隊
- 編隊轉換
- 編隊平移
- 編隊尺寸改變
- UAV ↔ UAV 預測避碰
- 靜態障礙物避障
- stale trajectory / stale map
- peer failure
- planner failure
- emergency braking

未來接 LiDAR / Depth Camera 時，只新增或替換 Sensor Adapter。

---

## 2. 不可破壞的 Runtime Data Flow

```text
Mission / GCS
    │
    ▼
Formation / Local Goal
    │
    ▼
EgoSwarm Planner
    │
    ▼
Trajectory Safety Validation
    │
    ├── SAFE ────────────────┐
    │                        │
    └── UNSAFE               │
          │                  │
          ▼                  │
   Emergency Brake           │
          │                  │
          └──────────┬───────┘
                     ▼
              AgentTrajectory
                     │
                     ▼
              BasicController
                     │
              B-spline p/v/a
                     │
                     ▼
           PX4 TrajectorySetpoint
```

### 強制規則

- `BasicController` 是唯一 PX4 `TrajectorySetpoint` runtime publisher。
- Planner Node 不直接控制 PX4。
- Agent 不直接控制其他 Agent 所屬 PX4。
- Agent 間透過 ROS2/DDS 交換：
  - Agent state
  - mission intent
  - formation information
  - future trajectory
  - planner/safety status

---

# 3. 模組責任

## 3.1 `src/ego_swarm/`

主要機群規劃 domain。

負責：

- common swarm frame
- vehicle planning state
- planning snapshot
- B-spline representation
- peer future trajectory
- formation epoch
- formation slot geometry
- formation assignment
- planner backend
- dynamic feasibility
- peer collision prediction
- trajectory safety validation
- emergency braking

不得：

- 依賴 `rclcpp`
- 持有 publisher/subscriber
- 直接發布 PX4 messages
- 依賴 LiDAR / camera driver

---

## 3.2 `src/perception/`

障礙物表示與 sensor-independent mapping。

負責：

- LocalObstacleMap abstraction
- raw point obstacle map
- voxel / spatial map（後續）
- obstacle geometry
- synthetic obstacle generation
- PointCloud normalization adapter

不得：

- 做 formation planning
- 做 swarm trajectory optimization
- 控制 PX4

---

## 3.3 `src/nodes/`

ROS2 組裝層。

負責：

- topic
- ROS2 message validation
- parameters
- timer
- QoS
- module composition
- ROS2 ↔ pure C++ data conversion

主要 runtime node：

- `BasicControllerNode`
- `EgoSwarmPlannerNode`
- `ObstacleCloudSimulatorNode`

不得在 Node 中實作主要數學演算法。

---

## 3.4 `src/controllers/`

飛控 translation layer。

負責：

- PX4 Offboard heartbeat
- VehicleCommand
- TrajectorySetpoint
- trajectory sampling result → PX4
- PX4 local NED tracking
- flight state

不得：

- formation assignment
- obstacle planning
- peer trajectory optimization

---

## 3.5 `src/safety/`

保留為 flight/runtime monitoring。

例如：

- PX4 state translation
- state validity
- position / runtime monitoring

不要把 EGO trajectory optimization safety 全部塞入這個模組。

trajectory-specific：

- dynamic feasibility
- continuous trajectory validation
- emergency braking

放在 `src/ego_swarm/`。

---

## 3.6 `src/formation/` 與 `src/trajectory/`

維持 legacy。

不得重新把：

- `MissionPlanner`
- `TrajectoryController`

接回 runtime。

現行 formation / trajectory development 使用 `src/ego_swarm/`。

---

# 4. Planning Snapshot

一次 planner cycle 必須對同一份一致資料做決策。

建議：

```cpp
struct VehiclePlanningState
{
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    double timestamp_s;
};

struct PlanningSnapshot
{
    VehiclePlanningState self_state;

    Vector3 local_goal;

    SwarmFrameSnapshot frame;
    PeerTrajectorySnapshot peers;
    ObstacleMapSnapshot obstacles;

    double planning_time_s;
};
```

流程：

```text
collect
  ↓
validate freshness / frame / timestamps
  ↓
freeze snapshot
  ↓
plan(snapshot)
```

Planner 執行過程不要直接重新讀 ROS callback 更新中的資料。

---

# 5. Swarm Frame

AgentTrajectory 的跨 Agent transport contract 固定使用 WGS84 geodetic：

```text
[latitude_deg, longitude_deg, height_m]
```

每個接收端在自己的有效 GPS Home 首次建立後 freeze 一份 `SwarmFrame` snapshot，
並將收到的 WGS84 控制點轉成自己的公尺制 local NED。這個 local NED 只存在於
接收端的規劃與控制計算，不會作為跨 Agent message 的座標格式。如此即使不同
Agent 的 Home 不同，也不會把發布端 NED 與接收端 NED 混用。

`swarm_origin_agent_index` 保留作為 launch 相容參數；WGS84 trajectory exchange
不再要求所有 Agent freeze 同一個原點。`frame_revision` 改為全球 WGS84 contract
版本，僅用來識別 message schema／座標語義，不是 peer origin 比對值。

---

# 6. Peer Future Trajectory

每個 peer record 至少包含：

```text
agent_id
trajectory_id
trajectory
start_time
receive_time
end_time
frame_revision
```

狀態：

```text
ACTIVE
TERMINAL_HOLD
STALE
MISSING
INVALID
```

必須支援：

- update
- erase
- prune
- expiration
- freshness check

規則：

- 移動中的 trajectory 必須在 receive timeout 與 end time 內才可使用。
- 完成任務的 Agent 必須以 `terminal_hold=true` 持續發布固定終點 heartbeat；
  heartbeat fresh 時，接收端把該終點當成佔位 peer 參與避碰。
- heartbeat timeout 後才移除終點，不可無限期使用最後有效資料。

---

# 7. Clock Contract

Peer collision prediction 依賴共同時間。

即使 SITL 通常共用 host clock，也必須實作：

```text
max_peer_time_skew_s
```

驗證：

- message stamp
- trajectory start_time
- receive time
- current planner time

未來實機 clock sync：

- chrony
- NTP
- PTP

不應要求修改 planner API。

---

# 8. Obstacle Architecture

## 8.1 統一資料流

```text
Obstacle Source
      │
      ▼
Normalized obstacle representation
      │
      ▼
LocalObstacleMap
      │
      ▼
Planner
```

Planner 不知道 source 是：

- YAML
- Gazebo
- synthetic LiDAR
- real LiDAR
- depth camera

---

## 8.2 Simulation Source A — `explicit_empty`

表示「明確沒有障礙物」。

必須與「沒有 map」區分。

```text
fresh empty map:
    valid = true
    count = 0
    stamp = current

missing map:
    valid = false
```

禁止用「沒有收到 PointCloud」代表沒有障礙物。

---

## 8.3 Simulation Source B — `ground_truth`

由：

- YAML
- simulator world geometry

直接建立障礙物。

用途：

- planner correctness
- regression
- formation + obstacle
- optimizer validation

不模擬 sensor FOV。

---

## 8.4 Simulation Source C — `synthetic_sensor`

從 ground-truth world 根據 UAV 當前 pose 模擬：

- range
- horizontal FOV
- vertical FOV
- noise
- dropout
- latency

Synthetic sensor pose 必須隨 UAV 更新。

不得固定於 swarm origin。

---

## 8.5 Future Real Sensors

```text
LiDAR / Depth Camera
      │
 sensor frame
      │
      ▼
Sensor Adapter
      │
 local NED / swarm_ned
      │
      ▼
LocalObstacleMap
```

不得因新增 sensor 修改 planner core。

---

# 9. LocalObstacleMap

Planner 不應永久直接持有：

```cpp
std::vector<Vector3> obstacle_points;
```

目標介面，例如：

```cpp
class IObstacleMap
{
public:
    virtual ~IObstacleMap() = default;

    virtual bool is_valid() const = 0;
    virtual bool is_fresh(double now_s) const = 0;

    virtual double nearest_distance(
        const Vector3& position) const = 0;

    virtual bool is_position_safe(
        const Vector3& position,
        double clearance_m) const = 0;
};
```

演進：

```text
V1 RawPointObstacleMap
V2 SpatialHash / VoxelObstacleMap
V3 optimized distance query
```

---

# 10. B-Spline Contract

最終 trajectory state：

```cpp
struct TrajectoryState
{
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 jerk;
    bool valid;
};
```

Planner 使用：

- p
- v
- a
- j

BasicController 至少使用：

- p
- v
- a

derivative 必須來自 spline analytic derivative，不使用控制迴圈 finite difference 作主要實作。

---

# 11. PX4 Tracking

```text
B-spline
   ├── p
   ├── v
   └── a
        │
        ▼
BasicController
        │
        ▼
PX4 TrajectorySetpoint
```

位置 frame conversion 需要 translation。

velocity / acceleration 不套 position origin translation，但必須確認 axis convention 一致。

---

# 12. Dynamic Feasibility

Planner config 至少：

```text
max_velocity_mps
max_acceleration_mps2
max_jerk_mps3
```

所有可執行 trajectory 都必須通過：

```text
|v| <= v_max
|a| <= a_max
|j| <= j_max
```

Phase 4 的 `PlannerConfig` 以 `max_velocity_mps`、`max_acceleration_mps2` 與
`max_jerk_mps3` 作為 canonical 動力學限制；Phase 3 的
`max_velocity_m_s`／`max_acceleration_m_s2` 仍保留為相容別名。`EgoSwarmPlanner`
用相同限制估算 candidate duration，`ContinuousSafetyValidator` 再以獨立 safety
gate 驗證整條 B-spline 的 analytic p/v/a/j 取樣，避免 planner cost 被誤當成安全保證。

---

# 13. Emergency Brake

建立獨立 pure C++：

```text
EmergencyBrakePlanner
```

觸發條件包含：

- no safe trajectory
- imminent peer collision
- imminent obstacle collision
- planner failure
- stale critical input
- invalid trajectory

UAV 仍有顯著速度時：

禁止只建立 current-position repeated control points。

應根據：

- current velocity
- max acceleration
- max jerk

建立可執行 braking trajectory。

完全停止後才可以 static HOLD。

目前 `EmergencyBrakePlanner` 使用自身 `VehiclePlanningState` 的 position、velocity、
acceleration 與 timestamp 建立有限 jerk braking B-spline。當速度仍顯著時，控制點不會
全部重複目前位置；只有速度接近零且加速度很小時，才允許產生固定 hold trajectory。

---

# 14. Continuous Safety Validator

Planner backend 的輸出不得直接發布。

流程：

```text
PlannerResult
     │
     ▼
ContinuousSafetyValidator
     │
     ├── SAFE → publish
     └── FAIL → EmergencyBrake / safe fallback
```

至少檢查：

- finite trajectory
- frame consistency
- trajectory freshness
- map freshness
- peer freshness
- dynamic feasibility
- minimum peer separation
- obstacle clearance

Validator 必須獨立於 planner backend。

目前 `ContinuousSafetyValidator` 的 safety gate 至少檢查：

- finite trajectory state 與輸出指標
- frame revision、trajectory start／end time 與自身狀態 freshness
- peer table 的 ACTIVE freshness 與時間有效性
- obstacle map 的 valid／fresh 狀態
- velocity、acceleration、jerk norm
- 取樣期間的 peer center separation 與 obstacle clearance

Validator 失敗、planner failure、stale peer 或 stale map 時，Node 交由
`EmergencyBrakePlanner` 產生 fail-closed trajectory；BasicController 仍是唯一 PX4
`TrajectorySetpoint` publisher。

---

# 15. Formation

## 15.1 不建立 Formation State Machine

不新增：

```text
IDLE → PREPARING → TRANSITIONING → ...
```

避免和：

- PlannerStatus
- PX4 FlightState
- peer freshness state

形成多套互相耦合狀態機。

---

## 15.2 FormationEpoch

目前以 `FormationEpoch.msg` 交換 sequence、parent identity、proposer/sender、
proposal/adopted 狀態、formation config、WGS84 center、排序後 member snapshot、
parent slot ownership、snapshot hash 與 creation timestamp。Topic 為
`/fmu_all_agent/swarm/formation_epochs`，使用 Reliable + Transient Local；已採用的
Agent 會定期重送目前 snapshot，讓後加入 Agent 取得 heartbeat。

收到新的 formation command 或穩定成員集合變更才建立下一代 Epoch。相同 config 與
相同成員 snapshot 的重複命令是 idempotent；短暫 discovery 波動要連續兩次穩定才會
觸發重編隊。

---

## 15.3 Formation Convergence

不是額外的 state machine state。Leaderless 收斂依 parent、sequence 與 proposer
優先權選出同一個 snapshot；只有觀測到 epoch 內所有 members 都發布相同 hash 的
`ADOPTED` 後，才執行 assignment 與新 trajectory。收斂前維持舊 epoch 或安全 hold。

---

## 15.4 Reconfiguration

Agent Node discovery 每 3 秒執行；新增成員必須同時具備 Agent Node 與 fresh、finite、
online 的 AgentState，既有成員要連續兩次消失才移除。Node 存在但 AgentState stale
不會偷偷移除，trajectory safety 仍 fail-closed。建立新 epoch 時沿用 parent center，
只重新產生槽位與 assignment。

## 15.5 Transition Release Contract

Phase 5 不建立全隊固定的 safe-order。每台 Agent 都可以先為自己的目標槽位規劃；
路線沒有空間或時間衝突時可以同時執行，只有實際互相衝突的 Agent 才局部協調。
候選路徑先檢查目前位置到目標槽位的直線；直線受阻時只增加 North/East 中段側向
控制點，Z 維持起點到目標的線性插值。不得以額外的上升或下降控制點作為避碰繞路。

衝突的路線以 `start_time` 較早者優先，同時間以較小 Agent 編號作為固定判斷；後者
只暫停或延後，不影響沒有衝突的 Agent。沒有安全水平候選時，機體維持 hold／煞停，
並在安全條件改變後重新規劃。Agent discovery 連續兩次確認成員消失或重新出現後，
沿用 parent center 建立下一代 epoch，重新執行 assignment；失效成員的 PX4 若已
落地或解除 armed，重新加入後仍須先恢復飛控的 Offboard／armed 狀態，Planner 不會
繞過飛控安全條件自行復飛。

---

# 16. Circle Formation

必須支援。

對 N 台 UAV：

```text
theta_i = 2*pi*i/N
```

slot：

```text
center + [R*cos(theta), R*sin(theta), z_offset]
```

Agent-ID rank assignment 僅作輸入無效時的明確 fallback；正常 assignment 使用
deterministic Hungarian algorithm，成本為 canonical NED 三維距離加上
`formation_slot_switch_penalty_m`，每個 Agent 與 slot 都恰好一個。

---

# 17. Formation Assignment

cost 建議：

```text
distance
+ slot_switch_penalty
+ crossing_penalty
```

可使用 Hungarian algorithm。

目標：

- 減少總移動距離
- 減少 crossing
- peer 掉線時減少其他 UAV 不必要 slot migration

---

# 18. Planner Backend

介面放：

```text
src/ego_swarm/ITrajectoryPlanner.hpp
```

不要放 `src/core/`。

```cpp
class ITrajectoryPlanner
{
public:
    virtual ~ITrajectoryPlanner() = default;

    virtual PlannerResult plan(
        const PlanningSnapshot& snapshot) = 0;
};
```

Backend：

```text
DeterministicPlanner
OptimizedEgoPlanner
```

deterministic：

- fallback
- regression
- open-space baseline

optimized：

後期加入：

- continuous B-spline optimization
- obstacle cost
- peer cost
- smoothness
- feasibility
- topology initial guesses

---

# 19. Simulation Profiles

## Profile A — Swarm Only

```text
obstacle_source = explicit_empty
```

測：

- peer avoidance
- formation
- PX4 tracking

## Profile B — Ground Truth Obstacle

```text
obstacle_source = ground_truth
```

測：

- environment obstacle avoidance
- formation + obstacle

## Profile C — Synthetic Sensor

```text
obstacle_source = synthetic_sensor
```

測：

- FOV
- range
- noise
- dropout
- stale perception

---

# 20. Required SITL Scenarios

至少：

1. 2 UAV head-on
2. 2 UAV crossing
3. 4 UAV random start → circle
4. circle translation
5. circle radius resize
6. circle + wall
7. circle + cylinder
8. peer trajectory loss
9. peer Agent process failure
10. obstacle map loss
11. bad timestamp
12. planner failure
13. emergency braking

---

# 21. Runtime Safety Metrics

至少記錄：

```text
actual_min_peer_distance
planned_min_peer_distance
minimum_obstacle_clearance
maximum_position_tracking_error
maximum_velocity_tracking_error
maximum_trajectory_age
maximum_map_age
planner_computation_time
planner_failure_count
emergency_brake_count
frame_mismatch_count
stale_peer_count
```

主要 safety KPI：

```text
actual_min_peer_distance
minimum_obstacle_clearance
```
