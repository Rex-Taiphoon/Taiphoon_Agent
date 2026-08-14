# 可直接交給 Codex 的 Phase Prompts

## Common Prefix

每次執行任一 Phase，都先附上以下內容：

```text
你正在開發 Rex-Taiphoon/Taiphoon_Agent 的 EGO-Swarm-dev。

開始前依照 repository scoped instructions 閱讀：
- AGENTS.md
- CODE_CONSTRAINT.md
- CONTEXT.md
- CHARACTERS.md
- src/AGENTS.md
- 所有即將修改目錄內的 AGENTS.md
- docs/codex/README.md
- docs/codex/EGO_SWARM_ARCHITECTURE.md
- docs/codex/EGO_SWARM_IMPLEMENTATION_GUIDE.md
- docs/codex/PHASE_STATUS.md

不要假設 README 比 source code 更新，以實際 implementation 為準。

修改前先輸出：
1. 本次 Phase
2. current architecture
3. 預計修改檔案
4. interface/data-flow changes
5. failure modes
6. test plan

一次只實作本次指定 Phase。
下一 Phase 的工作只能列成 FOLLOW-UP，不要提前實作。

遵守 AGENTS.md 的 Git / Docker / 測試 / Ask-first 規則。

完成後輸出：
- Changed files
- Behavior changes
- Safety impact
- Tests added
- Tests executed
- Test results
- Commands not run
- Remaining risks
- FOLLOW-UP
```

---

# Phase 0 Prompt

```text
執行 docs/codex/EGO_SWARM_IMPLEMENTATION_GUIDE.md 的 Phase 0 — Architecture Baseline。

目的不是增加功能，而是鎖定目前 EGO-Swarm-dev runtime behavior。

重點檢查：
- EgoSwarmPlanner
- BsplineTrajectory
- PeerTrajectoryTable
- SwarmFrame
- FormationSlotPlanner
- EgoSwarmPlannerNode
- BasicController
- BasicControllerNode
- obstacle simulator / PointCloud adapter

建立或補足 characterization tests，特別覆蓋：
- B-spline endpoint / invalid input
- peer future trajectory collision
- frame mismatch
- frame revision
- trajectory expiration
- finite PX4 output
- current obstacle behavior

建立 current behavior / architecture documentation。

不要修改 planner algorithm。
不要修改 PX4 runtime semantics。
不要實作下一 Phase。

完成後停止。
```

---

# Phase 1 Prompt

```text
執行 Phase 1 — Swarm Safety Foundation。

只處理：
- deterministic swarm origin
- frame revision/epoch contract
- peer trajectory lifecycle
- peer freshness
- timestamp sanity

新增：
swarm_origin_agent_index（預設 1）
peer_trajectory_timeout_s
max_peer_time_skew_s

每台 Agent 必須等待相同指定 origin 的有效 global position。
origin 初始化後 freeze。
origin Agent 後續離線不得重新建立 frame。
若 origin 啟動時不可用，planner 不得自行選自己的 GPS。

PeerTrajectoryTable 必須支援：
- erase
- prune
- expiration
- freshness

明確區分：
ACTIVE / STALE / MISSING / INVALID

禁止 trajectory 結束後永久把 endpoint 當成 peer。

補完整 unit tests。

不要修改 obstacle architecture。
不要修改 B-spline derivative。
不要修改 PX4 feed-forward。
完成後停止。
```

---

# Phase 2 Prompt

```text
執行 Phase 2 — Obstacle Abstraction & Simulation Sources。

目前沒有真 LiDAR / Depth Camera，但 obstacle avoidance architecture 必須完整保留。

建立 pure C++ obstacle-map abstraction，例如：
- src/perception/IObstacleMap.hpp
- src/perception/RawPointObstacleMap.hpp

Planner 不再永久直接依賴 raw vector<Vector3> obstacle points。

支援三種 simulation source：
1. explicit_empty
2. ground_truth
3. synthetic_sensor

explicit_empty 必須是 fresh + valid + zero obstacles。
不得用「沒有收到訊息」代表空環境。

ground_truth 從 YAML / simulator geometry 建立障礙物。

synthetic_sensor 必須使用 UAV current pose，並支援：
- range
- horizontal FOV
- vertical FOV
- noise
- dropout

修正目前 synthetic sensor pose 固定不動的問題。

新增：
obstacle_cloud_timeout_s
require_obstacle_cloud

必須區分：
fresh empty / missing / stale map。

不要加入 nonlinear optimizer。
完成後停止。
```

---

# Phase 3 Prompt

```text
執行 Phase 3 — B-Spline State + PX4 Feed-Forward。

將 trajectory evaluation 升級為：
- position
- velocity
- acceleration
- jerk

使用 analytic B-spline derivative。
不要使用控制迴圈 finite difference 作主要 derivative implementation。

BasicController 取樣 p/v/a 並填入 PX4 TrajectorySetpoint：
- position
- velocity
- acceleration

BasicController 維持唯一 PX4 TrajectorySetpoint publisher。

確認：
- position frame translation
- velocity axis convention
- acceleration axis convention

加入 derivative / continuity / finite / expiration / PX4 output tests。

不要加入 dynamic feasibility planner cost。
不要加入 emergency brake。
完成後停止。
```

---

# Phase 4 Prompt

```text
執行 Phase 4 — Dynamic Safety + Emergency Brake。

建立 VehiclePlanningState：
- position
- velocity
- acceleration
- timestamp

PlannerConfig 增加：
- max_velocity_mps
- max_acceleration_mps2
- max_jerk_mps3

建立 pure C++：
- ContinuousSafetyValidator
- EmergencyBrakePlanner

Validator 必須獨立於 planner backend，檢查：
- finite trajectory
- frame/time validity
- peer/map freshness
- dynamic feasibility
- peer separation
- obstacle clearance

當 vehicle 仍有顯著速度時：
禁止使用 repeated-current-position B-spline 當 emergency stop。

EmergencyBrakePlanner 應根據 current velocity / acceleration limits / jerk limits 產生 braking trajectory。

測試：
- stationary
- 3 m/s brake
- high-speed brake
- head-on conflict
- sudden obstacle
- stale peer
- stale map
- planner failure
- finite p/v/a
- speed reduction

不要開始 optimized EGO backend。
完成後停止。
```

---

# Phase 5 Prompt

```text
執行 Phase 5 — Safe Formation。

驗收重點：
4 台 PX4 SITL 必須可以安全形成 Circle Formation。

不要建立 Formation State Machine。

建立：
- FormationEpoch
- FormationAssignment

保留：
- FormationSlotPlanner
- Agent-ID rank assignment 作 fallback

至少支援：
- CIRCLE
- HORI_LINEAR
- VERT_LINEAR

Circle 第一優先。

FormationEpoch 至少固定：
- epoch id
- members
- formation type/config
- center
- slot assignment
- creation time

短暫 ROS discovery / peer packet loss 不得直接修改目前 Epoch。

加入 persistent slot assignment。
可採 Hungarian algorithm。

assignment cost 至少考慮：
- distance
- slot-switch penalty
- crossing penalty

formation transition：
freeze members
→ generate slots
→ assign
→ plan
→ exchange trajectory
→ safety validate
→ execute

unsafe 時：
replan / delay / reassign / brake。
禁止直接執行 crossing trajectory。

測試：
- random → circle
- circle convergence
- circle translation
- radius resize
- peer disconnect
- reconnect
- non-contiguous IDs
- circle + wall
- circle + cylinder

完成後停止。
```

---

# Phase 6 Prompt

```text
執行 Phase 6 — Optimized EGO Backend。

不要把 planner interface 放進 src/core。
放在 src/ego_swarm/。

建立：
- ITrajectoryPlanner
- DeterministicPlanner
- OptimizedEgoPlanner

保留 deterministic backend 作 fallback / regression。

新增 backend parameter：
planner_backend=deterministic
planner_backend=optimized

Optimized backend objective 至少包含：
- smoothness
- terminal goal
- dynamic feasibility
- obstacle
- peer swarm

對 CIRCLE 與 HORI_LINEAR 編隊新增下列不可違反的安全限制：
- 使用 FormationEpoch 的固定 formation altitude；所有 trajectory control
  point 的 NED Z 必須固定。
- 編隊避碰只能在 North/East 平面規劃；不得使用 above/below 繞行，也不得
  透過改變高度取得三維安全距離。
- 對已發布及已保留的 peer trajectory 建立空間／時間 reservation；只有
  reservation 不衝突的 Agent 才能同批移動，其餘 Agent 必須分時等待。
- 沒有安全水平 path 時只能 HOLD／brake，並重新產生水平候選或重新安排
  執行時間；不得改變高度。

Formation convergence 必須具備：
- 成員集合穩定後才建立初始 FormationEpoch，所有 Agent 使用相同
  epoch identity、snapshot hash、center 與 deterministic slot assignment。
- 每個 Agent 的 trajectory ID 在 runtime lifetime 單調遞增；舊封包晚到時
  只能忽略，不得清除最新 predecessor occupancy 或阻塞整個 transition。
- predecessor readiness 使用 fresh AgentState、槽位距離及最新 occupancy
  heartbeat 判定；槽位到達不使用位置差分速度門檻，不得依賴單一 terminal trajectory 封包的到達順序。
- 固定順序無解時，必須重新搜尋水平 lateral path 或重新安排時間；不得
  讓全隊永久停在 `Formation transition waiting`。
- 只有所有成員在固定高度抵達槽位且 heartbeat fresh，才算編隊收斂。

solver 與 cost function 分離。

optimizer 必須處理：
- NaN gradient
- iteration limit
- divergence
- infeasible result
- timeout

optimizer failure 不得直接輸出未驗證 trajectory。
必須回到既有 safety path。

若要新增外部 optimization dependency：
先回報 dependency / license / build impact / runtime footprint / alternative，
不要未經核准直接加入大型 dependency。

不要直接複製 GPL EGO-Swarm source implementation。

不要開始 topology。
完成後停止。
```

---

# Phase 7 Prompt

```text
執行 Phase 7 — Topological Initial Guesses。

建立多個 optimizer initial guesses：
- straight
- left
- right
- above
- below

`above` 與 `below` 僅適用一般 3D 任務；CIRCLE 與 HORI_LINEAR 編隊不得
採用垂直 initial guess 或垂直最終 trajectory。

這些不是最終 trajectory。

每一個：
initial guess
→ optimized planner
→ safety validate
→ cost compare

不得退回固定 5 candidate 直接選最終結果的舊模式。

補 local-minimum / obstacle topology tests。

完成後停止。
```

---

# Phase 8 Prompt

```text
執行 Phase 8 — Multi-PX4 SITL Validation。

建立可重複執行的 automated swarm SITL test flow。

優先使用 AutoTest 中現有 PX4 SITL launcher，不自行建立平行的 Docker 啟動流程。

依序測：
1 UAV
2 UAV
4 UAV
8 UAV

Scenario 至少：
- head-on
- crossing
- circle formation
- circle translation
- radius resize
- wall
- cylinder
- peer trajectory loss
- peer process failure
- map loss
- timestamp fault
- planner failure
- emergency brake

收集 machine-readable metrics：
- actual_min_peer_distance
- planned_min_peer_distance
- minimum_obstacle_clearance
- max position tracking error
- max velocity tracking error
- max trajectory age
- max map age
- planner computation time
- CPU
- memory
- emergency brake count
- failure count

如果現有 Docker image / environment 不可用：
不要自行 docker build。
列出 NOT RUN 與使用者應執行的命令。

完成後停止。
```

---

# Phase 9 Prompt

```text
執行 Phase 9 — Real Sensor Integration。

新增：
- LiDAR Adapter
- Depth Camera Adapter

Sensor Adapter 負責：
sensor frame
→ vehicle/local NED
→ swarm_ned
→ LocalObstacleMap

不要讓 EgoSwarmPlanner 直接處理 optical frame 或 sensor-driver-specific message。

正常情況不得修改：
- LocalObstacleMap contract
- ITrajectoryPlanner
- FormationEpoch
- FormationAssignment
- ContinuousSafetyValidator
- EmergencyBrakePlanner
- BasicController

如果必須修改這些核心 contract，先回報為 architecture defect，不要直接擴張 scope。

完成後停止。
```
