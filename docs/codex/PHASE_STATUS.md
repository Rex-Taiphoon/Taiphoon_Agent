# EGO-Swarm Phase Status

> 每完成一個 Codex Phase 後更新本檔。
> `Accepted` 必須由人類開發者確認，不由 Codex 自行宣告。

| Phase | Status | Branch/Commit | Unit Test | SITL | Review Notes |
|---|---|---|---|---|---|
| 0 Architecture Baseline | Accepted | EGO-Swarm-dev | Docker: 149 tests passed | Human SITL verification: basic functions OK | Characterization tests and baseline docs added; human acceptance confirmed |
| 1 Swarm Safety Foundation | Accepted | | Docker: 162 tests passed | Human SITL verification: basic functions OK | Deterministic origin/frame freeze, peer lifecycle/freshness and timestamp sanity implemented; human acceptance confirmed |
| 2 Obstacle Abstraction | Accepted | | Docker: 171 tests passed | Basic SITL smoke passed | Pure C++ map abstraction, explicit empty／ground truth／synthetic sensor and freshness gate implemented；四機 Agent/PX4 已完成 quick_takeoff、HOLD、MOVE_FORWARD、MODE_RTL、MODE_LAND 基本鏈路驗證，未納入編隊驗證；人類開發者已確認驗證完成 |
| 3 B-Spline p/v/a/j + PX4 | Accepted | | Docker: 174 tests passed | 四機 SITL 基本鏈路與非零 p/v/a feed-forward smoke passed | 人類開發者已確認驗證完成；未提前進入 Phase 4 |
| 4 Dynamic Safety + Brake | Accepted | EGO-Swarm-dev (uncommitted) | Docker: 190 tests passed | 四機 Agent/PX4 quick_takeoff、六方向移動、編隊與 peer safety smoke passed | WGS84 transport、receiver-local NED Adapter、ContinuousSafetyValidator、EmergencyBrakePlanner 與 terminal hold heartbeat 已實作；2026-08-14 人類開發者確認基本指令與動態安全驗證完成 |
| 5 Safe Formation | Accepted | EGO-Swarm-dev (uncommitted) | Docker: 209 tests passed | 四機 SITL：無衝突時可並行規劃；衝突時局部等待／重規劃；fmu4 Agent 失效後 sequence 2 自動重編為 `[1,2,3]`，恢復後 sequence 3 納回四機 | 水平路線、時間／Agent ID 仲裁、無額外上下繞行、單調 trajectory ID、Agent failure recovery 已驗證；2026-08-14 人類開發者確認編隊功能驗證完成 |
| 6 Optimized EGO Backend | In Progress | | Docker: 226 tests passed；optimized-only SITL 的 Forward 40 smoke 通過，四機水平編隊長時間安全驗證仍待修正 | optimized-only runtime、expanded horizontal search、priority occupancy hardening 與 fail-closed HOLD／brake 已實作 | deterministic 不再作 runtime fallback；3 m hard peer distance 維持；Forward 40 中曾反覆觸發 live peer warning brake 後恢復；Hori transition 曾觀察到實測距離低於 3 m；human acceptance pending |
| 7 Topological Initial Guesses | Not Started | | | | |
| 8 Multi-PX4 SITL Validation | Not Started | | | | |
| 9 Real Sensor Integration | Not Started | | | | |

Allowed status:

```text
Not Started
In Progress
Blocked
Ready for Review
Accepted
```

## Phase 2 Verification Record（2026-08-12）

- Docker build：`colcon build --packages-select taiphoon_agent` 通過。
- Docker unit tests：`171 tests, 0 errors, 0 failures, 0 skipped`。
- SITL 基本 smoke test：先以 `docker compose up -d` 啟動四個 Agent，再以
  `launch_all.sh --count 4 --rmw cyclonedds` 啟動四台 PX4；`quick_takeoff.sh`
  後四個 AgentState 均確認 `flight_mode=5`、`armed=true`、約 8 m NED 高度。
- `HOLD`、`MOVE_FORWARD`、其餘基本位移／yaw 命令均成功發布；`MOVE_FORWARD`
  實際造成四機 NED 前向位移，planner status 的 `map_valid=true`。
- 四機 `MODE_RTL` 均進入 RTL，`MODE_LAND` 均進入 Auto Land；SITL 清理前未將
  尚未完成的落地判定標記為通過。編隊命令依開發者指示不列入本次驗證。
- PX4 SITL 已依 `AutoTest/scripts/px4-sitl-launcher/stop_all.sh` 清理；Agent
  容器未刪除。

## Phase 3 Verification Record（2026-08-12）

- Docker build：明確 source ROS2 Humble 與既有 `px4_msgs` workspace 後，
  `colcon build --packages-select taiphoon_agent` 通過。
- Docker unit tests：`174 tests, 0 errors, 0 failures, 0 skipped`。
- Pure C++ derivative tests：已驗證 cubic analytic p/v/a/j、端點 derivative，
  以及內部 knot 的 p/v/a continuity。
- 四機 SITL：先以 `docker compose up -d` 啟動四個 Agent，再以
  `launch_all.sh --count 4 --rmw cyclonedds` 啟動 PX4；`quick_takeoff.sh`
  後四個 AgentState 均確認 `flight_mode=5`、`armed=true`、`online=true`，
  高度約 8 m。
- `HOLD`、`MOVE_FORWARD 2.0`、`MOVE_FORWARD 5.0` 與 `MODE_RTL`、`MODE_LAND`
  均成功廣播；四機 `MODE_RTL` 進入 `flight_mode=6`，`MODE_LAND` 進入
  `flight_mode=7`。SITL 清理前未將尚未完成的落地判定標記為通過。
- `MOVE_FORWARD 20.0` 執行期間擷取 fmu1 PX4 `TrajectorySetpoint`：
  `position=[21.0147,-2.3261,-7.6422]`、`velocity.x=1.80894`、
  `acceleration.x=0.446349`，全部為 finite，證明 PX4 feed-forward 已由
  analytic derivative 提供；jerk/yawspeed 維持 finite zero。
- SITL 已依 `AutoTest/scripts/px4-sitl-launcher/stop_all.sh` 清理；Agent
  容器保留執行中。

## Phase 4 Verification Record（2026-08-12）

- Docker build：明確 source ROS2 Humble 與既有 `px4_msgs` workspace 後，
  `colcon build --packages-select taiphoon_agent` 通過。
- Docker unit tests：`190 tests, 0 errors, 0 failures, 0 skipped`。
- Phase 4 pure C++ 測試已涵蓋 stationary hold、3 m/s brake、high-speed brake、
  noisy acceleration、head-on peer conflict、sudden obstacle、stale peer/map、
  planner failure、finite p/v/a 與 deterministic planner output safety gate。
- SITL 依序以 `docker compose up -d` 啟動四個 Agent，再以
  `launch_all.sh --count 4 --rmw cyclonedds` 啟動四台 PX4；`quick_takeoff.sh`
  後四台均確認 `flight_mode=5`、`armed=true`、`online=true`、`landed=false`、
  `px4_state_valid=true`，且 PlannerStatus 為 `map_valid=true`、
  `localization_valid=true`、`peer_data_valid=true`。
- 以 fmu2 Agent command topic 逐台驗證 `MOVE_FORWARD`、`MOVE_BACKWARD`、
  `MOVE_LEFT`、`MOVE_RIGHT`、`MOVE_UP`、`MOVE_DOWN`；每個命令均造成對應方向的
  WGS84／AgentState 位置變化，並維持 `peer_data_valid=true`。部分命令在同儕安全
  邊界上先進入 HOLD，之後安全條件成立再恢復 `reason=ok`，沒有穿越安全閾值。
- 以四機起始間距約 3 m 的狀態廣播移動命令時，曾觀測到 fmu1
  `Emergency brake: peer_collision`；這是安全拒絕，不是碰撞，符合
  `peer_collision_min_distance_m=3.0` 的 fail-closed 合約。純 C++ 測試另涵蓋
  head-on peer conflict 與連續最小距離驗證。
- 編隊驗證：`FORMATION_CIRCLE 10 8` 與 `FORMATION_HORI_LINEAR 10 8` 均觀測四台
  `PlannerStatus state=1`、`reason=ok`、`peer_data_valid=true`。以較緊的
  `FORMATION_CIRCLE 6`／水平轉換測試時，部分候選路徑因起始機間距與 3 m 安全門檻
  不足而 HOLD／Emergency brake；未讓軌跡穿過不安全距離。
- WGS84 peer trajectory 已由每個接收端自己的固定 Home 轉成 local NED；SITL 未再
  出現 `frame origin does not match`。terminal hold heartbeat 也保留已完成 Agent
  的終點佔位；BasicController 在 heartbeat 超過 B-spline duration 後不再重複輸出
  `Timed trajectory unavailable (result=3)`。
- SITL PX4 容器已依 `AutoTest/scripts/px4-sitl-launcher/stop_all.sh` 清理；
  Agent 容器保留執行中。

## Human Acceptance Record（2026-08-12）

- Phase 2 Obstacle Abstraction：人類開發者確認 Docker build、unit tests 與四機 SITL
  基本鏈路驗證完成，狀態更新為 `Accepted`。
- Phase 3 B-Spline p/v/a/j + PX4：人類開發者確認 analytic derivative、PX4
  feed-forward 與四機基本指令驗證完成，狀態更新為 `Accepted`。
- Phase 4 Dynamic Safety + Brake：2026-08-14 人類開發者確認基本指令、動態安全與煞車驗證完成，狀態更新為 `Accepted`。
- Phase 5 Safe Formation：2026-08-14 人類開發者確認編隊與 Agent failure recovery 驗證完成，狀態更新為 `Accepted`。

## Phase 5 Verification Record（2026-08-13）

- Docker build：明確 source ROS2 Humble 與既有 `px4_msgs` workspace 後，
  `colcon build --packages-select taiphoon_agent` 通過。
- Docker unit tests：`209 tests, 0 errors, 0 failures, 0 skipped`。
- 四機 SITL 依序以 `docker compose up -d` 啟動 Agent，再以
  `launch_all.sh --count 4 --rmw cyclonedds` 啟動 PX4；`quick_takeoff.sh` 後四台均
  確認 `flight_mode=5`、`armed=true`、`landed=false`。
- `FORMATION_CIRCLE 10 8` 建立四機 epoch；無衝突的 Agent 可同時開始規劃，只有
  路線互相衝突的 Agent 才進入局部等待／重規劃。transition 只使用直線或 North/East
  水平側向候選；Z 只沿目標線性插值，未觀察到從上方／下方繞行。純 C++ 回歸測試另
  確認較早 `start_time` 與較小 Agent ID 的仲裁規則，以及 formation planner 不產生
  額外垂直 detour。
- fmu4 Agent 隨機失效後，discovery 連續兩次穩定掃描自動建立 sequence 2，成員由
  `[1,2,3,4]` 變為 `[1,2,3]`；等待 deterministic safe-order 完成後三台均為
  terminal hold 且 `peer_data_valid=true`。
- 重新啟動 fmu4 Agent 後，Agent 狀態恢復為 online 且 PX4 狀態有效；系統自動建立
  sequence 3 納回 fmu4。PX4 在 Agent 失效期間依安全機制落地，恢復後保持解除 armed，
  不由 Planner 自動復飛；其他三台維持安全 hold。
- Emergency brake 進行中沿同一條時間軸發布，未再觀察到舊 terminal hold trajectory ID
  覆蓋新軌跡或 `peer table latest` out-of-order rejection。

## Phase 6 Verification Record（2026-08-14）

- 純 C++ planner seam 已完成：`ITrajectoryPlanner`、`PlanningSnapshot`、
  `DeterministicPlanner`、`OptimizedEgoPlanner`、projected-gradient solver、
  formation reservation 與 convergence evaluator；未新增 ROS2 message/topic，且
  `BasicController` 仍是唯一 PX4 `TrajectorySetpoint` 發布端。
- Docker 內先完成 `colcon build --packages-select taiphoon_agent`，再執行完整套件
  測試；結果為 `226 tests, 0 errors, 0 failures, 0 skipped`，其中 Phase 6 pure C++
  測試為 13 項全數通過。新增測試涵蓋長距離移動在預設 20 ms optimizer budget 下的
  planning path。
- compose、entrypoint 與 launch 預設均已切換為 `planner_backend=optimized`；四個
  Agent 啟動 log 均確認 `backend=optimized`。設定 `deterministic` 會使 Planner
  initialization 失敗，不會進入 runtime fallback。
- 依 Agent 先於 PX4 的順序啟動四機 SITL，`quick_takeoff.sh` 後四台均確認
  `flight_mode=5`、`armed=true`、`landed=false`、`online=true`、
  `px4_state_valid=true`，且 PlannerStatus 的 localization／peer freshness 均有效。
- optimized open-space `MOVE_FORWARD 5` 完成，fmu1 實際 NED X 約前進 5.4 m，之後
  發布 terminal hold；active planner sample 的規劃時間約 10–15 ms。
- optimized `FORMATION_CIRCLE 10 8` smoke 中，觀測到 active planner 的最小 peer
  距離約 6.7–8.0 m，且 trajectory control-point altitude 維持相同值；部分成員也
  觀測到 `formation_converged; terminal hold active`。
- 四機 `HORI_LINEAR` transition 曾在警戒帶內觸發 optimized emergency brake；一個
  snapshot 的 live peer minimum 為 3.47、3.48、5.71、5.05 m，符合 fail-closed
  方向。但延長觀測後仍有一組 live peer distance 約降至 2.09 m 並進入 hard emergency
  brake，故四機水平編隊的實際 3 m 安全驗收尚未完成，Phase 6 維持 `In Progress`。
- 2026-08-14 重新啟動 Agent 與四台 PX4，依序完成 `quick_takeoff.sh`、fmu1
  `YAW 90`、`MOVE_FORWARD 40`。測試早期曾反覆出現
  `optimized_emergency_brake: live peer warning distance reached`，當時 live peer
  minimum 約 3.4–4.4 m；未觀察到原先的 `optimized_timeout`。後續 planner 恢復規劃，
  fmu1 最終位置為 `NED=(-1.15, 40.21, -7.83)`，Forward 40 smoke 通過。此結果記錄一個
  尚待後續處理的行為：live warning brake 會在安全警戒帶內重複觸發，雖然本次最終能恢復，
  但會造成移動期間暫時停滯；四機水平編隊長時間安全驗收仍未完成。
- Phase 6 pure C++ tests 已涵蓋 point obstacle lateral offset、固定／線性 Z、NaN／
  timeout／iteration limit／divergence、optimized failure fail-closed、reservation、assignment crossing
  refinement 與 convergence 條件；Phase 7 維持 `Not Started`，human acceptance pending。

---

## Current Accepted Safety Contract

尚未建立。

---

## Known Risks

- `SwarmFrame` 由每個 planner node 以自己的有效 GPS freeze local Home；
  `swarm_origin_agent_index` 僅保留為 launch 相容參數。
- planner 目前對 missing/stale obstacle map 採 fail-closed HOLD；`require_obstacle_cloud`
  的 false 分支只改變診斷原因文字，未把 missing map 放寬成空環境。
- Phase 2 synthetic sensor 已改由 AgentState／AgentAttitude 與固定 origin 更新 pose；尚未
  整合真實 LiDAR／Depth Camera，也尚未模擬 sensor latency。
- Phase 3 已提供 analytic p/v/a/j derivative 與 PX4 p/v/a feed-forward；Phase 4
  新增的是 hard dynamic safety gate，不是 optimized planner cost。
- Phase 4 已建立 WGS84 transport、receiver-local NED conversion、terminal hold、
  continuous trajectory safety validator 與動態 emergency brake；2026-08-14 已由人類開發者確認驗證完成。
- FormationEpoch 已提供跨 Agent snapshot/ADOPTED 同步、deterministic assignment、
  crossing refinement、連續 reservation、無衝突並行、衝突局部仲裁與成員失效重編；
  optimized backend 失敗時不使用 deterministic fallback，而是進入 fail-closed HOLD／
  emergency brake 並重試。
- Phase 5 編隊 Node 已禁止額外的上下垂直避碰 candidate；Phase 6 optimized planner
  仍維持 CIRCLE／HORI_LINEAR 固定高度與 VERT_LINEAR 線性 Z 約束；多 topology initial
  guesses 依 scope 留待 Phase 7。

---

## FOLLOW-UP Queue

只記錄尚未進入目前 Phase scope 的工作。

| Item | Origin Phase | Target Phase | Reason |
|---|---:|---:|---|
| Real sensor adapter and sensor latency/dropout characterization | 2 | 9 | Phase 2 只完成 simulation sources 與 pure C++ LocalObstacleMap seam |
