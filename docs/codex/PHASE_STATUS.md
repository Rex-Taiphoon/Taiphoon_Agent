# EGO-Swarm Phase Status

> 每完成一個 Codex Phase 後更新本檔。
> `Accepted` 必須由人類開發者確認，不由 Codex 自行宣告。

| Phase | Status | Branch/Commit | Unit Test | SITL | Review Notes |
|---|---|---|---|---|---|
| 0 Architecture Baseline | Accepted | EGO-Swarm-dev | Docker: 149 tests passed | Human SITL verification: basic functions OK | Characterization tests and baseline docs added; human acceptance confirmed |
| 1 Swarm Safety Foundation | Accepted | | Docker: 162 tests passed | Human SITL verification: basic functions OK | Deterministic origin/frame freeze, peer lifecycle/freshness and timestamp sanity implemented; human acceptance confirmed |
| 2 Obstacle Abstraction | Ready for Review | | Docker: 171 tests passed | Basic SITL smoke passed | Pure C++ map abstraction, explicit empty／ground truth／synthetic sensor and freshness gate implemented；四機 Agent/PX4 已完成 quick_takeoff、HOLD、MOVE_FORWARD、MODE_RTL、MODE_LAND 基本鏈路驗證，未納入編隊驗證 |
| 3 B-Spline p/v/a/j + PX4 | Not Started | | | | |
| 4 Dynamic Safety + Brake | Not Started | | | | |
| 5 Safe Formation | Not Started | | | | |
| 6 Optimized EGO Backend | Not Started | | | | |
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

---

## Current Accepted Safety Contract

尚未建立。

---

## Known Risks

- `SwarmFrame` 的 frame freeze 由 planner node 以 `swarm_origin_agent_index`
  （預設 1）實作：等待指定 origin 的有效 GPS，建立後不因移動或離線改選新 origin。
- planner 目前對 missing/stale obstacle map 採 fail-closed HOLD；`require_obstacle_cloud`
  的 false 分支只改變診斷原因文字，未把 missing map 放寬成空環境。
- Phase 2 synthetic sensor 已改由 AgentState／AgentAttitude 與固定 origin 更新 pose；尚未
  整合真實 LiDAR／Depth Camera，也尚未模擬 sensor latency。
- B-spline 尚未提供 analytic p/v/a/j derivative，PX4 feed-forward 留待 Phase 3。
- continuous trajectory safety validator 與動態 emergency brake 尚未建立；留待 Phase 4。

---

## FOLLOW-UP Queue

只記錄尚未進入目前 Phase scope 的工作。

| Item | Origin Phase | Target Phase | Reason |
|---|---:|---:|---|
| Real sensor adapter and sensor latency/dropout characterization | 2 | 9 | Phase 2 只完成 simulation sources 與 pure C++ LocalObstacleMap seam |
| Analytic B-spline derivatives and PX4 feed-forward | 0 | 3 | Current controller publishes finite zero default p/v/a fields |
| Continuous safety validation and emergency brake | 0 | 4 | Current planner only validates sampled candidate distance |
