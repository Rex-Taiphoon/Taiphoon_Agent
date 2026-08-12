# EGO-Swarm Phase Status

> 每完成一個 Codex Phase 後更新本檔。
> `Accepted` 必須由人類開發者確認，不由 Codex 自行宣告。

| Phase | Status | Branch/Commit | Unit Test | SITL | Review Notes |
|---|---|---|---|---|---|
| 0 Architecture Baseline | Accepted | EGO-Swarm-dev | Docker: 149 tests passed | Human SITL verification: basic functions OK | Characterization tests and baseline docs added; human acceptance confirmed |
| 1 Swarm Safety Foundation | Accepted | | Docker: 162 tests passed | Human SITL verification: basic functions OK | Deterministic origin/frame freeze, peer lifecycle/freshness and timestamp sanity implemented; human acceptance confirmed |
| 2 Obstacle Abstraction | Not Started | | | | |
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

---

## Current Accepted Safety Contract

尚未建立。

---

## Known Risks

- `SwarmFrame` 的 frame freeze 由 planner node 以 `swarm_origin_agent_index`
  （預設 1）實作：等待指定 origin 的有效 GPS，建立後不因移動或離線改選新 origin。
- 尚未收到 obstacle cloud 時 planner 仍把空列表當成無障礙物；freshness／require policy 留待 Phase 2。
- synthetic sensor pose 目前固定在 `(0, 0, 0)`，尚未跟隨 UAV；留待 Phase 2。
- B-spline 尚未提供 analytic p/v/a/j derivative，PX4 feed-forward 留待 Phase 3。
- continuous trajectory safety validator 與動態 emergency brake 尚未建立；留待 Phase 4。

---

## FOLLOW-UP Queue

只記錄尚未進入目前 Phase scope 的工作。

| Item | Origin Phase | Target Phase | Reason |
|---|---:|---:|---|
| Obstacle source abstraction and explicit empty profile | 0 | 2 | Current node passes missing cloud as empty raw vector |
| Analytic B-spline derivatives and PX4 feed-forward | 0 | 3 | Current controller publishes finite zero default p/v/a fields |
| Continuous safety validation and emergency brake | 0 | 4 | Current planner only validates sampled candidate distance |
