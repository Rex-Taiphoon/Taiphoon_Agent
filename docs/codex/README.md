# Codex 開發入口 — Taiphoon EGO-Swarm

> 此目錄是 Codex 進行 EGO-Swarm / 機群規劃相關工作的主要開發文件入口。
> `AGENTS.md` 負責「必須遵守的規則」，本目錄負責「架構、階段與單次任務」。

## Codex 閱讀順序

任何 EGO-Swarm / 編隊 / 避障 / 軌跡 / PX4 Offboard 相關任務開始前，依序閱讀：

1. repository 根目錄 `AGENTS.md`
2. `CODE_CONSTRAINT.md`
3. `CONTEXT.md`
4. `CHARACTERS.md`
5. `src/AGENTS.md`
6. 即將修改目錄中的 scoped `AGENTS.md`
7. 本檔
8. `EGO_SWARM_ARCHITECTURE.md`
9. `EGO_SWARM_IMPLEMENTATION_GUIDE.md`
10. 僅閱讀目前要執行 Phase 在 `EGO_SWARM_CODEX_PROMPTS.md` 中的對應段落

不要一次實作多個 Phase。

---

## 文件分工

### `EGO_SWARM_ARCHITECTURE.md`

描述最終系統邊界與資料流。

回答：

- 哪個模組負責什麼？
- 模擬障礙物與未來真實 LiDAR / Depth Camera 如何共用架構？
- Formation、Planner、Safety、BasicController、PX4 的邊界在哪？
- 哪些 runtime data flow 不可破壞？

### `EGO_SWARM_IMPLEMENTATION_GUIDE.md`

描述分階段開發順序與每階段的 Definition of Done。

回答：

- 先做什麼？
- 哪些功能不可提前做？
- 每階段要增加哪些測試？
- 何時可以進下一階段？

### `EGO_SWARM_CODEX_PROMPTS.md`

提供可直接交給 Codex 的單一 Phase 任務提示詞。

每次只複製一個 Phase。

### `PHASE_STATUS.md`

紀錄各 Phase 是否：

- Not Started
- In Progress
- Blocked
- Ready for Review
- Accepted

以及已知風險與驗證結果。

---

## 核心目標

目前主要開發環境：

- ROS2 Humble
- PX4 SITL
- Multi-PX4 simulation
- 尚未要求真實 LiDAR / Depth Camera

但架構必須保留完整避障能力。

模擬階段障礙物來源支援：

- `explicit_empty`
- `ground_truth`
- `synthetic_sensor`

未來實機增加：

- LiDAR Adapter
- Depth Camera Adapter

只替換 Obstacle Source / Sensor Adapter。

不得因此重新設計：

- LocalObstacleMap
- trajectory planner
- formation
- continuous safety validator
- emergency brake
- BasicController / PX4 output

---

## 目前重要架構決策

### Formation

不建立獨立 Formation State Machine。

使用：

- `FormationEpoch`
- persistent slot assignment
- reconfiguration policy
- derived `formation_converged`

避免建立另一套與 `PlannerStatus`、PX4 `FlightState` 重疊的狀態機。

### Planner

現有 deterministic candidate planner：

- 保留
- 作 fallback
- 作 regression baseline

未來允許加入：

- `OptimizedEgoPlanner`

不得把 deterministic backend 視為永久唯一實作。

### PX4

`BasicController` 必須維持為唯一 runtime `TrajectorySetpoint` publisher。

Planner Node 不得直接寫入 PX4 setpoint topic。

### Safety

Planner output 必須先通過獨立的 trajectory safety validation。

如果 UAV 尚有顯著速度，失敗時不得直接以 current-position static hold 取代 braking trajectory。

---

## 單次 Codex 任務原則

修改前先回報：

1. 本次 Phase
2. 預計修改檔案
3. 介面變更
4. data flow
5. failure modes
6. test plan

完成後回報：

1. Changed files
2. Behavior changes
3. Safety impact
4. Tests added
5. Tests executed
6. Test results
7. Commands not run
8. Remaining risks
9. FOLLOW-UP

FOLLOW-UP 只記錄，不提前實作下一 Phase。
