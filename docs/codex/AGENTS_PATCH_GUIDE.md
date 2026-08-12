# Existing AGENTS.md Patch Guide

> 這份文件說明在開始 Phase 0 前，建議如何更新既有 scoped `AGENTS.md`。
> 建議由人類 review 後套用；不要讓 Codex 未經確認大幅改寫所有規則。

---

# 1. `/AGENTS.md`

## Existing concern

目前「多機協作一律透過 Agent ↔ Agent 指令」措辭太窄。

實際架構還需要交換：

- state
- mission intent
- future trajectory
- formation data
- planner status

## Recommended replacement

將類似：

```text
多機協作一律透過 Agent ↔ Agent 指令，不讓單一 Agent 直接控多台飛控。
```

改為：

```text
多機協作一律透過 ROS2/DDS 的 Agent-level topic/message 交換狀態、
任務意圖、編隊資訊與未來軌跡；任何 Agent 不得直接發布控制命令至
其他 Agent 所屬的飛控。
```

## Add Codex docs entry

建議新增：

```text
- 任何 EGO-Swarm、機群編隊、trajectory planning、obstacle avoidance
  相關工作開始前，先閱讀 `docs/codex/README.md` 及其指定文件。
- `docs/codex/EGO_SWARM_IMPLEMENTATION_GUIDE.md` 定義分階段 scope；
  AI Agent 一次只實作一個已指定 Phase。
```

---

# 2. `/src/AGENTS.md`

## Existing concern

目前將：

```text
ego_swarm/
```

描述成 deterministic candidate baseline，容易被解讀為永久限制。

## Recommended module description

```text
`ego_swarm/` | 現行機群局部規劃核心：共同座標、B-spline、
peer future trajectory、編隊、planner backend、trajectory-level safety。
現有 deterministic candidate planner 保留作 fallback / regression，
允許後續加入 optimized EGO backend。
```

## Add dependency direction

```text
- 現行 trajectory planning 與 formation runtime 皆以 `src/ego_swarm/`
  為主要 domain；legacy `src/trajectory/`、`src/formation/` 不重新接回 runtime。
- obstacle map / sensor-independent obstacle representation 放 `src/perception/`。
```

---

# 3. `/src/nodes/AGENTS.md`

## Existing concern

模組清單偏舊，沒有完整列出現行 EGO planner 與 obstacle simulator。

## Recommended content addition

列出：

```text
- BasicControllerNode
- EgoSwarmPlannerNode
- ObstacleCloudSimulatorNode
```

核心原則補：

```text
- trajectory / swarm / formation 演算法放 `src/ego_swarm/`。
- obstacle-map 演算法放 `src/perception/`。
- Node 只做 ROS2 message、parameter、timer、QoS、資料轉換與模組組裝。
```

---

# 4. `/src/controllers/AGENTS.md`

## Existing status

大方向與新架構一致。

## Recommended additions

```text
- `BasicController` 是現行 runtime 唯一允許發布 PX4
  `TrajectorySetpoint` 的 component。
- Controller 可以執行 planner trajectory 的 p/v/a sampling 與 PX4
  message translation，但不得做 formation assignment、swarm collision
  planning 或 obstacle path planning。
```

---

# 5. `/src/safety/AGENTS.md`

## Existing status

不需大幅修改。

## Recommendation

保持此資料夾聚焦：

- PX4 runtime monitoring
- state translation
- simple flight safety monitors

不要把：

- `ContinuousSafetyValidator`
- `EmergencyBrakePlanner`

強制放進 `src/safety/`。

trajectory-specific safety 建議放：

```text
src/ego_swarm/
```

---

# 6. `/src/formation/AGENTS.md`

## Existing status

與新架構基本一致。

維持：

```text
src/formation/
= legacy / historical pure logic
```

不要新增新的 runtime `FormationManager` 到這裡。

新的：

- FormationEpoch
- FormationAssignment
- FormationSlotPlanner

放 `src/ego_swarm/`。

---

# 7. `/src/trajectory/AGENTS.md`

## Existing conflict

「保持 deterministic candidate baseline 定義」會與未來 optimized backend 直接衝突。

## Recommended replacement

```text
現行局部軌跡規劃一律位於 `src/ego_swarm/`。

legacy `TrajectoryPlanner` / `TrajectoryController` 不得重新接回 runtime。

現有 deterministic candidate planner 必須保留作 fallback 與 regression
baseline，但允許 `src/ego_swarm/` 新增其他 planner backend，例如
`OptimizedEgoPlanner`。
```

---

# 8. `/src/core/AGENTS.md`

## Existing status

保持不變較好。

重要：

不要因新 planner backend 建立：

```text
src/core/ITrajectoryPlanner.hpp
```

應放：

```text
src/ego_swarm/ITrajectoryPlanner.hpp
```

避免擴張飛控 core contract responsibility。

---

# 9. `/AutoTest/AGENTS.md`

## Existing conflict

文件列出 Docker build 前置命令，但 root AGENTS 禁止 AI Agent 自行 build Docker image。

## Recommended clarification

```text
測試所需 Docker image 必須由使用者預先建立。

如果必要 image 不存在，AI Agent 不得自行執行 `docker build` 或
`docker compose build`。應停止需要該 image 的測試步驟，回報 NOT RUN，
並提供使用者應自行執行的 build command。
```

---

# 10. 新增 scoped instructions

必須新增：

```text
src/ego_swarm/AGENTS.md
src/perception/AGENTS.md
```

使用本檔案包中對應版本。

---

# 11. 建議套用順序

```text
1. 新增 docs/codex/*
2. 新增 src/ego_swarm/AGENTS.md
3. 新增 src/perception/AGENTS.md
4. 更新 root AGENTS.md
5. 更新 src/AGENTS.md
6. 更新 src/nodes/AGENTS.md
7. 更新 src/controllers/AGENTS.md
8. 更新 src/trajectory/AGENTS.md
9. 更新 AutoTest/AGENTS.md
10. 不重新啟用 legacy formation/trajectory runtime
```

完成後才開始 Phase 0。
