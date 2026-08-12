# AGENTS.md — src/ego_swarm

## 目的

`src/ego_swarm/` 是 Taiphoon_Agent 現行機群局部規劃與 trajectory-safety domain。

此目錄中的核心類別必須保持 pure C++，讓：

- unit test
- PX4 SITL
- future real-sensor deployment

共用相同規劃邏輯。

---

## 責任

本模組負責：

- `SwarmFrame`
- planning state / snapshot
- B-spline trajectory representation
- trajectory derivatives
- peer future trajectory lifecycle
- formation epoch
- formation slot geometry
- formation assignment
- deterministic planner backend
- optimized planner backend
- dynamic feasibility
- trajectory-level peer collision checking
- continuous trajectory safety validation
- emergency braking trajectory

---

## 核心原則

### 1. Pure C++

本目錄的演算法核心不得：

- include `rclcpp`
- 建立 ROS publisher/subscriber
- 依賴 ROS Node lifecycle
- 直接發布 PX4 message

ROS2 message conversion 放 `src/nodes/` 或明確 Adapter。

### 2. Planner 不控制 PX4

本目錄只產生 trajectory / planner result。

唯一 PX4 TrajectorySetpoint runtime publisher 是 `BasicController`。

### 3. Sensor-agnostic

Planner 不得直接依賴：

- LiDAR driver
- depth camera driver
- camera optical frame
- simulator-specific world API

障礙物透過 `src/perception/` 的 LocalObstacleMap abstraction 使用。

### 4. Deterministic backend 是 fallback，不是永久限制

目前 deterministic candidate planner 必須保留：

- regression baseline
- fallback
- simple open-space swarm test

但允許新增：

- `ITrajectoryPlanner`
- `OptimizedEgoPlanner`
- topological initializer

不得因舊文件的 deterministic baseline 描述而阻止新 backend。

### 5. Planner backend interface 留在本 domain

若建立：

```text
ITrajectoryPlanner
```

放在：

```text
src/ego_swarm/
```

不要為此擴張 `src/core/` 飛控介面層。

### 6. Formation 使用 Epoch，不新增重疊 State Machine

現行 formation runtime 應使用：

- `FormationEpoch`
- persistent assignment
- reconfiguration policy
- derived convergence status

除非人類開發者重新核准，不新增：

```text
IDLE / PREPARING / TRANSITIONING / STABLE / ...
```

這類獨立 Formation FSM。

### 7. Safety Validator 獨立於 Planner Backend

任何 planner backend result 必須可以被共同：

```text
ContinuousSafetyValidator
```

驗證。

不得讓 optimized planner 自己的 cost 被視為最終 safety guarantee。

### 8. Emergency Stop 必須考慮動態狀態

UAV 仍有顯著 velocity 時：

不得以 repeated-current-position control points 取代 braking trajectory。

### 9. Freshness 是 safety contract

peer / frame / obstacle data 的：

- timestamp
- receive time
- expiration
- validity

必須有明確 contract。

禁止 silent reuse 不確定資料。

---

## Formation

必須支援：

- CIRCLE
- HORI_LINEAR
- VERT_LINEAR

Circle 為必要功能。

Agent-ID rank assignment 可保留作 fallback。

正式 transition 後續優先使用 persistent / minimum-cost slot assignment。

---

## Testing

所有 unit tests 放 repository 根目錄：

```text
test/
```

`src/` 內禁止新增測試原始碼。

任何 safety-critical 演算法至少測：

- nominal
- boundary
- NaN / Inf
- stale data
- timeout
- failure recovery

---

## Documentation

修改以下 contract 時同步更新：

- `docs/codex/EGO_SWARM_ARCHITECTURE.md`
- `docs/taiphoon-agent-interface.md`（若影響對外 interface）
- `README.md`（若影響操作方式）
- 對應 message 文件

---

## 禁止事項

- 不重新啟用 legacy `MissionPlanner`
- 不重新啟用 legacy `TrajectoryController`
- 不直接 copy 大段 upstream GPL implementation
- 不把 ROS2 callback / topic logic 塞進 pure planner class
- 不建立第二個 PX4 TrajectorySetpoint publisher
