# AGENTS.md — src/perception

## 目的

`src/perception/` 提供 obstacle representation 與 sensor-independent LocalObstacleMap。

目標是讓目前 PX4 SITL 模擬資料與未來真實 LiDAR / Depth Camera 共用相同 planner interface。

---

## 責任

本模組負責：

- obstacle geometry
- normalized obstacle data
- LocalObstacleMap abstraction
- raw point map
- future voxel / spatial map
- synthetic sensor geometry
- PointCloud data adapter
- obstacle distance / collision query

---

## 核心原則

### 1. Planner 不知道 Sensor Source

下列來源都應轉成相同 map contract：

```text
explicit_empty
ground_truth
synthetic_sensor
LiDAR
Depth Camera
```

### 2. Pure Map Core

LocalObstacleMap 的核心 query 邏輯優先保持 pure C++。

若需要 ROS2 `sensor_msgs/msg/PointCloud2`：

- ROS message conversion 透過 Adapter
- 不讓 map query class 依賴 ROS Node lifecycle

### 3. Empty Map ≠ Missing Map

必須明確區分：

```text
fresh valid empty environment
```

與：

```text
no map / stale map
```

禁止把「沒有收到 obstacle message」解讀成安全空間。

### 4. Synthetic Sensor Pose 必須跟 UAV 移動

Sensor emulation 的：

- position
- heading
- pitch

必須從 vehicle pose 更新。

禁止固定在不隨 UAV 移動的場景原點。

### 5. Map Freshness

Obstacle map 必須能提供：

- timestamp
- valid
- freshness

Planner 依 policy 決定 stale map 的 fail-safe。

### 6. 不做 Trajectory Planning

本目錄不得負責：

- formation assignment
- peer trajectory planning
- EGO optimization
- PX4 control

---

## Simulation Sources

至少保留：

### explicit_empty

fresh valid zero-obstacle map。

### ground_truth

由 YAML / simulation geometry 建立完整障礙物。

### synthetic_sensor

根據 vehicle pose、range、FOV、noise、dropout 建立有限視野障礙物。

---

## Future Real Sensor

新增：

- LiDAR Adapter
- Depth Camera Adapter

時應輸出相同 normalized map input。

若接真實 sensor 必須修改 planner core，先回報 architecture defect。

---

## Testing

單元測試放：

```text
test/
```

至少測：

- empty
- missing
- stale
- invalid point
- NaN / Inf
- box
- cylinder
- wall
- range
- FOV
- moving pose
- dropout
- deterministic seed

---

## 禁止事項

- 不在 perception 中直接發布 PX4 setpoint
- 不建立 formation logic
- 不把 simulator-specific geometry 直接暴露給 planner
