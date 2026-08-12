# EGO-Swarm Current Behavior Baseline

## 文件狀態

這是以 Phase 0 characterization baseline 為起點、並已納入 Phase 1 變更的
目前行為摘要。內容以目前 source code 與 Docker 內測試為準，不把尚未實作的
後續安全契約描述成 runtime 保證。

## Runtime data flow

```text
AgentState / GPS / mission intent / PointCloud2 / peer AgentTrajectory
                         ↓
              EgoSwarmPlannerNode callbacks
                         ↓
      swarm_ned position + fresh LocalObstacleMap + peer table
                         ↓
              EgoSwarmPlanner::plan
                         ↓
        AgentTrajectory（valid 或 current-position hold）
                         ↓
       BasicController timed B-spline sampling
                         ↓
          PX4 TrajectorySetpoint（唯一發布端）
```

## Pure C++ components

### `BsplineTrajectory`

- 僅接受至少四個有限控制點與正的 knot interval。
- 使用均勻 clamped cubic B-spline；`knot_interval_s` 是最小基準 knot
  間隔，實際 duration 由候選路徑長度、`max_velocity_m_s` 與
  `max_acceleration_m_s2` 計算。
- `evaluate()` 對有限的負時間與超過 duration 的時間夾到兩端點。
- 無效軌跡或非有限取樣時間回傳 `is_valid == false`。
- Phase 0 未加入 velocity、acceleration 或 jerk derivative。

### `PeerTrajectoryTable`

- `update()` 只接受有效 B-spline、`frame_revision`、`message_time_s`、
  `start_time_s`、`receive_time_s` 與 `end_time_s`，並檢查共同時間偏差。
- 每筆 record 保存 `agent_id`、`trajectory_id`、`frame_revision`、message／start／
  receive／end time；同一 Agent 的 trajectory ID 不可倒退。
- `get_status()` 明確回傳 `ACTIVE`、`STALE`、`MISSING` 或 `INVALID`；`prune()`
  清除非 ACTIVE record。
- planner 只取得 ACTIVE 軌跡；若候選軌跡時間已超過 peer 的 `end_time_s`，
  不再把 peer 的最後 endpoint 當成 ghost UAV。

### `SwarmFrame`

- 使用 WGS84 局部近似將 GPS 轉換成 `swarm_ned`（North、East、Down）。
- `set_origin()` 第一次成功後 freeze；不同 origin 會被拒絕，並以 `frame_revision()`
  的非零值識別同一個座標 lifetime。
- `are_origins_compatible()` 使用緯／經度 `1e-9` 度、高度 `1e-3` 公尺容差，
  供 Node 拒絕不同 frame origin 的 peer trajectory。

### `FormationSlotPlanner`

- 將 Agent ID 排序後以 rank 產生 CIRCLE、HORI_LINEAR、VERT_LINEAR 槽位。
- 對重複 ID、空成員及非法幾何參數回傳失敗。
- `are_slots_safe()` 使用機體中心三維距離與呼叫端提供的門檻。
- 尚無 persistent assignment 或 FormationEpoch。

### `EgoSwarmPlanner`

- 保留 deterministic candidate baseline：直線、左右側向、上下垂直共五種
  中段偏移候選。
- 依目標距離、控制點平滑性、障礙物距離與 peer 距離成本選擇候選。
- 每個候選以固定 sample interval 評估；任一 sample 低於 obstacle clearance
  或 peer collision distance 即拒絕。
- 沒有安全候選時回傳由目前位置構成的有限 hold B-spline，但 `is_valid` 為
  false、`is_hold` 為 true。
- `EgoSwarmPlanner` 只依賴 `IObstacleMap`；目前 backend 是
  `RawPointObstacleMap`。只有 `explicit_empty` source 產生的 valid empty map
  代表沒有障礙物。

## ROS2 assembly behavior

### `EgoSwarmPlannerNode`

- 依 `swarm_origin_agent_index`（預設 Agent 1）等待指定 origin 的有效 GPS；
  第一次建立後 freeze，不因 origin 移動或暫時離線而改選其他 Agent，並以
  `frame_revision` 識別同一個座標生命週期。
- peer `AgentTrajectory` 必須通過 agent ID、valid、order、frame revision、
  frame origin、knot／duration、控制點長度、有限值、timestamp sanity、freshness
  與 trajectory ID 單調性檢查；逾時或結束的資料不再參與 planner。
- obstacle callback 只接受 `header.frame_id == "swarm_ned"`、合法 timestamp 且所有
  x/y/z 有限的 PointCloud2；成功收到空 cloud 會建立 valid empty map。
- planner loop 以 `obstacle_cloud_timeout_s` 區分 `missing`、`fresh` 與 `stale`。
  尚未收到或已過期的 map 都會阻擋新的移動規劃並發布 HOLD status，不會被視為空
  障礙物；`require_obstacle_cloud` 只影響降級原因文字，Phase 2 維持 fail-closed。
- `obstacle_cloud_simulator` 的 `synthetic_sensor` 從 AgentState／AgentAttitude 更新
  UAV 在 `swarm_ned` 的 pose；尚未取得 pose 時不發布固定原點資料。
- planner result 發布至 `/fmu_all_agent/swarm/agent_trajectories` 與本機
  `/planning/trajectory`；status 發布至本機 `/planning/status`。
- `/{agent_namespace}/command` 中的 `MOVE_*`、`MOVE_POINT` 與
  `FORMATION_CANCEL` 會交由 Planner 設定目標；`MODE_*`、`ARM`、`TAKEOFF`、
  `LAND`、`YAW` 仍只由 BasicController 執行。
- active trajectory 重發時保留原始 `start_time` 與 `trajectory_id`，直到
  軌跡完成或目標／座標生命週期改變，避免每個 planning cycle 重置軌跡進度。
- Planner 同時以 PX4 local NED 與 GPS 轉換出的 common NED 判斷到達；兩者都在
  各自容許誤差內時，以目標座標填滿控制點發布一次固定 hold trajectory、清除
  任務並停止新的移動軌跡。status reason 為
  `No mission target; BasicController keeps its hold setpoint`；軌跡逾時後
  BasicController 仍使用最後控制點，不回退至即時 odometry。
- 沒有 mission goal 或 formation goal 時，planner 只發布 HOLD status，不發布
  `AgentTrajectory`；BasicController 保留其固定位置／高度 setpoint。

### `BasicController`

- 是目前唯一發布 PX4 `TrajectorySetpoint` 的 runtime component。
- 接收自身 `AgentTrajectory` 後取樣 position；frame revision 改變時先清除舊
  軌跡，必須收到新 revision 的下一筆軌跡才重新啟用。
- timed trajectory 尚未開始、取樣失敗或已過期時，發布目前位置的 finite hold
  position。
- 目前 `publish_control_commands()` 寫入 position 與 yaw；velocity、
  acceleration、jerk、yawspeed 仍維持 PX4 ROS message 建構時的 finite zero
  預設值，尚未提供 feed-forward derivative。

## Baseline characterization tests

Phase 0 鎖定下列行為：

- B-spline endpoint、時間夾限與 invalid input。
- peer future trajectory 在目前位置造成距離衝突時，所有候選被拒絕並降為 hold。
- 不同 frame origin 被視為不相容；BasicController frame revision 變更會清除舊軌跡。
- 過期 timed trajectory 產生 finite current-position hold。
- PX4 `TrajectorySetpoint` 的 position、velocity、acceleration、jerk、yaw 與
  yawspeed 欄位均為有限值。
- 空障礙物輸入目前代表無障礙物；既有 PointCloud geometry、range/FOV、noise 與
  dropout 測試維持通過。
