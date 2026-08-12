# PlanningSnapshot 設計基線

## 文件狀態

本文件是 Phase 0 的設計基線，不代表已新增 `PlanningSnapshot` runtime
型別。它先固定一次 planner cycle 應該使用的資料邊界，供後續 Phase 在不改變
現有控制資料流的前提下實作。

## 目的

EGO-Swarm planner 必須對同一個時間點的自身狀態、目標、共同座標、同儕未來
軌跡與障礙物資料做決策。ROS2 callback 可能在 planner 執行期間更新成員資料，
因此收集資料與執行規劃之間需要明確的 freeze seam：

```text
ROS2 callbacks
    ↓
collect and validate
    ↓
PlanningSnapshot（單一 cycle 的唯讀資料）
    ↓
planner backend
    ↓
trajectory result → BasicController → PX4
```

## Snapshot 欄位契約

| 欄位 | 內容 | Phase 0 現況 |
|---|---|---|
| `self_state` | position、velocity、acceleration、timestamp | Node 目前只提供有效 GPS 轉換後的 position；速度、加速度尚未進入 EGO planner contract |
| `local_goal` | swarm_ned 目標位置 | 由 `NedTarget`／mission intent 或編隊槽位即時計算 |
| `frame` | origin、frame revision、有效性 | `SwarmFrame` 與 Node 成員保存；Node planner cycle 前更新 |
| `peers` | peer trajectory、start time、有效性 | `PeerTrajectoryTable` 保存最後有效軌跡，尚無 receive time／expiration |
| `obstacles` | frame、點列表、有效性、時間 | Node 直接保存 raw `vector<Vector3>`；沒有 cloud 時傳入空列表 |
| `planning_time` | 此 cycle 的共同時間 | Node 使用 planner loop 當下的 ROS time |

後續實作應使 planner backend 只讀取 snapshot，不在計算過程重新讀取 ROS2
callback 修改中的資料。Snapshot 不應包含 publisher、subscription、ROS2 Node
或 PX4 message。

## 驗證順序

1. 檢查自身位置、目標、frame、peer trajectory 與 obstacle points 是否有限且
   屬於相同 `swarm_ned` frame。
2. 凍結上述資料與 `planning_time`。
3. 呼叫 deterministic planner 或未來的其他 backend。
4. 對輸出 trajectory 做共同的 safety validation（後續 Phase）。
5. 只將結果交給 `BasicController`；planner 不直接發布 PX4 setpoint。

## Phase 0 決策

- 本 Phase 只記錄 seam 與現有欄位映射，不加入 snapshot 類別或改變 planner
  signature。
- `BasicController` 仍是唯一 `TrajectorySetpoint` publisher。
- `PeerTrajectoryTable` 的 lifecycle、obstacle map abstraction、p/v/a state 與
  continuous safety validation 分別留在後續 Phase。

