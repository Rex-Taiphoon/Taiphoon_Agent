# 命令目標基準研究

## 結論

目前命令基準不是單一來源，而是分兩階段：

1. 如果尚未建立任何持久化位置目標，第一次 `MOVE_*` 會以當下自身的 local NED 位置初始化目標。
2. 目標建立後，後續 `MOVE_FORWARD`、`MOVE_BACKWARD`、`MOVE_LEFT`、`MOVE_RIGHT`、
   `MOVE_UP`、`MOVE_DOWN` 都以 planner 保存的 `goal_position_` 為基準累加，不會重新讀取當下
   UAV 的 NED 位置作為新起點。

因此，對一般連續命令而言，目前實際語意是：

```text
第一次相對 MOVE：current local NED → persistent goal_position_
後續相對 MOVE：persistent goal_position_ → 更新後的 goal_position_
```

`BasicController` 不再為這些 MOVE 命令建立第二份位置任務；它只將命令交給
`EgoSwarmPlannerNode`，最後由 planner 發布 `AgentTrajectory`，再由 BasicController
取樣成 PX4 `TrajectorySetpoint`。

## 原始碼依據

### 相對位置命令

核心邏輯在 [`ego_swarm_planner_node.cpp`](../../src/nodes/ego_swarm_planner_node.cpp)：

- `set_goal_from_command()` 先取得當下 local NED `current`。
- 若目前有編隊，基準改為該 Agent 的 formation slot target。
- 若沒有編隊但已有 `has_position_target_`，基準改為保存的 `goal_position_`。
- 只有前兩者都不成立時，才使用當下 `current`。
- 位移計算完成後，結果重新寫回 `goal_position_`，並設定
  `has_position_target_ = true`。

對應程式區段約在 `set_goal_from_command()` 的 `command_base` 選擇與
`goal_position_ = add_vectors(...)` 處理。

### 水平 MOVE 與 yaw

水平 MOVE 不直接使用即時 attitude 重新決定每次方向。程式會保存
`target_yaw_rad_`：

- 第一次水平 MOVE 若沒有 yaw target，才從新鮮 attitude 初始化。
- 後續 `YAW` 會累加到 `target_yaw_rad_`。
- `FORWARD/BACKWARD/LEFT/RIGHT` 使用保存的 target yaw 將 body-frame 位移轉成 NED。
- 既有的目標位置不會因為 UAV 實際 yaw 後續漂移而重新旋轉。

所以例如：

```text
YAW 90
MOVE_FORWARD 30
```

會以保存的 yaw target 90 度把 30 m forward 轉成 NED 位移，再加到保存的
`goal_position_` 上；不是以送出命令當下的實際位置重新計算。

### `MOVE_UP`／`MOVE_DOWN`

這兩個命令同樣先選擇 `command_base`，再只修改 NED Z：

- `MOVE_UP d`：`goal_position_.z -= d`
- `MOVE_DOWN d`：`goal_position_.z += d`

NED 的 Z 軸正方向是向下，因此上升是減少 Z。首次使用時，如果還沒有位置目標，
基準仍是當下 local NED；之後則是保存的目標高度。

## 各命令的基準整理

| 命令 | 目前基準 | 是否保存新目標 |
|---|---|---|
| `MOVE_FORWARD` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `MOVE_BACKWARD` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `MOVE_LEFT` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `MOVE_RIGHT` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `MOVE_UP` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `MOVE_DOWN` | 首次為 current NED；之後為 `goal_position_` | 是 |
| `YAW` | 不改位置；以保存的 target yaw 或新鮮 current attitude 初始化 yaw target | 是 yaw target |
| `MOVE_POINT x y z yaw` | 絕對 local NED 目標 `(x,y,z)` | 是 |
| `FORMATION_CIRCLE` | 使用 formation center 建立 slot targets | 是 formation epoch／slot target |
| `FORMATION_HORI_LINEAR` | 使用 formation center 建立 slot targets | 是 formation epoch／slot target |
| `FORMATION_VERT_LINEAR` | 使用 formation center 建立 slot targets | 是 formation epoch／slot target |

## 編隊命令的特殊規則

`FORMATION_CIRCLE`、`FORMATION_HORI_LINEAR`、`FORMATION_VERT_LINEAR` 是全機廣播命令，
不是單機相對位移：

- 如果已有 accepted formation epoch，新的編隊重配置沿用 parent formation center。
- 初次建立編隊且有保存的 formation center target 時，使用保存中心。
- 若所有成員都有有效的未來 target endpoint，使用成員 target endpoint 的中心。
- 否則初次建立編隊會以當下成員位置的平均值建立中心。
- 每台 UAV 的最終槽位目標是 `formation_center + assigned_slot_offset`。

取消編隊時，程式會把本 Agent 當前的 slot target 複製到 `goal_position_`，再清除
formation mode。這表示：

```text
FORMATION_CANCEL
MOVE_FORWARD 5
```

會從編隊槽位的目標位置延伸 5 m，而不是從取消瞬間的實際 odometry 位置重新計算。

## 與實際飛行位置的差異

`current local NED` 仍然會用於：

- 建立 planner state 的實際起點。
- safety validator 的 live position gate。
- 判斷當前是否已到達 target。
- emergency brake 的煞停起點。

但它不是已建立任務後下一個相對 MOVE 的累加基準。若 UAV 因 tracking lag、煞停或
定位誤差尚未完全抵達目標，下一個相對命令仍會從最後保存的 final target 計算；
這是目前「以目標位置為基準」的核心行為。

## 例外與注意事項

- 如果第一次水平 MOVE 沒有新鮮 attitude，planner 會拒絕命令，不會建立半成品 target。
- `YAW` 只改變 yaw target，不改變位置 target；BasicController 同時保留 PX4 yaw setpoint。
- `MOVE_POINT` 是絕對 NED 目標，不是相對位移。
- Optimized planner 重新規劃、emergency brake 或 HOLD 不會把 final target 靜默改成 current
  position；recovery 完成後仍以原 final target 重試。

