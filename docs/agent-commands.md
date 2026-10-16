# Agent 命令參考

所有 ROS2 命令都必須在 Docker 開發容器內執行。啟動與部署操作請見根目錄 README。

## 命令參考

| 命令 | 說明 | 參數 | 範例 |
|------|------|------|------|
| `MODE_OFFBOARD` | 切換至 Offboard 模式 | 無 | `MODE_OFFBOARD` |
| `MODE_RTL` | 返航至起飛點 | 無 | `MODE_RTL` |
| `MODE_LAND` | 立即降落 | 無 | `MODE_LAND` |
| `HOLD` | 定點懸停 | 無 | `HOLD` |
| `ARM` | 解鎖馬達 | 無 | `ARM` |
| `DISARM` | 上鎖馬達 | 無 | `DISARM` |
| `TAKEOFF` | 起飛至指定高度，無參數時使用預設值 | 高度 (m) | `TAKEOFF 10` |
| `MOVE_FORWARD` | 依命令收到當下機頭方向向前移動 | 距離 (m) | `MOVE_FORWARD 5` |
| `MOVE_BACKWARD` | 依命令收到當下機頭方向向後移動 | 距離 (m) | `MOVE_BACKWARD 3` |
| `MOVE_LEFT` | 依命令收到當下機頭方向向左移動 | 距離 (m) | `MOVE_LEFT 3` |
| `MOVE_RIGHT` | 依命令收到當下機頭方向向右移動 | 距離 (m) | `MOVE_RIGHT 3` |
| `MOVE_UP` | 向上移動 | 距離 (m) | `MOVE_UP 5` |
| `MOVE_DOWN` | 向下移動 | 距離 (m) | `MOVE_DOWN 2` |
| `YAW` | 以目前航向為基準順時針旋轉（直接由 BasicController 發布 PX4 yaw） | 相對角度 (°) | `YAW 90` |
| `MOVE_POINT` | 移動至指定 NED 絕對位置並設定絕對航向 | x y z yaw | `MOVE_POINT 10 5 -15 90` |

### 編隊命令

| 命令字串 | 參數格式 | 說明 |
|----------|----------|------|
| `FORMATION_CIRCLE <半徑> <高度>` | `param_value`: 半徑 (m), `param2`: 高度 (m) | 環形編隊，等角分布於質心周圍 |
| `FORMATION_HORI_LINEAR <間距> <高度>` | `param_value`: 間距 (m), `param2`: 高度 (m) | 水平直列，沿 NED Y 軸排列 |
| `FORMATION_VERT_LINEAR <間距>` | `param_value`: 間距 (m) | 垂直直列，沿 NED Z 軸排列 |
| `FORMATION_V_SHAPE <間距> <高度>` | `param_value`: 翼上間距 (m), `param2`: 高度 (m) | 燕行（V 字），頂點朝 North、兩翼 45° 向後，中心為各槽位平均 |
| `FORMATION_CANCEL` | 無參數 | 取消編隊，停止發布 NED 目標 |

全體 `/uav_all_agent/command` 的 `MOVE_FORWARD/BACKWARD/LEFT/RIGHT/UP/DOWN` 只在編隊
啟用時有效，方向固定為 `+North/-North/+East/-East/-Down/+Down`；各 Agent 在自己的
`planner_ned` 套用同一個、與原點無關的位移，再以 WGS84 FormationEpoch 交換 center。
全體 `MOVE_POINT` 仍保留既有絕對 NED 行為：每個收到命令的 Agent 會用自身
`planner_ned` 解讀 x/y/z，因此在 Home 不同時不具唯一的全球目標語意；修正此契約會
牽涉命令架構與 runtime 行為，本次不更動。單機 MOVE 語意維持不變。

**使用範例**：

```bash
# 環形編隊（半徑 20m，高度 30m）
docker compose exec uav_1_agent bash -c \
  "ros2 topic pub --times 5 /uav_all_agent/command std_msgs/msg/String \
   'data: FORMATION_CIRCLE 20 30'"
```

> 編隊命令必須透過 **`/uav_all_agent/command`** 廣播；Planner 會以 Epoch 去重，
> 相同 config 與成員 snapshot 的重複命令不會建立新 epoch；單機 MOVE 或取消編隊後重送
> 相同編隊命令會重新啟用既有 Epoch，保留 center 與 slot assignment。
>
> `<高度>` 以 Home 為基準：第一次編隊若成員已先移動，中心只取成員目標的水平平均，
> 高度仍為 Home，不會疊加目前飛行高度。Agent 關閉後存活成員會自動重組；飛行中重新
> 啟動的 Agent 需再次發送 `FORMATION_*` 才會加入，其他成員期間維持目前編隊。


### 簡寫命令

部分移動指令支援簡寫形式：

| 完整形式 | 簡寫形式 |
|----------|----------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

### EGO-Swarm 軌跡與舊 NED 目標 topic

`/{agent_namespace}/planning/trajectory` 是 EGO-Swarm 規劃結果的目標 Topic，供
`BasicController` 以 analytic B-spline derivative 取樣 p/v/a 後發布 PX4
`TrajectorySetpoint`。舊的
`/uav_{N}_agent/ned_target` 僅保留相容或純邏輯測試用途：

EGO-Swarm Planner 另訂閱下列本機任務意圖 Topic：

```text
/{agent_namespace}/planning/mission_intent
std_msgs/msg/String
QoS: reliable, depth 10
```

`/{agent_namespace}/command` 的 `MOVE_*` 與 `MOVE_POINT` 由 EGO-Swarm Planner
唯一建立位置目標；BasicController 不再轉發第二份 mission intent。水平 MOVE 會在
第一次需要建立 target yaw 時使用新鮮 AgentAttitude，之後以保存的 target yaw 與
goal position 累加相對位移；飛行途中實際 yaw 改變不會旋轉既有目標。`YAW` 的相對角度
也累加到保存的 target yaw；`MODE_*`、`ARM`、`TAKEOFF`、`LAND` 仍由 BasicController
執行；`planning/mission_intent` 僅保留為相容的獨立輸入。

到達目標後，Planner 會以目標座標填滿固定 hold B-spline，並發布
`No mission target; BasicController keeps its hold setpoint`。軌跡逾時後，
BasicController 仍鎖定最後控制點，不會改用漂移中的即時 odometry 作為新目標。

```bash
# 手動發布 NED 目標（Debug）
ros2 topic pub --once /uav_1_agent/ned_target taiphoon_agent/msg/NedTarget \
  '{x: 10.0, y: 5.0, z: -15.0, yaw: 90.0}'
```

> `yaw` 欄位若設為 `NaN`（預設值），表示不控制偏航，由飛控自行維持目前朝向。

### AgentTrajectory 的 `control_points`

`AgentTrajectory.control_points` 是 `float64[]` 扁平陣列，使用 WGS84 geodetic
座標。每 3 個連續元素代表一個控制點
`[latitude_deg, longitude_deg, height_m]`，排列格式為：

```text
[lat0, lon0, h0, lat1, lon1, h1, ..., latN, lonN, hN]
```

陣列長度必須是 3 的倍數；不可依賴巢狀陣列結構。
`coordinate_frame` 固定為 `COORDINATE_FRAME_WGS84_GEODETIC`，
`frame_revision` 是全球座標 contract 版本，不是各 Agent 的 local Home 原點。
目前 runtime 固定發布 10 個控制點；接收端會以自己的固定 Home 將控制點轉成公尺制 local NED，再執行 B-spline
取樣、動力學限制與同儕避碰。完成任務的 Agent 會持續發布
`terminal_hold=true` 的固定終點軌跡 heartbeat，直到超過 peer timeout，讓其他
Agent 仍把該終點視為佔位障礙。這類待命 heartbeat 只發布到 global
`/uav_all_agent/swarm/agent_trajectories`，不會送入本機
`/{agent_namespace}/planning/trajectory`；只有本機需要執行的移動、煞車與任務終點 hold
才會交給 BasicController 取樣成 PX4 `TrajectorySetpoint`。

---
