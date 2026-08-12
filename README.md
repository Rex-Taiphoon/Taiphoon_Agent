# Taiphoon_Agent

**無人機任務與感測器執行中樞**，基於 ROS2 (Humble) + PX4 的自主飛行控制系統，
以 EGO-Swarm 實作多機編隊、局部障礙物避讓與同儕未來軌跡避碰，並支援邊緣運算與即時影像處理。

> 本 README 已反映 EGO-Swarm 目標架構。Phase 0 只補充 characterization tests、
> 設計基線與現況文件，不新增 ROS2 topic／message 或改變 PX4 runtime 資料流。
> 三個 EGO-Swarm 訊息的欄位以目前 workspace 的 `.msg` 定義為準；Node 的啟動方式
> 與 QoS 以本文件及介面規格書列出的實作為準。

EGO-Swarm 的 Phase 0 基線請參閱 [`docs/codex/README.md`](docs/codex/README.md)；其中
[`EGO_SWARM_CURRENT_BEHAVIOR.md`](docs/codex/EGO_SWARM_CURRENT_BEHAVIOR.md) 記錄目前
runtime 行為與已知限制。

---

## 快速開始（Docker）

```bash
# 1. 建構映像
docker build -f docker/Dockerfile.uxrce-agent -t taiphoon/uxrce-agent:latest .
docker compose build

# 2. 拉取 PX4 SITL 映像
docker pull px4io/px4-sitl:latest

# 3. 建立網路
docker network create Taiphoon_Net

# 4. 啟動開發容器
docker compose up -d fmu1_agent

# 5. 啟動模擬器
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4

# 6. 編譯 Taiphoon_Agent
docker compose exec fmu1_agent bash -c \
  "cd /home/taiphoon/Taiphoon_Agent && \
   colcon build --symlink-install --packages-select taiphoon_agent"

# 7. 快速起飛（使用 CycloneDDS）
bash AutoTest/scripts/quick_takeoff.sh
```

> **實機模式**：加上 `--profile hw` 啟動 uXRCE-DDS Agent 常駐服務。
>
> ```bash
> docker compose --profile hw up -d
> ```

## 系統需求

- Docker + Docker Compose
- macOS / Linux（Windows 需 WSL2）
- 至少 8GB 可用記憶體

> **AgentState 介面相容性**：`taiphoon_agent/msg/AgentState` 是 Agent 與 GCS 共用的
> ROS2 message。欄位有異動時，Agent 與 GCS 都必須重新建置，並使用同一版本的
> message 定義，否則 DDS 型別可能不相容而無法正確交換狀態。

---

## 操作手冊 — 基本控制

所有 ROS2 指令在 Taiphoon_Agent 容器內執行。建議先進入容器再執行指令：

```bash
docker compose exec fmu1_agent bash
```

進入後需先載入環境（每次開啟新終端都需要）：

```bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
```

### 1. 模擬器管理

```bash
# 啟動四台模擬器（預設）
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4

# 指定 Docker 網路
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --net Taiphoon_Net

# 使用 CycloneDDS 啟動
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --net Taiphoon_Net --rmw cyclonedds

# 關閉所有模擬器
bash AutoTest/scripts/px4-sitl-launcher/stop_all.sh
```

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--count N` | 無人機數量 | 4 |
| `--net NAME` | Docker 網路名稱 | Taiphoon_Net |
| `--rmw TYPE` | DDS 中介軟體（fastrtps / cyclonedds） | fastrtps |

每台無人機對應兩個容器：`drone{N}`（PX4 SITL）與 `drone{N}-uxrce-dds-agent`（uXRCE-DDS Agent）。命名空間 `fmu{N}`、MAV_SYS_ID 與通訊埠由 `--count` 自動遞增。

### 2. 啟動 Taiphoon_Agent（容器內）

先進入容器再載入環境，然後 Launch：

```bash
docker compose exec fmu1_agent bash

# 在容器內：
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=1
```

啟動後 Agent 會自動切換至 Offboard 模式（可由 `auto_start_sequence` 參數控制開關）。

Launch 參數：

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `drone_count` | int | 4 | 無人機數量；多機時由 EGO-Swarm 規劃器共同處理編隊 |

EGO-Swarm 目標執行鏈中，每台無人機包含：
- **BasicController** — 單機飛行控制與唯一 PX4 `TrajectorySetpoint` 輸出端。
- **ego_swarm_planner** — GPS common frame、編隊槽位、PointCloud2 與同儕未來軌跡的 deterministic candidate baseline 局部規劃節點。
- **obstacle_cloud_simulator** — 依 YAML 產生 PointCloud2 障礙物資料的選配模擬節點。

兩個 Node 已納入 `basic_controller.launch.py`；EGO planner 固定啟動，障礙物模擬器由
`enable_obstacle_cloud_simulator` 控制。

### 3. 設定飛行模式

```bash
# 切換至 Offboard 模式
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'

# 切換至 Hold 模式（定點懸停）
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: HOLD'

# 切換至 Land 模式（立即降落）
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_LAND'
```

`TAKEOFF` 會在命令接受時鎖定當下的 NED X/Y 與目標高度；`HOLD` 會鎖定命令接受時的
NED 三軸位置。後續 odometry 更新只用於觀測，不會把即時估測位置改寫成新的懸停目標。

全體廣播（所有 Agent 同時執行）：

```bash
ros2 topic pub --once /fmu_all_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'
```

### 4. 解鎖指令

```bash
# 建議發送 5 次以確保指令送達
ros2 topic pub --times 5 /fmu1_agent/command std_msgs/msg/String 'data: ARM'
```

> 注意：需先在 Offboard 模式下發送 OffboardControlMode 心跳，PX4 才允許解鎖。

### 5. 上鎖指令

```bash
ros2 topic pub --times 5 /fmu1_agent/command std_msgs/msg/String 'data: DISARM'
```

### 6. 移動指令

```bash
# 向前移動 5 公尺
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_FORWARD 5'

# 向後 3 公尺
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_BACKWARD 3'

# 向左 3 公尺
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_LEFT 3'

# 向右 3 公尺
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_RIGHT 3'

# 向上 5 公尺
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_UP 5'

# 向下 2 公尺（最低不低於 1m）
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_DOWN 2'

# 順時針旋轉 90 度
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: YAW 90'
```

> 移動指令基於 NED 座標系（x=前、y=右、z=下），從當前位置出發。
> MOVE_UP / MOVE_DOWN / YAW 需在飛行狀態（HOVER / TAKEOFF / MOVING）下執行。

### 7. 返航指令

```bash
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_RTL'
```

> RTL（Return to Launch）會讓無人機自動返回起飛點並降落。

---
---

## 操作手冊 — Agent 容器失效模擬

`agent_supervisor` 提供單一 Agent 的 shutdown service，可刻意模擬飛行電腦失效：

```bash
ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"
```

> **高風險操作**：`data` 必須為 `true`。服務接受請求並送出回應後，會以 SIGTERM 觸發 Agent 容器 PID 1 立即退出；不檢查飛行狀態、不執行 RTL/LAND，也不提供 `/fmu_all_agent/shutdown` 廣播。後續行為完全由 PX4 failsafe 決定。

若 Compose 服務設定為 `restart: no`，PID 1 終止後容器會保持停止，不會自動重新啟動；若使用自動重啟策略，容器可能重新啟動，無法保持失效狀態。

服務與容器狀態驗證：

```bash
# 在 Agent 容器內確認 service
ros2 service list | grep -F /fmu1_agent/shutdown
ros2 service type /fmu1_agent/shutdown
```

```bash
# 確認完成後，在另一個終端執行；回應送出後容器會終止
docker compose exec fmu1_agent bash -c \
  'ros2 service call /fmu1_agent/shutdown std_srvs/srv/SetBool "{data: true}"'
```

```bash
# 回到宿主機的另一個終端執行
docker compose ps fmu1_agent
docker inspect --format '{{.State.Status}} exit={{.State.ExitCode}} restart={{.RestartCount}}' fmu1_agent
```

---

## 操作手冊 — 全體控制起飛（CycloneDDS）

在宿主機直接執行完整起飛流程，適用於已啟動 SITL 模擬與 Agent 容器的環境：

```bash
# 1. OFFBOARD 模式
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --times 3 --rate 1 \
    /fmu_all_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'"

sleep 3

# 2. 解鎖
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --times 3 --rate 1 \
    /fmu_all_agent/command std_msgs/msg/String 'data: ARM'"

sleep 2

# 3. 起飛 8m
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --times 3 --rate 1 \
    /fmu_all_agent/command std_msgs/msg/String 'data: TAKEOFF 8.0'"

# 4. 左右移動
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --once \
    /fmu_all_agent/command std_msgs/msg/String 'data: MOVE_LEFT 5.0'"

# 5. 上下移動
docker exec fmu1_agent bash -c "
  ros2 topic pub --wait-matching-subscriptions 3 --once \
    /fmu_all_agent/command std_msgs/msg/String 'data: MOVE_UP 5'"
```

> 也可直接執行一鍵腳本：`bash AutoTest/scripts/quick_takeoff.sh`

---

## 操作手冊 — Offboard 直接控制 Topic

進入 Offboard 模式後，可直接對 PX4 內部 topic 發送控制指令：

```bash
# 位置、速度、加速度、姿態等控制，使用 NED 坐標系
/fmu1/fmu/in/trajectory_setpoint

# 設定姿態，使用 FRD 坐標系
/fmu3/fmu/in/vehicle_rates_setpoint

# 設定姿態四元數
/fmu3/fmu/in/vehicle_attitude_setpoint_v1
```

> 這些是 PX4 uXRCE-DDS 橋接後的內部 topic，直接控制飛控行為。
> 建議先透過 Agent 命令層（`/fmu{N}_agent/command`）操作，確有必要時再使用直接控制。

---

## 操作手冊 — EGO-Swarm 編隊飛行

編隊功能由 **ego_swarm_planner** 實作。每台無人機在 GPS common frame 中計算自身
的虛擬編隊槽位，再由 EGO-Swarm 局部規劃器產生包含障礙物與同儕避碰的未來軌跡。
既有 `/fmu_all_agent/command` 編隊命令協定保持不變。

### 啟動多機編隊環境

開啟三個終端視窗分別執行：

```bash
# 終端 1：啟動 4 台模擬器
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --net Taiphoon_Net
```

```bash
# 終端 2：啟動 Taiphoon_Agent（目標為 4 組 BasicController + ego_swarm_planner）
docker compose exec fmu1_agent bash
# 在容器內：
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4
```

```bash
# 終端 3：進入容器，手動下達指令
docker compose exec fmu1_agent bash
# 在容器內：
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
```

### 標準編隊流程

在終端 3 依序執行：

```bash
# 1. 解鎖（發送 5 次確保送達）
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: ARM'

# 2. 起飛至 15m
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: TAKEOFF 15'

# 3. 環形編隊（半徑 10m，高度 10m）
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: FORMATION_CIRCLE 10 10'

# 4. 水平直列編隊（間距 5m，高度 10m）
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: FORMATION_HORI_LINEAR 5 10'

# 5. 垂直直列編隊（間距 5m）
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: FORMATION_VERT_LINEAR 5'

# 6. 取消編隊
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: FORMATION_CANCEL'
```

### 編隊行為說明

| 編隊類型 | 描述 | 離線容錯 |
|----------|------|----------|
| **環形** | 所有無人機等角分布在圓周上，質心為共同座標中的編隊參考點 | EGO-Swarm 依目前 AgentState 重新計算槽位 |
| **水平直列** | 沿 Y 軸（右向）等間距排列 | EGO-Swarm 依目前 AgentState 重新計算槽位 |
| **垂直直列** | 沿 Z 軸（向下）等間距排列 | EGO-Swarm 依目前 AgentState 重新計算槽位 |

> 編隊運算以 **GPS common frame** 為共同基準：收到 `FORMATION_*` 指令後，
> `ego_swarm_planner` 為每台 Agent 分配虛擬槽位，各 Agent 自行規劃前往自己的槽位。
> GPS-only 可以正常執行編隊；PX4 odometry 與 attitude 仍由 BasicController 用於
> 即時軌跡追蹤。EGO planner 每 3 秒掃描同儕 Agent，依目前成員更新槽位。

---

## EGO-Swarm 編隊與避障

EGO-Swarm 以本機局部規劃器同時處理編隊槽位、靜態障礙物與同儕未來軌跡。舊的
`MissionPlanner`、`TrajectoryController` 與第一版觀測式防撞已被取代；舊
`/ned_target`、`/waypoints` 僅保留相容或純邏輯測試用途。

實際控制資料流如下：

```text
GCS 命令
    │
    ▼
ego_swarm_planner
    ├── GPS common frame／虛擬編隊槽位
    ├── PointCloud2 局部障礙物
    ├── 同儕 AgentTrajectory
    └── 本機 AgentTrajectory
              │
              ▼
       BasicController
              │
              ▼
       PX4 TrajectorySetpoint
```

- 同儕避碰距離是機體中心到機體中心的三維直線距離，不加計機體半徑。
- 最小距離不可硬編碼為 3 m；Launch 與 entrypoint 以
  `formation_min_distance_m`、`peer_collision_min_distance_m` 提供設定，可使用
  1.5 m、2 m 或 3 m 等值。
- 短暫缺少同儕未來軌跡時，沿用最後一份有效軌跡，且不因資料年齡擴張安全半徑。
- `BasicController` 是唯一發布 PX4 `TrajectorySetpoint` 的節點。

### PointCloud2 障礙物模擬

`obstacle_cloud_simulator` 讀取 YAML 幾何場景，發布：

```text
/{agent_namespace}/perception/obstacles
sensor_msgs/msg/PointCloud2
```

目標支援 `box`、`cylinder`、`wall`，以及 `ground_truth_map`、`sensor_emulation`。
可直接從 [`config/obstacles/default.yaml`](config/obstacles/default.yaml) 複製場景再
修改；Launch 可使用 `enable_obstacle_cloud_simulator:=true` 啟動模擬器：

```yaml
frame_id: swarm_ned
mode: sensor_emulation
sample_spacing_m: 0.5
publish_rate_hz: 15.0
random_seed: 20260810

sensor:
  range_m: 20.0
  horizontal_fov_deg: 270.0
  vertical_fov_deg: 90.0
  noise_std_m: 0.02
  dropout_ratio: 0.05

obstacles:
  - type: box
    center_ned: [12.0, 0.0, -3.0]
    size_m: [4.0, 8.0, 6.0]
  - type: cylinder
    center_ned: [22.0, 5.0, -3.0]
    radius_m: 2.0
    height_m: 6.0
  - type: wall
    center_ned: [32.0, 0.0, -3.0]
    size_m: [0.5, 20.0, 6.0]
```

PointCloud2 與 EGO-Swarm 規劃資料使用共同 `swarm_ned` 座標；障礙物 Publisher 與
EGO planner Subscriber 均使用 best_effort QoS depth 10。

---

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
| `MOVE_FORWARD` | 向前移動 | 距離 (m) | `MOVE_FORWARD 5` |
| `MOVE_BACKWARD` | 向後移動 | 距離 (m) | `MOVE_BACKWARD 3` |
| `MOVE_LEFT` | 向左移動 | 距離 (m) | `MOVE_LEFT 3` |
| `MOVE_RIGHT` | 向右移動 | 距離 (m) | `MOVE_RIGHT 3` |
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
| `FORMATION_CANCEL` | 無參數 | 取消編隊，停止發布 NED 目標 |

**使用範例**：

```bash
# 環形編隊（半徑 20m，高度 30m）
docker compose exec fmu1_agent bash -c \
  "ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String \
   'data: FORMATION_CIRCLE 20 30'"
```

> 編隊命令必須透過 **`/fmu_all_agent/command`** 廣播，
> 所有機體的 `ego_swarm_planner` 同時收到並各自計算自身的虛擬編隊槽位與未來軌跡。


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
`BasicController` 取樣後發布 PX4 `TrajectorySetpoint`。舊的
`/fmu{N}_agent/ned_target` 僅保留相容或純邏輯測試用途：

EGO-Swarm Planner 另訂閱下列本機任務意圖 Topic：

```text
/{agent_namespace}/planning/mission_intent
std_msgs/msg/String
QoS: reliable, depth 10
```

`/{agent_namespace}/command` 的 `MOVE_*` 與 `MOVE_POINT` 位置欄位會由 EGO-Swarm
Planner 接收並轉換成目標；`MODE_*`、`ARM`、`TAKEOFF`、`LAND`、`YAW` 仍只由
BasicController 執行。`MOVE_POINT` 的 yaw 欄位也由 BasicController 直接設定為
PX4 絕對 yaw，避免 Planner 忽略航向欄位。

到達目標後，Planner 會以目標座標填滿固定 hold B-spline，並發布
`No mission target; BasicController keeps its hold setpoint`。軌跡逾時後，
BasicController 仍鎖定最後控制點，不會改用漂移中的即時 odometry 作為新目標。

```bash
# 手動發布 NED 目標（Debug）
ros2 topic pub --once /fmu1_agent/ned_target taiphoon_agent/msg/NedTarget \
  '{x: 10.0, y: 5.0, z: -15.0, yaw: 90.0}'
```

> `yaw` 欄位若設為 `NaN`（預設值），表示不控制偏航，由飛控自行維持目前朝向。

### AgentTrajectory 的 `control_points`

`AgentTrajectory.control_points` 是 `float32[]` 扁平陣列，使用 `swarm_ned` 座標。
每 3 個連續元素代表一個控制點 `[x, y, z]`，排列格式為：

```text
[x0, y0, z0, x1, y1, z1, ..., xN, yN, zN]
```

陣列長度必須是 3 的倍數；不可依賴巢狀 `float32[3][]` 結構。

---

## 參數設定

### BasicController 參數

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `target_altitude_m` | double | 3.0 | 預設起飛高度（公尺） |
| `hover_duration_s` | double | 3.0 | 預設懸停時間（秒） |
| `startup_delay_s` | double | 0.0 | 啟動後延遲自動 Offboard 的秒數（多機依 `(i-1) * 2.0` 錯開） |
| `auto_start_sequence` | bool | true | 連線後是否自動切換至 Offboard 模式 |
| `target_system` | int | 1 | PX4 MAVLink 系統 ID |
| `agent_namespace` | string | fmu1 | Agent 命令命名空間 |
| `state_publish_rate_hz` | double | 10.0 | AgentState／AgentAttitude 名目發布頻率（Hz）；非法值或低於 10 時固定使用 10 Hz |
| `collision_safety_margin_m` | double | 3.0 | 舊觀測式防撞參數；EGO-Swarm 取代後不作為新的規劃契約 |
| `collision_peer_timeout_s` | double | 2.0 | 舊觀測式防撞參數；EGO-Swarm 取代後不作為新的規劃契約 |

### EGO-Swarm Planner 實際參數

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `agent_namespace` | string | `fmu1_agent` | Agent 對外命名空間 |
| `agent_index` | int | `1` | Agent 索引；小於等於 0 時由 Node 推算 |
| `swarm_origin_agent_index` | int | `1` | 固定 `swarm_ned` 原點的 Agent 索引；尚未取得該 Agent 有效 GPS 前保持 HOLD，原點建立後不再切換 |
| `planning_rate_hz` | double | `10.0` | 規劃迴圈頻率 |
| `formation_min_distance_m` | double | `3.0` | 編隊槽位的機體中心到中心最小距離；可調為 1.5、2 或 3 m |
| `peer_collision_min_distance_m` | double | `3.0` | 同儕未來軌跡的機體中心到中心最小距離；不因資料年齡擴張 |
| `obstacle_clearance_m` | double | `1.0` | EGO-Swarm 與局部障礙物的安全間距 |
| `knot_interval_s` | double | `1.0` | B-spline 最小基準 knot 間隔；實際 duration 由 EGO-Swarm 依候選路徑長度與限制計算 |
| `max_velocity_m_s` | double | `2.0` | EGO-Swarm 計算軌跡 duration 使用的最大速度（公尺/秒） |
| `max_acceleration_m_s2` | double | `1.0` | EGO-Swarm 計算軌跡 duration 使用的最大加速度（公尺/秒^2） |
| `candidate_offset_m` | double | `3.0` | 局部候選軌跡側向／垂直偏移 |
| `local_arrival_tolerance_m` | double | `0.5` | PX4 local NED 到達容許誤差（公尺） |
| `global_arrival_tolerance_m` | double | `1.0` | GPS 轉換共同 NED 到達容許誤差（公尺） |

目前核心是 deterministic candidate baseline，並非完整原始 EGO-Swarm nonlinear optimizer。
EGO-Swarm 會依候選路徑長度、最大速度與最大加速度計算每條 B-spline 的 duration。
同一條 active trajectory 在執行期間保留原始 `start_time` 與 `trajectory_id`，不會每個
planning cycle 重置進度。Planner 同時檢查 PX4 local NED 與 GPS common NED；兩者都在
容許誤差內才視為到達。到達後發布一次目標位置 hold trajectory，清除任務並停止產生
新的移動軌跡；沒有移動或編隊目標時，BasicController 維持 finite hold setpoint。

### ObstacleCloudSimulator 實際參數

| 參數 | 類型 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | string | `fmu1_agent` | Agent 對外命名空間 |
| `config_file` | string | 空字串 | YAML 場景檔；Launch 預設為 `config/obstacles/default.yaml` |
| `publish_rate_hz` | double | `15.0` | PointCloud2 發布頻率 |
| `mode` | string | `sensor_emulation` | `ground_truth_map` 或 `sensor_emulation` |
| `range_m` | double | `20.0` | 感測範圍 |
| `horizontal_fov_deg` | double | `270.0` | 水平視野 |
| `vertical_fov_deg` | double | `90.0` | 垂直視野 |
| `noise_std_m` | double | `0.02` | 感測位置雜訊標準差 |
| `dropout_ratio` | double | `0.05` | 點雲 dropout 比例 |
| `sample_spacing_m` | double | `0.5` | 障礙物表面採樣間距 |

範例：停用自動 Offboard 並設定起飛高度 5m：

```bash
docker compose exec fmu1_agent bash -c "
  source /opt/ros/humble/setup.bash
  source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
  source /home/taiphoon/Taiphoon_Agent/install/setup.bash
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=1 \
      auto_start_sequence:=false target_altitude_m:=5.0
"
```

---

## Docker 架構

```
macOS 宿主
  ├── taiphoon_agent_dev (host 網路)     ← 開發/編譯/ROS2 指令
  │     ├── BasicController[N]            ← 唯一 PX4 TrajectorySetpoint 輸出端
  │     ├── EgoSwarmPlanner[N]            ← 編隊、障礙物與同儕軌跡規劃
  │     └── ObstacleCloudSimulator        ← YAML PointCloud2 模擬（需要時啟動）
  │
  └── Taiphoon_Net (bridge 172.18.x.x)
        ├── drone1-uxrce-dds-agent (uXRCE-DDS Agent)  ← PX4 ↔ ROS2 橋接
        ├── drone1       (PX4 SITL)         ← 飛控模擬
        ├── drone2-uxrce-dds-agent
        ├── drone2
        └── ...
```

| 容器 | 映像 | 網路 | 說明 |
|------|------|------|------|
| `taiphoon_agent_dev` | `taiphoon-agent:dev` | host | ROS2 + 開發環境 |
| `drone{N}` | `px4io/px4-sitl:latest` | Taiphoon_Net | PX4 SITL（SIH 模擬） |
| `drone{N}-uxrce-dds-agent` | `taiphoon/uxrce-agent:latest` | Taiphoon_Net | uXRCE-DDS 通訊橋接 |
| `uxrce_agent_hw` | `taiphoon/uxrce-agent:latest` | host | 實機 uXRCE-DDS 常駐（`--profile hw`） |

---

## 系統架構

以下為目前 EGO-Swarm 實際模組邊界與執行檔：

```
src/
  ├── core/           - 抽象介面定義（IController, IBasicFlightController）
  ├── controllers/    - 飛行控制器（BasicController）、命令處理（AgentCommandHandler）
  ├── ego_swarm/      - SwarmFrame、FormationSlotPlanner、B-spline 與局部規劃核心
  ├── perception/     - 障礙物幾何採樣與 PointCloud2 Adapter
  ├── nodes/          - ROS2 節點封裝
  │     ├── basic_controller_node.hpp/cpp   - BasicControllerNode（生命週期管理）
  │     └── ...                              - 既有 Agent 節點封裝
  ├── utils/          - 共用工具（CommandTypes, math_utils）
  ├── main.cpp        - BasicController 進入點
  ├── main_ego_swarm_planner.cpp       - EGO-Swarm Planner 進入點
  └── main_obstacle_cloud_simulator.cpp - 障礙物模擬器進入點
msg/              - 自訂 ROS2 訊息定義（既有 AgentState/NedTarget 與 EGO-Swarm 新訊息）
launch/           - ROS2 Launch 檔案
AutoTest/         - 自動測試腳本與紀錄
docker/           - Dockerfile 與 entrypoint
```

EGO-Swarm 的 ROS2 資料流為：

```text
GCS command
    ↓
ego_swarm_planner
    ├─ GPS common frame／編隊槽位
    ├─ PointCloud2 障礙物
    └─ 同儕 AgentTrajectory
    ↓
BasicController
    ↓
PX4 TrajectorySetpoint
```

狀態機流程：

```
IDLE -> OFFBOARD_INIT -> OFFBOARD_ACTIVE -> ARMED -> TAKEOFF -> HOVER
                                                         \-> MOVING
                                                         \-> RTL -> POST_RTL
                                                         \-> LAND
```


## 建置與開發

### 在 Docker 容器內建置

**單行編譯（推薦）**：

```bash
docker compose exec fmu1_agent bash -c \
  "cd /home/taiphoon/Taiphoon_Agent && \
   colcon build --symlink-install --packages-select taiphoon_agent"
```

> `--symlink-install` 會以符號連結安裝，修改 Python / 設定檔後無需重新編譯。

**互動式編譯**（進入容器逐步執行）：

```bash
docker compose exec fmu1_agent bash

source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

cd /home/taiphoon/Taiphoon_Agent
colcon build --packages-select taiphoon_agent

source install/setup.bash
```

### 編譯後驗證


```bash
# 確認執行檔存在
ls -la install/lib/taiphoon_agent/basic_controller
ls -la install/lib/taiphoon_agent/ego_swarm_planner
ls -la install/lib/taiphoon_agent/obstacle_cloud_simulator

# 確認 Launch 檔案存在
ls -la install/share/taiphoon_agent/launch/basic_controller.launch.py
```

## 故障排除

| 現象 | 可能原因 | 解決方式 |
|------|----------|----------|
| Launch 時找不到 `taiphoon_agent` | 未 source workspace | 執行 `source install/setup.bash` |
| Launch 時找不到自訂訊息 | 未 source px4_msgs workspace | 執行 `source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash` |
| `ros2 topic echo` 無法解析 NedTarget | 未 source 自訂訊息 workspace | 同上，需先 source install/setup.bash |
| 無人機無法切換 Offboard | 心跳未建立 | 檢查 `auto_start_sequence` 參數是否為 true |
| 無人機不起飛 | 高度或位置命令異常 | 檢查 `/fmu{N}_agent/planning/status` 與 `/fmu{N}_agent/planning/trajectory` |
| 編隊不啟動 | `ego_swarm_planner` 未啟動或 GPS common frame 尚未建立 | 確認 Node 是否存在、GPS 是否有效 |
| 模擬器無法啟動 | 網路或 Docker 權限問題 | 確認 `Taiphoon_Net` 已建立且 Docker 有足夠資源 |

### 日誌檢視

```bash
# 檢視所有 ROS2 topic
ros2 topic list

# 檢視特定 namespace 的里程計
ros2 topic echo /fmu1/fmu/out/vehicle_odometry

# 檢視 Agent 狀態（GPS / 電池 / 飛行模式）
ros2 topic echo /fmu1_agent/state

# 量測 AgentState 發布頻率（名目頻率至少 10 Hz）
ros2 topic hz /fmu1_agent/state

# 檢視 position_global 的 PX4 原始來源
ros2 topic echo /fmu1/fmu/out/vehicle_global_position

# 檢視 EGO-Swarm 規劃結果
ros2 topic echo /fmu1_agent/planning/trajectory taiphoon_agent/msg/AgentTrajectory

# 檢視 PointCloud2 障礙物
ros2 topic echo /fmu1_agent/perception/obstacles

# 舊 NED 目標（僅相容／純邏輯測試）
ros2 topic echo /fmu1_agent/ned_target
```
