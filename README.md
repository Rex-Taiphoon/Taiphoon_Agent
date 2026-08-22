# Taiphoon_Agent

**無人機任務與感測器執行中樞**，基於 ROS2 (Humble) + PX4 的自主飛行控制系統，
以 EGO-Swarm 實作多機編隊、局部障礙物避讓與同儕未來軌跡避碰，並支援邊緣運算與即時影像處理。

> 本 README 已反映目前 EGO-Swarm runtime。移動命令由 Planner 單一路徑處理，
> 單機 `MOVE_FORWARD/BACKWARD/LEFT/RIGHT/UP/DOWN` 以保存的目標位置累加；水平位移使用保存的
> target yaw。全體命令在啟用編隊時以共同 `swarm_ned` 平移 Formation center，保留 slot
> assignment；runtime planner 使用 optimized continuous backend，編隊使用 WGS84 FormationEpoch
> 與 deterministic minimum-cost assignment。水平 `FIXED_Z` MOVE 會先以 Local Goal
> 分段；直線 seed 受阻時使用 bounded 時空 A* 產生側繞 seed，再交給既有 optimizer
> 與 safety validator，整條軌跡維持原高度。

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

EGO 路徑規劃 SITL 驗證（需要已準備 `taiphoon-agent:dev` 與
`px4io/px4-sitl:latest`，腳本會自行清理 Agent 與 PX4）：

```bash
# 單機：BasicController 回歸、直線、固定障礙繞行、封閉範圍 HOLD／Brake
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario single

# 雙機：moving peer 交叉路徑與 3 m 中心距離
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario dual

# 四機：FORMATION_CIRCLE 10 10 固定高度與既有 Formation 回歸
bash AutoTest/scripts/ego_route_planning_sitl_validation.sh --scenario swarm
```

單機與雙機路徑測試會先對所有測試中的 Agent 發送既有 `TAKEOFF 8.0`；四機 Formation
測試則發送既有 `TAKEOFF 10.0`，再使用 `FORMATION_CIRCLE 10 10`，讓編隊高度與起飛
高度一致。
先啟動高度監測器取得地面基準，再發送對應的既有 TAKEOFF；單機／雙機至少達 `7.5 m`，
四機 Formation 至少達 `9.5 m`，並穩定 1 秒後才發布 MOVE 或 Formation（容許 0.5 m
SITL 誤差）。不以固定 sleep 代替起飛完成判斷。

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--count N` | 無人機數量 | 4 |
| `--net NAME` | Docker 網路名稱 | Taiphoon_Net |
| `--rmw TYPE` | DDS 中介軟體（fastrtps / cyclonedds） | fastrtps |
| `--initial-spacing-m M` | PX4 起始水平間距；必須大於預設 3m peer safety 門檻 | 5.0 |

SITL launcher 預設以 8m 水平間距啟動，為飛控定位與控制誤差保留安全
裕度；這不會改寫 planner 的 `peer_collision_min_distance_m`。

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
- **ego_swarm_planner** — WGS84 軌跡交換、本機 local NED、編隊槽位、PointCloud2 與同儕未來軌跡的 optimized continuous 局部規劃節點。
- **obstacle_cloud_simulator** — 依 YAML 產生 PointCloud2 障礙物資料的選配模擬節點。

兩個 Node 已納入 `basic_controller.launch.py`；EGO planner 固定啟動，障礙物模擬器由
`enable_obstacle_cloud_simulator` 控制。

#### Compose 中個別控制障礙物模擬器

`docker-compose.yml` 已為每台 Agent 提供獨立的自動啟動開關，預設值都是
`false`。這是實機安全預設：沒有實際感測器時，不應自動用空障礙物地圖取代缺少的感測資料。

SITL 若要讓四台 Agent 都自動啟動 `obstacle_cloud_simulator`，在宿主機執行：

```bash
FMU1_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
FMU2_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
FMU3_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
FMU4_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
docker compose up -d
```

也可以只開啟指定 Agent，例如只啟動 fmu1 與 fmu2：

```bash
FMU1_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
FMU2_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
docker compose up -d
```

各 Agent 的預設 `OBSTACLE_CLOUD_MODE` 為 `explicit_empty`，會持續發布有效的零點
PointCloud2，讓 `planning/status.map_valid` 成為 `true`。需要實際 YAML 障礙物或感測器
模擬時，可個別覆寫，例如：

```bash
FMU1_ENABLE_OBSTACLE_CLOUD_SIMULATOR=true \
FMU1_OBSTACLE_CLOUD_MODE=synthetic_sensor \
docker compose up -d fmu1_agent
```

這些 `FMU{N}_...` 變數也可以放在專案根目錄的 `.env`，之後直接執行
`docker compose up -d`。修改開關後需要重新執行 `docker compose up -d`，讓 Compose
重新建立受影響的 Agent 容器；啟動後可用下列指令確認：

```bash
docker compose exec fmu1_agent bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
   source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
   ros2 node list | grep obstacle_cloud_simulator"
```

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

編隊功能由 **ego_swarm_planner** 實作。每台無人機以 WGS84 交換位置，再在自己的
固定 Home 建立 receiver-local NED，使用共同 FormationEpoch 計算自身的虛擬編隊槽位，
再由 EGO-Swarm 局部規劃器產生包含障礙物與同儕避碰的未來軌跡。
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

> 編隊以 WGS84 全球位置交換 snapshot；每台 Agent 收到相同 FormationEpoch 後，
> 在自己的固定 Home 轉成 receiver-local NED，使用 Hungarian minimum-cost assignment
> 獨立得到相同的一對一槽位結果。Epoch 需等所有成員發布相同 hash 的 ADOPTED 才啟用；
> discovery 每 3 秒掃描，成員集合連續兩次穩定後才觸發自動重編隊，中心沿用 parent center。

編隊轉換時，Planner 只允許目前位置到槽位的直線或 North/East 水平側向候選；
側向候選的 Z 只沿起點到目標線性插值，不會從上方或下方繞過同儕。沒有衝突的
Agent 可以同時規劃與移動；只有路線互相衝突的 Agent 才會局部等待或重新規劃。
衝突時先以較早的 `start_time` 路線優先，同時間則由較小的 Agent 編號優先；沒有
安全水平路徑時才 HOLD／煞停，不會用垂直繞行突破安全距離。任一 Agent 連續兩次
discovery scan 消失時，其餘成員會建立下一代 FormationEpoch、移除失效成員並重新
計算 assignment；Agent 重新上線後也會以相同機制重新加入。

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
    ├── WGS84 → receiver-local NED／虛擬編隊槽位
    ├── PointCloud2 局部障礙物
    ├── 同儕 AgentTrajectory
    ├── ContinuousSafetyValidator
    │       ├── SAFE → 本機 AgentTrajectory
    │       └── UNSAFE / stale / planner failure → EmergencyBrakePlanner
    └── 本機 AgentTrajectory
              │
              ▼
       BasicController
              │
              ▼
       PX4 TrajectorySetpoint
```

- 同儕避碰距離是機體中心到機體中心的三維直線距離，不加計機體半徑。
- SITL launcher 的初始水平間距預設為 8m；這只是測試起點的安全裕度，
  不會改寫 planner 的 `peer_collision_min_distance_m`。
- 最小距離不可硬編碼為 3 m；Launch 與 entrypoint 以
  `formation_min_distance_m`、`peer_collision_min_distance_m` 提供設定，可使用
  1.5 m、2 m 或 3 m 等值。
- 只有 freshness contract 內的 ACTIVE 或 TERMINAL_HOLD 同儕軌跡會進入驗證；同儕資料 stale、invalid 或
  過期時，規劃器 fail-closed 產生 dynamic emergency brake，不把最後 endpoint 當成 ghost UAV。
- `BasicController` 是唯一發布 PX4 `TrajectorySetpoint` 的節點。

### 動態安全與煞車

每個 planner cycle 會以自身 AgentState 的連續 NED 位置樣本建立
`VehiclePlanningState`（position、velocity、acceleration、timestamp）。Candidate 必須依序
通過 `OptimizedEgoPlanner`、reservation（若需要）與獨立的 `ContinuousSafetyValidator`，
才會更新 active trajectory 並發布。驗證失敗、planner failure 或關鍵資料過期時，
`EmergencyBrakePlanner` 依目前速度／加速度與 jerk limit 產生減速 B-spline；速度尚未接近零
以前不使用 repeated-current-position static hold。

### PointCloud2 障礙物模擬

`obstacle_cloud_simulator` 讀取 YAML 幾何場景，發布：

```text
/{agent_namespace}/perception/obstacles
sensor_msgs/msg/PointCloud2
```

支援 `box`、`cylinder`、`wall`，以及 `explicit_empty`、`ground_truth`、
`synthetic_sensor` 三種障礙物來源（亦相容 `ground_truth_map`、
`sensor_emulation` 舊名稱）。Synthetic sensor 會從 AgentState／AgentAttitude
更新 UAV 在 `swarm_ned` 的目前 pose；尚未取得 pose 時不發布固定原點的假資料。
可直接從 [`config/obstacles/default.yaml`](config/obstacles/default.yaml) 複製場景再
修改；Launch 可使用 `enable_obstacle_cloud_simulator:=true` 啟動模擬器：

```yaml
frame_id: swarm_ned
# 無障礙基線；要測試 YAML 幾何時由 launch/ROS2 mode 改為 ground_truth 或 synthetic_sensor
mode: explicit_empty
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
| `FORMATION_CANCEL` | 無參數 | 取消編隊，停止發布 NED 目標 |

全體 `/fmu_all_agent/command` 的 `MOVE_FORWARD/BACKWARD/LEFT/RIGHT/UP/DOWN` 只在編隊
啟用時有效，方向固定為共同 `swarm_ned` 的 `+North/-North/+East/-East/-Down/+Down`；
全體 `MOVE_POINT` 的 x/y/z 是共同 Formation center 絕對目標。單機 MOVE 語意維持不變。

**使用範例**：

```bash
# 環形編隊（半徑 20m，高度 30m）
docker compose exec fmu1_agent bash -c \
  "ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String \
   'data: FORMATION_CIRCLE 20 30'"
```

> 編隊命令必須透過 **`/fmu_all_agent/command`** 廣播；Planner 會以 Epoch 去重，
> 相同 config 與成員 snapshot 的重複命令不會建立新 epoch；單機 MOVE 或取消編隊後重送
> 相同編隊命令會重新啟用既有 Epoch，保留 center 與 slot assignment。


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
`/fmu{N}_agent/ned_target` 僅保留相容或純邏輯測試用途：

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
ros2 topic pub --once /fmu1_agent/ned_target taiphoon_agent/msg/NedTarget \
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
接收端會以自己的固定 Home 將控制點轉成公尺制 local NED，再執行 B-spline
取樣、動力學限制與同儕避碰。完成任務的 Agent 會持續發布
`terminal_hold=true` 的固定終點軌跡 heartbeat，直到超過 peer timeout，讓其他
Agent 仍把該終點視為佔位障礙。這類待命 heartbeat 只發布到 global
`/fmu_all_agent/swarm/agent_trajectories`，不會送入本機
`/{agent_namespace}/planning/trajectory`；只有本機需要執行的移動、煞車與任務終點 hold
才會交給 BasicController 取樣成 PX4 `TrajectorySetpoint`。

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
| `swarm_origin_agent_index` | int | `1` | 保留的相容參數；WGS84 軌跡交換時各 Agent 以自己的固定 Home 建立 local NED |
| `peer_trajectory_timeout_s` | double | `2.0` | 同儕未來軌跡 freshness timeout；逾時後不再交給 planner 使用 |
| `max_peer_time_skew_s` | double | `1.0` | 同儕軌跡 message／start time 與本機 receive time 允許的最大時間偏差 |
| `obstacle_cloud_timeout_s` | double | `1.0` | 局部障礙物地圖 freshness timeout；逾時後 planner 進入 HOLD |
| `require_obstacle_cloud` | bool | `true` | 是否要求 fresh obstacle map；missing/stale 不會被當成 explicit empty |
| `planning_rate_hz` | double | `10.0` | 規劃迴圈頻率 |
| `enable_local_goal_planning` | bool | `true` | Local Goal segmented planning；水平 FIXED_Z 受阻時啟用 bounded 時空 A*，FREE_3D／LINEAR_Z 維持既有 planner |
| `enable_generic_trajectory_reservation` | bool | `false` | 將 reservation 擴展至 MOVE 等所有 moving trajectory；Formation 仍固定使用 reservation |
| `local_goal_horizon_m` | double | `10.0` | Local Goal 單一 planning segment 的距離 horizon（公尺） |
| `local_goal_replan_ratio` | double | `0.70` | Local Goal segment progress 達此比例後，允許嘗試下一段規劃 |
| `minimum_replan_interval_s` | double | `0.5` | 正常 progress-based replan 的最短間隔（秒）；不阻擋 safety、Final Target 變更與 segment 結束 |
| `formation_min_distance_m` | double | `3.0` | 編隊槽位的機體中心到中心最小距離；可調為 1.5、2 或 3 m |
| `peer_collision_min_distance_m` | double | `3.0` | 同儕未來軌跡的機體中心到中心最小距離；不因資料年齡擴張 |
| `obstacle_clearance_m` | double | `1.0` | EGO-Swarm 與局部障礙物的安全間距 |
| `knot_interval_s` | double | `1.0` | B-spline 最小基準 knot 間隔；實際 duration 由 EGO-Swarm 依候選路徑長度與限制計算 |
| `max_velocity_m_s` | double | `2.5` | EGO-Swarm 計算軌跡 duration 使用的最大速度（公尺/秒） |
| `max_acceleration_m_s2` | double | `1.5` | EGO-Swarm 計算軌跡 duration 使用的最大加速度（公尺/秒^2） |
| `max_velocity_mps` | double | `2.5` | safety validator／煞車使用的最大速度（公尺/秒）；與舊名稱同步時以此名稱為 canonical |
| `max_acceleration_mps2` | double | `1.5` | safety validator／煞車使用的最大加速度（公尺/秒^2） |
| `max_jerk_mps3` | double | `3.0` | safety validator／煞車使用的最大 jerk（公尺/秒^3） |
| `candidate_offset_m` | double | `3.0` | 局部候選軌跡側向／垂直偏移 |
| `local_arrival_tolerance_m` | double | `0.5` | PX4 local NED 到達容許誤差（公尺） |
| `global_arrival_tolerance_m` | double | `1.0` | GPS 轉換共同 NED 到達容許誤差（公尺） |
| `move_attitude_timeout_s` | double | `1.0` | 水平 MOVE 可使用的 AgentAttitude freshness window |
| `formation_slot_switch_penalty_m` | double | `2.0` | minimum-cost assignment 換槽位成本 |
| `formation_member_stable_scan_count` | int | `2` | discovery 成員集合連續穩定掃描次數 |
| `planner_backend` | string | `optimized` | optimized-only runtime；其他值使 planner 初始化失敗 |
| `optimizer_max_iterations` | int | `40` | optimized projected-gradient iteration 上限 |
| `optimizer_time_budget_ms` | double | `50.0` | optimized backend 單次 wall-time 上限（毫秒）；保留 finite-difference 與 SITL callback 負載餘裕 |
| `formation_crossing_penalty_m` | double | `10.0` | assignment 水平 crossing 等價成本 |
| `reservation_lead_time_s` | double | `0.5` | future trajectory reservation 最短提前時間 |
| `reservation_step_s` | double | `0.5` | reservation start time 搜尋步進 |
| `reservation_horizon_s` | double | `5.0` | reservation start time 最遠搜尋範圍 |

目前 runtime 只使用內建 pure C++ continuous control-point projected-gradient backend，
不引入外部 optimizer dependency。`planner_backend` 只接受 `optimized`；deterministic
類別保留作為既有純邏輯相容檔案，但不再由 runtime 組裝或作 fallback。Optimized backend
只使用單一直線 B-spline seed，起終點為硬約束，中間控制點的水平搜尋範圍為內部 10 m；
`FIXED_Z` 與 `LINEAR_Z` 路徑會分別固定高度或只允許起終點之間的線性高度變化。
objective 包含 terminal goal、smoothness、動力學超限、障礙物 clearance barrier 與
peer time-aligned separation barrier，並保留 obstacle 1 m、peer 3 m 的 hard safety
門檻。動力學 gate 預設上限為 velocity 2.5 m/s、acceleration 1.5 m/s²、jerk 3.0 m/s³；
optimizer wall-time budget 為 50 ms。Optimizer 失敗時不使用 deterministic fallback；靜止／低速 Agent 立即建立固定
位置 HOLD 並週期性重試，移動中的 Agent 使用 EmergencyBrakePlanner 後再 HOLD。
EGO-Swarm 會依候選路徑長度、最大速度、最大加速度與最大 jerk 計算每條 B-spline
的 duration；若 analytic p/v/a/j 取樣仍超限，會逐步延長候選時間後才交給
ContinuousSafetyValidator。
同一條 active trajectory 在執行期間保留原始 `start_time` 與 `trajectory_id`，不會每個
planning cycle 重置進度。Planner 以自己的固定 Home 將 WGS84 與 PX4 local NED
轉換到同一個公尺制 local frame；兩者都在容許誤差內才視為到達。到達後持續發布
`terminal_hold=true` 的目標位置 hold trajectory，清除任務但保留 peer occupancy
heartbeat；沒有移動或編隊目標時，BasicController 維持原本鎖定的 finite hold setpoint，
不會被 global occupancy heartbeat 改寫。
同儕 `AgentTrajectory` 另受 `peer_trajectory_timeout_s`、`max_peer_time_skew_s`、
控制點／duration 合理性與 `frame_revision` 約束；ACTIVE 與 TERMINAL_HOLD 資料都
參與 peer collision 評估。ContinuousSafetyValidator 會再檢查整條候選軌跡的 p/v/a/j、
peer/map freshness、同儕 separation 與 obstacle clearance；移動軌跡逾時或資料失效時
切換至 EmergencyBrakePlanner，完成後由 sender 發布終點 terminal hold。
編隊 terminal hold 會清除舊 hold 後再發布單調遞增的 trajectory ID，避免舊 heartbeat
覆蓋新軌跡；EmergencyBrakePlanner 進行中的減速軌跡沿原時間軸發布，不會每個 cycle
重新煞車。

### ObstacleCloudSimulator 實際參數

| 參數 | 類型 | Node 預設值 | 說明 |
|------|------|-------------|------|
| `agent_namespace` | string | `fmu1_agent` | Agent 對外命名空間 |
| `config_file` | string | 空字串 | YAML 場景檔；Launch 預設為 `config/obstacles/default.yaml` |
| `publish_rate_hz` | double | `15.0` | PointCloud2 發布頻率 |
| `agent_index` | int | `1` | Agent 索引；供 synthetic sensor 建立共同 frame |
| `swarm_origin_agent_index` | int | `1` | synthetic sensor 的相容參數；障礙物資料仍以 `swarm_ned` 公尺座標輸入 |
| `mode` | string | `explicit_empty` | ROS2 parameter 覆蓋 YAML `mode`；預設發布 fresh zero-point cloud |
| `range_m` | double | `20.0` | 感測範圍 |
| `horizontal_fov_deg` | double | `270.0` | 水平視野 |
| `vertical_fov_deg` | double | `90.0` | 垂直視野 |
| `noise_std_m` | double | `0.02` | 感測位置雜訊標準差 |
| `dropout_ratio` | double | `0.05` | 點雲 dropout 比例 |
| `sample_spacing_m` | double | `0.5` | 障礙物表面採樣間距 |

Planner 只接受 `IObstacleMap` 的 fresh map。未收到 cloud 是 `missing`，收到但超過
`obstacle_cloud_timeout_s` 是 `stale`；只有 `explicit_empty` source 產生的 zero-point
map 才是 `fresh + valid + empty`。因此啟動 obstacle simulator 時，無障礙場景應明確使用
`obstacle_mode:=explicit_empty`，不能以停發 PointCloud2 代表空環境。

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
    ├─ WGS84 → receiver-local NED／編隊槽位
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
| 編隊不啟動 | `ego_swarm_planner` 未啟動或本機 local NED Home 尚未建立 | 確認 Node 是否存在、GPS 是否有效 |
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
