# Taiphoon_Agent

**無人機任務與感測器執行中樞**，基於 ROS2 (Humble) + PX4 的自主飛行控制系統，支援多機編隊（環形 / 水平直列 / 垂直直列）、邊緣運算與即時影像處理。

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
docker compose up -d taiphoon-agt

# 5. 啟動模擬器
bash skills/px4-sitl-launcher/scripts/launch_all.sh --count 1 --net Taiphoon_Net

# 6. 確認 ROS2 通訊
docker compose exec taiphoon-agt bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"
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

---

## 操作手冊 — 基本控制

所有 ROS2 指令在 Taiphoon_Agent 容器內執行。建議先進入容器再執行指令：

```bash
docker compose exec taiphoon-agt bash
```

進入後需先載入環境（每次開啟新終端都需要）：

```bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
```

### 1. 模擬器管理

在**宿主機**（非容器內）執行：

```bash
# 啟動單機模擬
bash skills/px4-sitl-launcher/scripts/launch_all.sh --count 1 --net Taiphoon_Net

# 啟動多機模擬（3 台一字型排列）
bash skills/px4-sitl-launcher/scripts/launch_all.sh --count 3 --net Taiphoon_Net

# 使用 CycloneDDS 啟動
bash skills/px4-sitl-launcher/scripts/launch_all.sh --count 3 --net Taiphoon_Net --rmw cyclonedds

# 關閉所有模擬器
bash skills/px4-sitl-launcher/scripts/stop_all.sh
```

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--count N` | 無人機數量 | 3 |
| `--net NAME` | Docker 網路名稱 | Taiphoon_Net |
| `--rmw TYPE` | DDS 中介軟體（fastrtps / cyclonedds） | fastrtps |

每台無人機對應兩個容器：`drone{N}`（PX4 SITL）與 `drone{N}-uxrce-dds-agent`（uXRCE-DDS Agent）。命名空間 `fmu{N}`、MAV_SYS_ID 與通訊埠由 `--count` 自動遞增。

### 2. 啟動 Taiphoon_Agent（容器內）

先進入容器再載入環境，然後 Launch：

```bash
docker compose exec taiphoon-agt bash

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
| `drone_count` | int | 3 | 無人機數量；>1 時自動啟動每台對應的 **FormationNode** |

當 `drone_count > 1` 時，Launch 檔案會為每台無人機建立兩個節點：
- **BasicController** — 單機飛行控制（狀態機：IDLE → OFFBOARD → ARMED → TAKEOFF → HOVER → ...）
- **FormationNode** — 編隊計算節點（5Hz 更新，接收 FORMATION_* 命令）

### 3. 設定飛行模式

```bash
# 切換至 Offboard 模式
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'

# 切換至 Hold 模式（定點懸停）
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: HOLD'

# 切換至 Land 模式（立即降落）
ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_LAND'
```

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

## 操作手冊 — 編隊飛行

編隊功能由 **FormationNode** 實作，每台無人機獨立執行一個 FormationNode 實例，
透過 `/fmu_all_agent/command` 廣播頻道接收編隊命令，各自計算自身目標位置。

### 啟動多機編隊環境

開啟三個終端視窗分別執行：

```bash
# 終端 1：啟動 4 台模擬器
bash skills/px4-sitl-launcher/scripts/launch_all.sh --count 4 --net Taiphoon_Net
```

```bash
# 終端 2：啟動 Taiphoon_Agent（含 4 組 BasicController + FormationNode）
docker compose exec taiphoon-agt bash
# 在容器內：
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4
```

```bash
# 終端 3：進入容器，手動下達指令
docker compose exec taiphoon-agt bash
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
| **環形** | 所有無人機等角分布在圓周上，質心為所有線上機體的中心 | ✅ 自動調整角度與半徑 |
| **水平直列** | 沿 Y 軸（右向）等間距排列 | ✅ 自動收縮間距 |
| **垂直直列** | 沿 Z 軸（向下）等間距排列 | ✅ 自動收縮間距 |

> 編隊運算使用 **質心（centroid）** 作為基準點。當某台無人機離線（超過 2 秒未回傳里程計），
> 所有線上機體會自動重新計算編隊位置，無需手動干預。

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
| `YAW` | 順時針旋轉 | 角度 (°) | `YAW 90` |
| `MOVE_POINT` | 移動至指定 NED 絕對位置 | x y z yaw | `MOVE_POINT 10 5 -15 90` |

### 編隊命令

| 命令 | 說明 | 參數 | 範例 |
|------|------|------|------|
| `FORMATION_CIRCLE` | 環形編隊 | 半徑 (m) 高度 (m) | `FORMATION_CIRCLE 5.0 10.0` |
| `FORMATION_HORI_LINEAR` | 水平直列編隊 | 間距 (m) 高度 (m) | `FORMATION_HORI_LINEAR 3.0 5.0` |
| `FORMATION_VERT_LINEAR` | 垂直直列編隊 | 間距 (m) | `FORMATION_VERT_LINEAR 3.0` |
| `FORMATION_CANCEL` | 取消當前編隊 | 無 | `FORMATION_CANCEL` |

> 編隊命令必須透過 **`/fmu_all_agent/command`** 廣播，
> 所有機體的 FormationNode 同時收到並各自計算自身在編隊中的目標位置。

### 簡寫命令

部分移動指令支援簡寫形式：

| 完整形式 | 簡寫形式 |
|----------|----------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

### NED 目標 topic

FormationNode 以 5Hz 頻率發布結構化目標至 `/fmu{N}_agent/ned_target`：

```bash
# 手動發布 NED 目標（Debug）
ros2 topic pub --once /fmu1_agent/ned_target taiphoon_agent/msg/NedTarget \
  '{x: 10.0, y: 5.0, z: -15.0, yaw: 90.0}'
```

> `yaw` 欄位若設為 `NaN`（預設值），表示不控制偏航，由飛控自行維持目前朝向。

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

### FormationNode 參數

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `drone_count` | double | 3.0 | 總無人機數量（影響編隊角度分配） |
| `agent_namespace` | string | fmu1_agent | 自身命令命名空間（用於推算索引） |
| `formation_update_rate_hz` | double | 5.0 | 編隊控制迴圈更新頻率（Hz） |

範例：停用自動 Offboard 並設定起飛高度 5m：

```bash
docker compose exec taiphoon-agt bash -c "
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
  │     ├── BasicController[N]            ← 單機飛行控制（N = 無人機數量）
  │     └── FormationNode[N]              ← 編隊計算（僅 drone_count>1 時啟動）
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

```
src/
  ├── core/           - 抽象介面定義（IController, IBasicFlightController）
  ├── controllers/    - 飛行控制器（BasicController）、命令處理（AgentCommandHandler）
  ├── nodes/          - ROS2 節點封裝
  │     ├── basic_controller_node.hpp/cpp   - BasicControllerNode（生命週期管理）
  │     └── formation_node.hpp/cpp          - FormationNode（編隊計算）
  ├── utils/          - 共用工具（CommandTypes, math_utils）
  ├── main.cpp        - BasicController 進入點
  └── main_formation.cpp - FormationNode 進入點
msg/              - 自訂 ROS2 訊息定義（NedTarget.msg）
config/           - YAML 設定檔（basic_controller.yaml）
launch/           - ROS2 Launch 檔案
AutoTest/         - 自動測試腳本與紀錄
docker/           - Dockerfile 與 entrypoint
skills/           - Codex Agent 技能定義
```

狀態機流程：

```
IDLE -> OFFBOARD_INIT -> OFFBOARD_ACTIVE -> ARMED -> TAKEOFF -> HOVER
                                                         \-> MOVING
                                                         \-> RTL -> POST_RTL
                                                         \-> LAND
```

## 自訂 ROS2 訊息

```msg
# NedTarget.msg
float32 x     # NED X（前向為正，公尺）
float32 y     # NED Y（右向為正，公尺）
float32 z     # NED Z（向下為正，高度取負值，公尺）
float32 yaw   # 目標航向角（度，NaN = 不控制偏航）
```

FormationNode 以 5Hz 發布此訊息至 `/fmu{N}_agent/ned_target`，
BasicController 訂閱後據此執行位置控制。

---

## 建置與開發

### 在 Docker 容器內建置

```bash
docker compose exec taiphoon-agt bash

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
ls -la install/lib/taiphoon_agent/formation_node

# 確認 Launch 檔案存在
ls -la install/share/taiphoon_agent/launch/basic_controller.launch.py
```

---

## 故障排除

| 現象 | 可能原因 | 解決方式 |
|------|----------|----------|
| Launch 時找不到 `taiphoon_agent` | 未 source workspace | 執行 `source install/setup.bash` |
| Launch 時找不到自訂訊息 | 未 source px4_msgs workspace | 執行 `source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash` |
| `ros2 topic echo` 無法解析 NedTarget | 未 source 自訂訊息 workspace | 同上，需先 source install/setup.bash |
| 無人機無法切換 Offboard | 心跳未建立 | 檢查 `auto_start_sequence` 參數是否為 true |
| 無人機不起飛 | 高度或位置命令異常 | 檢查 `/fmu{N}_agent/ned_target` topic 內容 |
| 編隊不啟動 | FormationNode 未啟動 | 確認 `drone_count` > 1 |
| 模擬器無法啟動 | 網路或 Docker 權限問題 | 確認 `Taiphoon_Net` 已建立且 Docker 有足夠資源 |

### 日誌檢視

```bash
# 檢視所有 ROS2 topic
ros2 topic list

# 檢視特定 namespace 的里程計
ros2 topic echo /fmu1/fmu/out/vehicle_odometry

# 檢視 NED 目標（formation_node 輸出）
ros2 topic echo /fmu1_agent/ned_target
```
