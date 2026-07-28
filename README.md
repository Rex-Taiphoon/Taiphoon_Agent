# Taiphoon_Agent

**無人機任務與感測器執行中樞**，基於 ROS2 (Humble) + PX4 的自主飛行控制系統，支援多機編隊、邊緣運算與即時影像處理。

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

## 系統需求

- Docker + Docker Compose
- macOS / Linux（Windows 需 WSL2）
- 至少 8GB 可用記憶體

---

## 操作手冊

所有 ROS2 指令在 Taiphoon_Agent 容器內執行，格式為：

```bash
docker compose exec taiphoon-agt bash -c "source /opt/ros/humble/setup.bash && <指令>"
```

### 1. 模擬器管理

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

每台無人機對應兩個容器：`drone{N}`（PX4 SITL）與 `drone{N}-agent`（uXRCE-DDS Agent）。命名空間 `fmu{N}`、MAV_SYS_ID 與通訊埠由 `--count` 自動遞增。

### 2. 啟動 Taiphoon_Agent

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  source /home/taiphoon/Taiphoon_Agent/install/setup.bash
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=1
"
```

啟動後 Agent 會自動切換至 Offboard 模式（可由 `auto_start_sequence` 參數控制開關）。

### 3. 設定飛行模式

```bash
# 切換至 Offboard 模式
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'
"

# 切換至 Hold 模式（定點懸停）
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: HOLD'
"

# 切換至 Land 模式（立即降落）
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_LAND'
"
```

全體廣播（所有 Agent 同時執行）：

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu_all_agent/command std_msgs/msg/String 'data: MODE_OFFBOARD'
"
```

### 4. 解鎖指令

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: ARM'
"
```

> 注意：需先在 Offboard 模式下發送 OffboardControlMode 心跳，PX4 才允許解鎖。

### 5. 上鎖指令

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: DISARM'
"
```

### 6. 移動指令

```bash
# 向前移動 5 公尺
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MOVE_FORWARD 5'
"

# 向後 / 左 / 右移動
# data: MOVE_BACKWARD 3
# data: MOVE_LEFT 3
# data: MOVE_RIGHT 3
```

> 移動指令基於 NED 座標系（x=前、y=右、z=下），從當前位置出發。

### 7. 返航指令

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /fmu1_agent/command std_msgs/msg/String 'data: MODE_RTL'
"
```

> RTL（Return to Launch）會讓無人機自動返回起飛點並降落。

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
| `TAKEOFF` | 起飛至指定高度 | 高度 (m) | `TAKEOFF 10` |
| `MOVE_FORWARD` | 向前移動 | 距離 (m) | `MOVE_FORWARD 5` |
| `MOVE_BACKWARD` | 向後移動 | 距離 (m) | `MOVE_BACKWARD 3` |
| `MOVE_LEFT` | 向左移動 | 距離 (m) | `MOVE_LEFT 3` |
| `MOVE_RIGHT` | 向右移動 | 距離 (m) | `MOVE_RIGHT 3` |

---

## 參數設定

Launch 時可調整參數來改變 Agent 行為：

| 參數 | 類型 | 預設值 | 說明 |
|------|------|--------|------|
| `target_altitude_m` | double | 3.0 | 預設起飛高度（公尺） |
| `hover_duration_s` | double | 3.0 | 預設懸停時間（秒） |
| `startup_delay_s` | double | 0.0 | 啟動後延遲自動 Offboard 的秒數 |
| `auto_start_sequence` | bool | true | 連線後是否自動切換至 Offboard 模式 |
| `target_system` | int | 1 | PX4 MAVLink 系統 ID |
| `agent_namespace` | string | fmu1 | Agent 命令命名空間 |

範例：停用自動 Offboard 並設定起飛高度 5m：

```bash
docker compose exec taiphoon-agt bash -c "
  source /opt/ros/humble/setup.bash
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
  │
  └── Taiphoon_Net (bridge 172.18.x.x)
        ├── drone1-agent (uXRCE-DDS Agent)  ← PX4 ↔ ROS2 橋接
        ├── drone1       (PX4 SITL)         ← 飛控模擬
        ├── drone2-agent
        ├── drone2
        └── ...
```

| 容器 | 映像 | 網路 | 說明 |
|------|------|------|------|
| `taiphoon_agent_dev` | `taiphoon-agent:dev` | host | ROS2 + 開發環境 |
| `drone{N}` | `px4io/px4-sitl:latest` | Taiphoon_Net | PX4 SITL（SIH 模擬） |
| `drone{N}-agent` | `taiphoon/uxrce-agent:latest` | Taiphoon_Net | uXRCE-DDS 通訊橋接 |

---

## 系統架構

```
src/core/         - 抽象介面定義（IController, IBasicFlightController）
src/controllers/  - 飛行控制器（BasicController）、命令處理（AgentCommandHandler）
src/nodes/        - ROS2 節點封裝（BasicControllerNode）
src/utils/        - 共用工具（CommandTypes, math_utils）
config/           - YAML 設定檔
launch/           - ROS2 Launch 檔案
AutoTest/         - 自動測試腳本與紀錄
docker/           - Dockerfile 與 entrypoint
skills/           - Codex Agent 技能定義
```

狀態機流程：

```
IDLE -> OFFBOARD_INIT -> OFFBOARD_ACTIVE -> ARMED -> TAKEOFF -> HOVER
                                                         \-> MOVING
                                                         \-> RTL -> LAND
```
