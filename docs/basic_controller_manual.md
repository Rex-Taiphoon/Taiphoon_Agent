# Taiphoon Agent BasicController 操作手冊

## 系統架構說明

BasicController 與 FormationNode 是 Taiphoon Agent 的兩大核心模組：

- **BasicController**：單機飛行控制，處理 Offboard 模式下的起降、移動、返航、解鎖/上鎖等命令
- **FormationNode**：編隊計算節點（多機時自動啟動），接收 FORMATION_* 命令，計算自身 NED 目標位置

系統採用 SOLID 原則設計，包含以下層次：

- **核心介面層（`core/`）**：IController、IBasicFlightController 定義控制器抽象介面
- **工具層（`utils/`）**：math_utils 提供數學比較與數值合法性檢查；CommandTypes 提供命令解析
- **控制器層（`controllers/`）**：BasicController 實作起降邏輯與 PX4 Offboard 控制；AgentCommandHandler 負責命令路由
- **節點層（`nodes/`）**：BasicControllerNode 封裝 ROS2 通訊；FormationNode 封裝編隊運算

### 狀態機流程

```
IDLE -> OFFBOARD_INIT -> OFFBOARD_ACTIVE -> ARMED -> TAKEOFF -> HOVER
                                                             \-> MOVING
                                                             \-> RTL -> POST_RTL
                                                             \-> LAND
```

## 啟動步驟

### Launch 檔案（多機模式）

```bash
# 先載入環境
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash

# Launch（4 台無人機）
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4
```

當 `drone_count` > 1 時，Launch 會為每台無人機建立一組 **BasicController** + **FormationNode**，
並自動設定命名空間與啟動延遲（多機依索引錯開 2 秒）。

### 單獨啟動

```bash
ros2 run taiphoon_agent basic_controller --ros-args \
    -r __ns:=/fmu1 \
    -p target_altitude_m:=3.0 \
    -p hover_duration_s:=3.0

ros2 run taiphoon_agent formation_node --ros-args \
    -r __ns:=/fmu1 \
    -p drone_count:=4.0 \
    -p agent_namespace:=fmu1_agent
```

## 支援的高階命令

以下命令可透過 `/fmu{N}_agent/command`（單機）或 `/fmu_all_agent/command`（全體廣播）發送。

### 基本飛行命令（BasicController 處理）

| 命令 | 完整格式 | 說明 |
|------|----------|------|
| `MODE_OFFBOARD` | `MODE_OFFBOARD` | 切換至 Offboard 模式 |
| `MODE_RTL` | `MODE_RTL` | 切換至返航模式 |
| `MODE_LAND` | `MODE_LAND` | 切換至降落模式 |
| `ARM` | `ARM` | 解鎖 |
| `DISARM` | `DISARM` | 上鎖 |
| `TAKEOFF` | `TAKEOFF [高度]` | 起飛至指定高度（公尺），無參數時使用預設值 3.0m |
| `HOLD` | `HOLD` | 保持目前位置懸停 |
| `MOVE_FORWARD` | `MOVE_FORWARD [距離]` | 向前移動指定距離（公尺） |
| `MOVE_BACKWARD` | `MOVE_BACKWARD [距離]` | 向後移動指定距離（公尺） |
| `MOVE_LEFT` | `MOVE_LEFT [距離]` | 向左移動指定距離（公尺） |
| `MOVE_RIGHT` | `MOVE_RIGHT [距離]` | 向右移動指定距離（公尺） |
| `MOVE_UP` | `MOVE_UP [距離]` | 向上移動指定距離（公尺），相對位移 |
| `MOVE_DOWN` | `MOVE_DOWN [距離]` | 向下移動指定距離（公尺），最低不低於 1m |
| `YAW` | `YAW [角度]` | 順時針旋轉指定角度（度），相對目前方向 |
| `MOVE_POINT` | `MOVE_POINT x y z yaw` | 移動至指定 NED 絕對位置（x/y/z/yaw） |

### 編隊命令（FormationNode 處理，僅限 `/fmu_all_agent/command`）

| 命令 | 完整格式 | 說明 |
|------|----------|------|
| `FORMATION_CIRCLE` | `FORMATION_CIRCLE 半徑 高度` | 環形編隊，等角分布 |
| `FORMATION_HORI_LINEAR` | `FORMATION_HORI_LINEAR 間距 高度` | 水平直列，沿 Y 軸排列 |
| `FORMATION_VERT_LINEAR` | `FORMATION_VERT_LINEAR 間距` | 垂直直列，沿 Z 軸排列 |
| `FORMATION_CANCEL` | `FORMATION_CANCEL` | 取消當前編隊任務 |

### 簡寫命令

| 完整形式 | 簡寫 |
|----------|------|
| `MOVE_FORWARD` | `FORWARD` |
| `MOVE_BACKWARD` | `BACKWARD` |
| `MOVE_LEFT` | `LEFT` |
| `MOVE_RIGHT` | `RIGHT` |

> 命令不區分大小寫。MOVE_UP / MOVE_DOWN / YAW / MOVE_POINT 需在飛行狀態（HOVER / TAKEOFF / MOVING）下執行。

## 依賴套件

- ROS2 Humble
- px4_msgs
- PX4-Autopilot（SIH 模擬器）

## 建置步驟（容器內）

```bash
# 1. 載入 ROS2 環境
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash   # px4_msgs

# 2. 進入專案目錄
cd ~/Taiphoon_Agent

# 3. 建置
colcon build --packages-select taiphoon_agent

# 4. 載入工作區
source install/setup.bash
```

## 修改參數

可透過以下方式修改起飛高度與停懸時間：

### YAML 設定檔

編輯 `config/basic_controller.yaml`：

```yaml
basic_controller_node:
  ros__parameters:
    target_altitude_m: 5.0    # 改為 5 公尺
    hover_duration_s: 10.0    # 改為 10 秒
```

### 命令列參數

```bash
ros2 run taiphoon_agent basic_controller --ros-args \
    -p target_altitude_m:=5.0 \
    -p hover_duration_s:=10.0
```

### FormationNode 參數命令列

```bash
ros2 run taiphoon_agent formation_node --ros-args \
    -p drone_count:=4.0 \
    -p agent_namespace:=fmu1_agent \
    -p formation_update_rate_hz:=5.0
```

## 預期行為（單機）

啟動後，無人機將自動依序執行：

1. **解鎖（Arm）**：發送 VehicleCommand（CMD=400, param1=1）
2. **起飛（Takeoff）**：以 20Hz 頻率發送 OffboardControlMode + TrajectorySetpoint
3. **抵達目標高度後自動停懸**：維持高度直到秒數達標
4. **降落（Land）**：設定目標高度為 0，降至地面後進入 IDLE

## 預期行為（編隊）

1. 啟動 Launch 後，每台無人機各自執行 BasicController 自動 Offboard → Arm → Takeoff
2. 所有無人機起飛至目標高度後懸停
3. 透過 `/fmu_all_agent/command` 發送 `FORMATION_CIRCLE R H`，
   所有 FormationNode 同時收到命令，各自計算目標位置
4. 若某無人機離線（超過 2 秒），FormationNode 自動從編隊中排除，重新計算質心與角度分配
5. 透過 `FORMATION_CANCEL` 取消編隊，各機恢復懸停

## 啟動驗證流程（快速測試）

```bash
# 終端 1：啟動 4 台模擬器
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --net Taiphoon_Net

# 終端 2：啟動 Taiphoon_Agent
docker compose exec taiphoon-agt bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4

# 終端 3：編隊指令
docker compose exec taiphoon-agt bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source /home/taiphoon/Taiphoon_Agent/install/setup.bash

ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: ARM'
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: TAKEOFF 15'
ros2 topic pub --times 5 /fmu_all_agent/command std_msgs/msg/String 'data: FORMATION_CIRCLE 10 10'
```

## 日誌檢視

```bash
# 檢視 NED 目標（formation_node 輸出）
ros2 topic echo /fmu1_agent/ned_target

# 檢視所有 topic
ros2 topic list
```

## 故障排除

### 問題：PX4 未切換至 Offboard 模式
- 確認 `offboard_control_mode` topic 有持續發布（心跳）
- 確認 `auto_start_sequence` 參數為 true
- 多機模式下注意 `startup_delay_s` 是否造成延遲

### 問題：無人機不起飛
- 確認 `/fmu/in/trajectory_setpoint` 有正確的 Z 軸設定
- 確認 PX4 參數 `COM_RCL_EXCEPT` 設為 4（允許 Offboard）
- 使用 `ros2 topic echo /fmu{N}/fmu/out/vehicle_status` 確認狀態

### 問題：Launch 後找不到節點
- 確認已 source 所有必要 workspace：
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
  source /home/taiphoon/Taiphoon_Agent/install/setup.bash
  ```

### 問題：編隊未生效
- 確認 `drone_count > 1`（FormationNode 只在多機模式下啟動）
- 確認命令透過 `/fmu_all_agent/command` 而非單機 topic 發送
- 確認所有無人機的里程計 topic `/fmu{N}/fmu/out/vehicle_odometry` 有資料

### 問題：無人機離線後編隊未更新
- FormationNode 預設逾時為 2 秒（`DRONE_TIMEOUT_S`），等超過此時間後自動排除
- 可觀察控制台日誌確認是否有 "No drones online" 警告

### 問題：建置失敗
- 確認 `px4_msgs` 已安裝：`sudo apt install ros-humble-px4-msgs`
- 確認 `colcon` 已安裝：`sudo apt install python3-colcon-common-extensions`
- 確認自訂訊息目錄 `msg/` 存在正確的 `.msg` 檔案

### 自訂訊息（NedTarget）

```msg
float32 x     # NED X（前向為正，公尺）
float32 y     # NED Y（右向為正，公尺）
float32 z     # NED Z（向下為正，高度取負值，公尺）
float32 yaw   # 目標航向角（度，順時針為正；NaN 表示不控制偏航）
```

> `yaw` 設為 NaN 時，飛控會維持目前朝向，適用於編隊飛行中不強制轉向的情境。
> 自訂訊息定義位於 `/msg/NedTarget.msg`，建置時由 `rosidl_generate_interfaces` 自動產生對應的 C++ 型別。
