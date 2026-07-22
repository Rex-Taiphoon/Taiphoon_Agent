# Taiphoon Agent BasicController 操作手冊

## 系統架構說明

BasicController 是 Taiphoon Agent 的基礎飛行控制模組，負責執行無人機的基本起降任務。
系統採用 SOLID 原則設計，包含以下層次：

- **核心介面層（core/）**：IController、IBasicFlightController 定義控制器抽象介面
- **工具層（utils/）**：math_utils 提供數學比較與數值合法性檢查
- **控制器層（controllers/）**：BasicController 實作起降邏輯與 PX4 Offboard 控制
- **節點層（nodes/）**：BasicControllerNode 封裝 ROS2 通訊與生命週期管理

### 狀態機流程

```
IDLE -> ARMED -> TAKEOFF -> HOVER -> LANDING -> IDLE
                                    |
                                    v
                                (上鎖完成)
```

## 依賴套件

- ROS2 Humble
- px4_msgs
- PX4-Autopilot（SIH 模擬器）

## 建置步驟

```bash
# 1. 載入 ROS2 環境
source /opt/ros/humble/setup.bash

# 2. 進入專案目錄
cd ~/Taiphoon_Agent

# 3. 確認 px4_msgs 已安裝
ros2 pkg list | grep px4_msgs

# 4. 建置
colcon build --packages-select taiphoon_agent

# 5. 載入工作區
source install/setup.bash
```

## 啟動步驟

### 前置作業：啟動 PX4 SITL 模擬器

```bash
cd ~/Taiphoon_Agent/skills/px4-sitl-launcher/scripts
bash launch_all.sh 3
```

### 啟動 BasicController

```bash
# 方式一：Launch 檔案（3 台無人機）
source ~/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py

# 方式二：單獨啟動一台
source ~/Taiphoon_Agent/install/setup.bash
ros2 run taiphoon_agent basic_controller --ros-args \
    -r __ns:=/fmu1 \
    -p target_altitude_m:=3.0 \
    -p hover_duration_s:=3.0
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

## 預期行為

啟動後，無人機將自動依序執行：

1. **解鎖（Arm）**：發送 VehicleCommand（CMD=400, param1=1）
2. **起飛（Takeoff）**：以 20Hz 頻率發送 OffboardControlMode + TrajectorySetpoint
3. **抵達目標高度後自動停懸**：維持高度直到秒數達標
4. **降落（Land）**：設定目標高度為 0，降至地面後自動上鎖

## 日誌檢視

```bash
# 檢視特定 namespace 的日誌
ros2 topic echo /fmu1/fmu/out/vehicle_odometry

# 檢視所有 topic
ros2 topic list
```

## 故障排除

### 問題：PX4 未切換至 Offboard 模式
- 確認 `offboard_control_mode` topic 有持續發布（心跳）
- 確認無人機已解鎖（ARMED 狀態）

### 問題：無人機不起飛
- 確認 `/fmu/in/trajectory_setpoint` 有正確的 Z 軸設定
- 確認 PX4 參數 `COM_RCL_EXCEPT` 設為 4（允許 Offboard）
- 使用 `ros2 topic echo /fmu1/fmu/out/vehicle_status` 確認狀態

### 問題：建置失敗
- 確認 `px4_msgs` 已安裝：`sudo apt install ros-humble-px4-msgs`
- 確認 `colcon` 已安裝：`sudo apt install python3-colcon-common-extensions`