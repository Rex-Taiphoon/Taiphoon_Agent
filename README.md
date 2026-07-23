# Taiphoon_Agent

**無人機任務與感測器執行中樞**

Taiphoon_Agent 是一套裝載於無人載具上的自主飛行控制中樞，基於 **ROS2 (Humble)** 與 **PX4 / Ardupilot** 飛控系統構建，專注於多機編隊協作、邊緣運算與即時影像處理。系統採用容器化設計，可掛載於任何執行 Linux 核心的電腦上。

---

## 目錄

- [核心特色](#核心特色)
- [系統需求](#系統需求)
- [快速安裝](#快速安裝)
- [快速啟動（單機）](#快速啟動單機)
- [多機編隊啟動](#多機編隊啟動)
- [ROS2 主題參考](#ros2-主題參考)
- [高階命令列表](#高階命令列表)
- [參數設定](#參數設定)
- [自動測試](#自動測試)
- [Docker 部署](#docker-部署)
- [常見問題](#常見問題)
- [系統架構](#系統架構)
- [開發規範](#開發規範)

---

## 核心特色

- **多機編隊控制** — 支援鏈式編隊架構，失效自動重組隊形
- **ROS2 原生通訊** — 透過 uXRCE-DDS 與飛行控制器通訊
- **邊緣運算整合** — 即時影像識別、感測器融合、避障與追蹤演算法
- **影像串流轉發** — 向地面控制站傳遞即時影像
- **Docker 容器化** — 跨平台可攜，可掛載至任何 Linux 核心主機
- **高度冗餘** — 單機失效時系統持續運行，自動切換編隊任務

---

## 系統需求

| 項目 | 建議規格 |
|------|---------|
| 作業系統 | Linux (Ubuntu 22.04 以上) |
| ROS2 | Humble Hawksbill |
| PX4-Autopilot | v1.14+（SIH 模擬模式） |
| 編譯器 | GCC 11+ 或 Clang 14+ |
| Docker | 24.0+（容器化部署用） |
| 相依套件 | px4_msgs, rclcpp, std_msgs |

---

## 快速安裝

### 1. 安裝 ROS2 Humble 與相依套件

`ash
# 安裝 ROS2 Humble（如尚未安裝）
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 安裝 px4_msgs
sudo apt install ros-humble-px4-msgs
`

### 2. 建置 Taiphoon_Agent

`ash
# 載入 ROS2 環境
source /opt/ros/humble/setup.bash

# 進入專案目錄
cd ~/Taiphoon_Agent

# 建置
colcon build --packages-select taiphoon_agent

# 載入工作區
source install/setup.bash
`

### 3. 驗證安裝

`ash
# 確認執行檔已正確建置
ros2 pkg list | grep taiphoon_agent

# 確認執行檔存在
ls ~/Taiphoon_Agent/install/taiphoon_agent/lib/taiphoon_agent/
`

---

## 快速啟動（單機）

以下步驟使用 PX4 SIH（Software-In-the-Hoop）模擬器，不需實際硬體即可驗證功能。

### 步驟 1：啟動 PX4 SITL 模擬器

`ash
cd ~/Taiphoon_Agent/AutoTest

# 啟動 1 臺 PX4 SITL（含 MicroXRCEAgent）
bash launch_with_params.sh 1
`

此腳本會：
- 啟動 1 個 MicroXRCEAgent 實例（UDP port 8888）
- 啟動 1 個 PX4 SIH 模擬器，命名空間為 /fmu1
- 設定必要的 PX4 參數（COM_RCL_EXCEPT=4, COM_ARM_WO_GPS=1）
- 起始位置：(25.039966, 121.511952) — 臺灣臺北

### 步驟 2：啟動 Taiphoon_Agent BasicController

新開一個終端機：

`ash
source ~/Taiphoon_Agent/install/setup.bash

# 啟動單一 Agent，控制 /fmu1
ros2 run taiphoon_agent basic_controller --ros-args \
    -r __ns:=/fmu1 \
    -p target_altitude_m:=3.0 \
    -p hover_duration_s:=5.0 \
    -p target_system:=1
`

### 步驟 3：觀察行為

啟動後，Agent 會自動執行以下序列：

`
IDLE -> 解鎖 (ARM) -> 起飛至 3m -> 停懸 5秒 -> RTL 返航 -> 自動降落
`

在另一個終端機觀察狀態：

`ash
# 檢視飛行里程計
ros2 topic echo /fmu1/fmu/out/vehicle_odometry

# 檢視車輛狀態
ros2 topic echo /fmu1/fmu/out/vehicle_status
`

---

## 多機編隊啟動

### 使用 Launch 檔案啟動（建議）

`ash
source ~/Taiphoon_Agent/install/setup.bash

# 啟動 3 臺無人機的 Taiphoon_Agent（預設值）
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=3
`

Launch 檔案會自動為每臺無人機建立獨立節點：

| 無人機 | 命名空間 | Agent 命名空間 | 啟動延遲 |
|--------|---------|---------------|---------|
| #1 | /fmu1 | fmu1_agent | 0 秒 |
| #2 | /fmu2 | fmu2_agent | 2 秒 |
| #3 | /fmu3 | fmu3_agent | 4 秒 |

### 手動啟動多機

`ash
# 終端機 1：啟動 PX4 SITL（3 臺）
cd ~/Taiphoon_Agent/AutoTest
bash launch_with_params.sh 3

# 等待所有 PX4 就緒（約 10 秒）

# 終端機 2：啟動所有 Agent
source ~/Taiphoon_Agent/install/setup.bash
ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=3
`

### 解鎖與起飛

`ash
# 解鎖所有無人機並切換至 Offboard 模式
cd ~/Taiphoon_Agent/AutoTest
bash arm_all.sh
`

---

## ROS2 主題參考

### 通訊架構

`
                    ┌─────────────────────────────┐
                    │       地面控制站 (GCS)        │
                    │  (QGC / ROS2 客戶端)          │
                    └──────────┬──────────────────┘
                               │ ROS2 Topics
          ┌────────────────────┼────────────────────┐
          ▼                    ▼                    ▼
  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐
  │ Taiphoon_Agent│   │ Taiphoon_Agent│   │ Taiphoon_Agent│
  │     #1        │   │     #2        │   │     #N        │
  │ fmu1_agent    │◄──►│ fmu2_agent    │◄──►│ fmuN_agent    │
  └───────┬───────┘   └───────┬───────┘   └───────┬───────┘
          │ uXRCE-DDS         │ uXRCE-DDS         │ uXRCE-DDS
          ▼                    ▼                    ▼
  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐
  │   PX4 #1      │   │   PX4 #2      │   │   PX4 #N      │
  │   /fmu1       │   │   /fmu2       │   │   /fmuN       │
  └───────────────┘   └───────────────┘   └───────────────┘
`

### 訂閱主題（Agent 接收）

| 主題 | 型別 | 說明 |
|------|------|------|
| /<agent_ns>/command | std_msgs/String | 特定 Agent 的高階命令 |
| /fmu_all_agent/command | std_msgs/String | 全體 Agent 統一命令 |
| /fmu/fmu/out/vehicle_odometry | px4_msgs/VehicleOdometry | PX4 里程計資料 |
| /fmu/fmu/out/vehicle_status | px4_msgs/VehicleStatus | PX4 車輛狀態 |
| /fmu/fmu/out/vehicle_command_ack | px4_msgs/VehicleCommandAck | 命令確認回覆 |

> **注意**：agent_ns 格式為 fmu{id}_agent，例如 fmu1_agent、fmu2_agent。

### 發布主題（Agent 發送）

| 主題 | 型別 | 說明 |
|------|------|------|
| /fmu/fmu/in/offboard_control_mode | px4_msgs/OffboardControlMode | Offboard 控制模式心跳 |
| /fmu/fmu/in/trajectory_setpoint | px4_msgs/TrajectorySetpoint | 位置/速度設定點 |
| /fmu/fmu/in/vehicle_command | px4_msgs/VehicleCommand | PX4 命令（Arm/Disarm/模式切換） |

---

## 高階命令列表

可透過 ROS2 主題傳送以下高階命令字串至 Agent：

### 傳送命令範例

`ash
# 對特定 Agent 下指令
ros2 topic pub /fmu1_agent/command std_msgs/String "data: 'TAKEOFF 5.0'" --once

# 對所有 Agent 下指令
ros2 topic pub /fmu_all_agent/command std_msgs/String "data: 'RTL'" --once
`

### 命令格式

命令格式為：<命令> [參數1] [參數2] ...

| 命令 | 參數 | 說明 |
|------|------|------|
| ARM | 無 | 解鎖無人機 |
| DISARM | 無 | 上鎖無人機 |
| TAKEOFF | <高度_m> | 起飛至指定高度 |
| HOLD | <持續時間_s> | 在空中停懸指定秒數 |
| LAND | 無 | 切換至降落模式 |
| RTL | 無 | 返航（Return To Launch） |
| OFFBOARD | 無 | 切換至 Offboard 模式 |
| MOVE_FORWARD | <距離_m> | 向前移動指定距離 |
| MOVE_RIGHT | <距離_m> | 向右移動指定距離 |
| TAKEOFF_LAND | <高度_m> <停懸秒數> | 完整起飛→停懸→降落序列 |

---

## 參數設定

### 透過 YAML 設定檔

編輯 config/basic_controller.yaml：

`yaml
basic_controller_node:
  ros__parameters:
    target_altitude_m: 5.0    # 目標起飛高度（公尺）
    hover_duration_s: 10.0    # 停懸持續時間（秒）
`

### 透過命令列覆寫

`ash
ros2 run taiphoon_agent basic_controller --ros-args \
    -r __ns:=/fmu1 \
    -p target_altitude_m:=5.0 \
    -p hover_duration_s:=10.0 \
    -p target_system:=1 \
    -p startup_delay_s:=0.0
`

| 參數 | 型別 | 預設值 | 說明 |
|------|------|--------|------|
| target_altitude_m | double | 3.0 | 目標起飛高度（公尺） |
| hover_duration_s | double | 3.0 | 停懸持續時間（秒） |
| target_system | int | 1 | PX4 目標系統 ID |
| startup_delay_s | double | 0.0 | 啟動延遲（秒） |
| agent_namespace | string | fmu1 | Agent 命名空間 |

---

## 自動測試

自動測試腳本位在 AutoTest/ 目錄下，用於驗證各項功能：

### 可用測試腳本

| 腳本 | 說明 |
|------|------|
| launch_with_params.sh [N] | 啟動 N 臺 PX4 SITL + MicroXRCEAgent |
| arm_all.sh | 解鎖所有無人機並切換至 Offboard 模式 |
| final_test.sh | 完整 Offboard 模式驗證測試 |
| quick_test.sh | 快速功能測試 |
| full_test.sh | 完整編隊功能測試 |
| check_px4.sh | 檢查 PX4 實例狀態 |
| restart_all.sh | 重新啟動所有實例 |
| verify_params.sh | 驗證 PX4 參數設定 |

### 執行測試

`ash
cd ~/Taiphoon_Agent/AutoTest

# 步驟 1：啟動模擬環境
bash launch_with_params.sh 3

# 步驟 2：執行驗證測試
bash final_test.sh
`

測試紀錄會自動儲存至 AutoTest/Log/ 目錄。

---

## Docker 部署

Taiphoon_Agent 已設計為可透過 Docker 掛載執行。

### 建立 Docker 映像（範例）

`dockerfile
FROM ros:humble-ros-base

# 安裝相依套件
RUN apt-get update
RUN apt-get install -y ros-humble-px4-msgs python3-colcon-common-extensions
RUN rm -rf /var/lib/apt/lists/*

# 複製原始碼
COPY . /workspace/taiphoon_agent

# 建置
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; cd /workspace/taiphoon_agent; colcon build --packages-select taiphoon_agent"

# 進入點
CMD ["/bin/bash", "-c", "source /workspace/taiphoon_agent/install/setup.bash; exec bash"]
`

### 執行容器

`ash
docker build -t taiphoon_agent .
docker run -it --rm --network host taiphoon_agent
`

> --network host 為必要，因為 Agent 需透過 UDP 與 PX4 SITL / MicroXRCEAgent 通訊。

---

## 常見問題

### Q: PX4 未切換至 Offboard 模式？

確認以下項目：
- Offboard 控制模式心跳持續發布（20Hz）
- PX4 參數 COM_RCL_EXCEPT 設為 4（允許 Offboard 命令）
- 無人機已解鎖（ARMED 狀態）

`ash
# 檢查參數
tmux send-keys -t sitl_0 "param show COM_RCL_EXCEPT" Enter
`

### Q: 無人機不起飛？

- 確認 /fmu/fmu/in/trajectory_setpoint 有正確的 Z 軸負值設定
- 檢查 PX4 參數 NAV_RCL_ACT 設為 0（RC 遺失時不動作）
- 使用 ros2 topic echo /fmu1/fmu/out/vehicle_status 確認狀態

### Q: 編隊中某臺無人機失效？

系統具備冗餘機制：
- 其餘 Agent 會自動偵測到無人機失效
- 編隊任務會自動重組，移除失效無人機
- 整套系統持續運行不受影響

### Q: 建置失敗？

`ash
# 確認 px4_msgs 已安裝
ros2 pkg list | grep px4_msgs

# 如未安裝
sudo apt install ros-humble-px4-msgs

# 確認 colcon 已安裝
sudo apt install python3-colcon-common-extensions
`

---

## 系統架構

### 核心模組

| 模組 | 路徑 | 說明 |
|------|------|------|
| 核心介面 | src/core/ | IController, IBasicFlightController 抽象介面定義 |
| 控制器 | src/controllers/ | BasicController 實作飛行控制邏輯（SOLID 原則） |
| 命令處理 | src/controllers/ | AgentCommandHandler 高階命令解析與路由 |
| 節點層 | src/nodes/ | BasicControllerNode ROS2 節點封裝與生命週期管理 |
| 工具 | src/utils/ | CommandTypes, math_utils 共用工具 |
| 設定檔 | config/ | YAML 參數設定 |
| Launch | launch/ | ROS2 Launch 檔案（支援多機參數化啟動） |
| 自動測試 | AutoTest/ | 測試腳本與紀錄 |

### 狀態機流程

`
IDLE -> OFFBOARD_INIT -> ARMED -> TAKEOFF -> HOVER -> RTL -> IDLE
                                           |
                                           ▼
                                        MOVING (水平移動)
                                           |
                                           ▼
                                        LAND
`

---

## 開發規範

- **語言**：C++ 為主，必要時輔以 Python / C
- **編碼風格**：Allman 括號、Doxygen 註解、遵循 SOLID 原則
- **命名慣例**：請參閱 CODE_CONSTRAINT.md
- **提交格式**：模組: 更動項目（簡要描述，100 字內）

---

## 授權條款

本專案為 Taiphoon 所屬，保留所有權利。
