---
name: px4-sitl-launcher
description: 以 Docker 容器啟動與管理多台 PX4 SITL 模擬器，搭配獨立 uXRCE-DDS Agent 容器橋接 ROS2。用於：(1) 啟動指定數量的 PX4 SITL 實例（使用 SIH 內建模擬器），(2) 將 PX4 透過 uXRCE-DDS 連接至 ROS2，(3) 執行多無人機模擬（無需 Gazebo），(4) 設定 MAVLink 廣播讓 QGC 地面站連線，(5) 停止或重新啟動 PX4 SITL 實例。不適用於 ROS2 除錯或問題排除，請改用 ros2-tools 技能；不適用於 uXRCE-DDS Agent 獨立管理，請改用 uxrce_dds_agent-tools 技能。
---

# 功能名稱
PX4 SITL 模擬器 Docker 啟動管理

本技能以 Docker 容器管理多台 PX4 SITL 模擬器。每台無人機由兩個獨立容器組成：
- **uXRCE-DDS Agent 容器**（`drone{N}-uxrce-dds-agent`）：使用 `taiphoon/uxrce-agent` 映像，負責 PX4 與 ROS2 之間的 DDS 橋接
- **PX4 SITL 容器**（`drone{N}`）：使用 `px4io/px4-sitl:latest` 官方映像，執行 SIH 內建模擬器

所有容器加入指定的 Docker 網路（預設 `Taiphoon_Net`），透過 UDP multicast 進行 DDS 通訊。PX4 使用環境變數注入參數，不依賴 WSL2 本機環境或 tmux。

# 前置條件
- Docker 已安裝並可執行
- `taiphoon/uxrce-agent:latest` 映像已建構（`docker build -f docker/Dockerfile.uxrce-agent -t taiphoon/uxrce-agent:latest .`）
- `px4io/px4-sitl:latest` 官方映像已拉取（`docker pull px4io/px4-sitl:latest`）
- Docker 網路 `Taiphoon_Net` 已存在（腳本會自動建立）

# 基本使用方式

## 啟動多機模擬器
批次啟動指定數量的 PX4 SITL 實例與對應的 uXRCE-DDS Agent，預設數量為 3 台。
`bash scripts/launch_all.sh --count 3 --net Taiphoon_Net [--rmw fastrtps|cyclonedds]`

## 啟動單機模擬器（背景模式）
啟動單一 PX4 SITL 實例與 Agent，在 Docker 背景執行。
`bash scripts/start_single_instance.sh --index 0 --net Taiphoon_Net [--rmw fastrtps|cyclonedds]`

## 啟動單機模擬器（前景模式）
在前景啟動 PX4 SITL，適合除錯。Agent 仍在背景容器執行。按 Ctrl+C 停止。
`bash scripts/start_single_instance.sh --index 0 --net Taiphoon_Net --fg`

## 關閉所有模擬器
停止所有執行中的 drone 與 drone-uxrce-dds-agent 容器。
`bash scripts/stop_all.sh [--net Taiphoon_Net]`

# 進階使用方式

## 使用 CycloneDDS 啟動多機
當需要切換 DDS 中介軟體時，透過 `--rmw` 參數指定。
`bash scripts/launch_all.sh --count 3 --net Taiphoon_Net --rmw cyclonedds`

## 清除快取後重新啟動
當 DDS 通訊出現異常時，先關閉所有容器再重新啟動。
`bash scripts/stop_all.sh && bash scripts/launch_all.sh --count 3`

## 手動啟動單一實例（使用 docker run）
不使用腳本，直接啟動容器：
```bash
# Agent
docker run -d --rm --net=Taiphoon_Net --name=drone1-uxrce-dds-agent \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  taiphoon/uxrce-agent:latest udp4 -p 8888

# PX4 SITL
docker run -d --rm --net=Taiphoon_Net --name=drone1 \
  -p 18570:14550/udp \
  -e PX4_UXRCE_DDS_PORT=8888 \
  -e PX4_UXRCE_DDS_NS=fmu1 \
  -e PX4_SIM_MODEL=sihsim_quadx \
  -e PX4_SIMULATOR=sihsim \
  -e PX4_HOME_LAT=25.03996592258936 \
  -e PX4_HOME_LON=121.51195163336611 \
  px4io/px4-sitl:latest
```

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 模擬時不啟用 Gazebo 或其他視覺化工具以節省資源
- 初始位置固定為 (25.03996592258936, 121.51195163336611)，一字型排列間距 3 公尺
- 通訊埠管理規則：uXRCE-DDS=8888+N，MAVLink=18570+N，ROS2 namespace=fmu{N+1}
- 詳細通訊埠對應表請參閱 `references/port-mapping.md`
- Agent 容器與 PX4 容器生命週期獨立：PX4 重啟不影響 Agent
