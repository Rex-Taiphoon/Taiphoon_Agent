---
name: px4-sitl-launcher
description: 啟動與管理多台 PX4 SITL 模擬器，搭配 uXRCE-DDS MicroXRCEAgent 橋接以整合 ROS2。用於：(1) 啟動指定數量的 PX4 SITL 實例（使用 SIH 內建模擬器），(2) 將 PX4 透過 uXRCE-DDS 連接至 ROS2，(3) 執行多無人機模擬（無需 Gazebo），(4) 設定 MAVLink 廣播讓 QGC 地面站連線，(5) 停止或重新啟動 PX4 SITL 實例。不適用於 ROS2 除錯或問題排除，請改用 ros2-tools 技能。
---

# 功能名稱
PX4 SITL 模擬器啟動管理

本技能負責啟動與管理多台 PX4 SITL（Software-In-The-Loop）模擬器實例，使用內建 SIH（Simulation-In-Hardware）模擬器而非 Gazebo 等外部視覺化工具，以節省系統資源。每個 PX4 實例皆透過 uXRCE-DDS 通訊協定搭配 MicroXRCEAgent 與 ROS2 進行資料交換，並自動設定 MAVLink 廣播讓 QGroundControl 地面站可連線監控。

主要功能涵蓋：批次啟動多台無人機模擬器、單機啟動（含前景除錯模式）、自動計算一字型編隊初始位置（以台北車站為基準點，間距 3 公尺）、統一的通訊埠管理（uXRCE-DDS 從 8888 起遞增、MAVLink GCS 從 18570 起遞增），以及一次性關閉所有模擬器實例。

# 前置條件
- 必須先建置 PX4-Autopilot 原始碼（`px4_sitl_default` target），請使用 px4-builder 技能
- 必須安裝 ROS2 Humble（預設路徑 `/opt/ros/humble`）
- 必須安裝 MicroXRCEAgent（預設路徑 `/usr/local/bin/MicroXRCEAgent`）
- 執行環境為 WSL2 Ubuntu 22.04
- 需要安裝 tmux 以管理背景模擬器會話

# 基本使用方式
以下為本技能提供的基本腳本，每個腳本對應一項核心功能。

## 啟動多機模擬器
批次啟動指定數量的 PX4 SITL 實例與對應的 MicroXRCEAgent，預設數量為 3 台。
`bash scripts/launch_all.sh [數量]`

## 啟動單機模擬器（背景模式）
啟動單一 PX4 SITL 實例與 MicroXRCEAgent，在 tmux 背景會話中執行。
`bash scripts/start_single_instance.sh <實例編號>`

## 啟動單機模擬器（前景模式）
在前景終端機中啟動單一 PX4 SITL 實例，適合除錯使用。MicroXRCEAgent 仍在 tmux 背景執行。
`bash scripts/start_single_instance_foreground.sh <實例編號>`

## 關閉所有模擬器
停止所有執行中的 PX4 SITL 實例與 MicroXRCEAgent，包含清理 tmux 會話與殘留行程。
`bash scripts/stop_all.sh`

# 進階使用方式
以下為常見使用場景的指令組合方式，由 Agent 根據需求自行組合基本腳本。

## 從 Windows 端啟動（PowerShell）
當需要在 Windows 終端機中開啟獨立視窗來觀察各無人機輸出時，使用 PowerShell 腳本啟動。
`powershell -ExecutionPolicy Bypass -File scripts/launch_windows.ps1 -Count 3`

## 清除快取後重新啟動
當 DDS 通訊出現異常或需要乾淨的模擬環境時，先清除 rootfs 快取再重新啟動。
`rm -rf "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/" && bash scripts/launch_all.sh 3`

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 模擬時不啟用 Gazebo 或其他視覺化工具以節省資源
- 初始位置固定為 (25.03996592258936, 121.51195163336611)，一字型排列間距 3 公尺
- 通訊埠管理規則：uXRCE-DDS=8888+N，MAVLink=18570+N，ROS2 namespace=fmu{N+1}
- 詳細通訊埠對應表請參閱 `references/port-mapping.md`