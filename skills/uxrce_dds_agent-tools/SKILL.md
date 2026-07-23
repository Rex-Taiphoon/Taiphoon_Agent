---
name: uxrce_dds_agent-tools
description: uXRCE-DDS Agent 管理工具，用於 PX4 與 ROS2 之間的 DDS 通訊橋接。用於：(1) 啟動 MicroXRCEAgent 以橋接 PX4 與 ROS2，(2) 重新啟動特定實例的 MicroXRCEAgent，(3) 檢查 MicroXRCEAgent 連線狀態與通訊埠，(4) 診斷 uXRCE-DDS 通訊異常。不適用於 ROS2 指令操作，請改用 ros2-tools 技能；不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能。
---

# 功能名稱
uXRCE-DDS Agent 通訊橋接管理

本技能負責管理 PX4 飛控與 ROS2 之間的 uXRCE-DDS（Micro XRCE-DDS）通訊橋接。MicroXRCEAgent 是連接 PX4 韌體（透過 UART/UDP）與 ROS2 DDS 網路的中介程式，每個 PX4 SITL 實例需要一個獨立的 Agent 實例，使用對應的 UDP 通訊埠（8888 起遞增）。

主要功能涵蓋：啟動 MicroXRCEAgent、重新啟動異常的 Agent 實例、檢查 Agent 執行狀態與通訊埠佔用情況，以及診斷 uXRCE-DDS 連線問題。

# 前置條件
- MicroXRCEAgent 已安裝（預設路徑 `/usr/local/bin/MicroXRCEAgent`）
- PX4 SITL 模擬器已啟動（請使用 px4-sitl-launcher 技能）
- ROS2 Humble 已安裝（預設路徑 `/opt/ros/humble`）
- 每次啟動 Agent 前需 `source /opt/ros/humble/setup.bash`

# 基本使用方式
以下為本技能提供的 MicroXRCEAgent 基本操作。

## 重新啟動特定實例的 Agent
當某個 PX4 實例的 MicroXRCEAgent 連線異常或無回應時，重新啟動該實例的 Agent。
`bash scripts/restart_agent.sh <實例編號>`

## 直接啟動 Agent（手動）
手動啟動 MicroXRCEAgent 監聽指定通訊埠，不依賴腳本。
`MicroXRCEAgent udp4 -p <通訊埠>`

# 進階使用方式
以下為常見使用場景的指令組合方式，由 Agent 根據需求自行組合基本指令。

## 檢查所有 Agent 連線狀態
確認所有 MicroXRCEAgent 是否正常執行及其監聽的通訊埠。
```bash
ps aux | grep MicroXRCEAgent
ss -tulpn | grep -E "8888|8889|8890"
```

## 強制關閉並重啟所有 Agent
當多個 Agent 同時異常時，批次清理並重新啟動。
```bash
pkill -f MicroXRCEAgent
# 再透過 px4-sitl-launcher 重新啟動所有實例
```

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 每個 PX4 實例對應一個獨立的 MicroXRCEAgent，通訊埠規則為 8888+N
- Agent 通訊埠必須與 PX4 實例的 `PX4_UXRCE_DDS_PORT` 設定一致
- 若 Agent 無法連線，應先檢查 PX4 實例是否正常執行再重新啟動 Agent