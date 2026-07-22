---
name: ros2-tools
description: "ROS2 工具與問題排除。用於：(1) 檢視與除錯 ROS2 topics、services、actions，(2) 處理 ROS2 daemon 問題，(3) 重新啟動 MicroXRCEAgent 與 ROS2 橋接，(4) 檢查 ROS2 節點狀態與網路連線，(5) 處理 WSL2 環境下的 ROS2 網路問題。不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能。"
---

# ROS2 工具集

本技能提供 ROS2 日常操作與問題排除的輔助工具，協助您在 WSL2 + PX4 環境中快速診斷 ROS2 相關問題。

## 前置需求

- ROS2 Humble 安裝於 `/opt/ros/humble`
- MicroXRCEAgent 已安裝（`/usr/local/bin/MicroXRCEAgent`）
- PX4 SITL 模擬器已啟動（使用 px4-sitl-launcher 技能）

## 技能目錄結構

```
skills/ROS2-tools/
├── SKILL.md
├── agents/openai.yaml
├── references/
│   ├── ros2-commands.md         # ROS2 指令參考
│   └── ros2-troubleshooting.md  # 問題排除
├── assets/
└── scripts/
    └── restart_agent.sh         # MicroXRCEAgent 重啟腳本
```

## 常用操作

### 檢查 ROS2 主題列表

當 `ros2 topic list` 無法顯示主題時，通常是 ROS2 daemon 未正確啟動：

```bash
source /opt/ros/humble/setup.bash
ros2 daemon stop    # 停止目前 daemon
ros2 daemon start   # 重新啟動 daemon
ros2 topic list     # 重新查詢主題列表
```

### 確認 PX4 發布的主題

PX4 透過 uXRCE-DDS 發布的主題範例（當 daemon 正常運作時）：

- `/fmu/out/vehicle_attitude` — 飛行姿態
- `/fmu/out/vehicle_global_position` — GPS 全域位置
- `/fmu/out/sensor_combined` — IMU 感測器資料
- `/fmu/in/vehicle_command` — 飛行指令

### 檢查所有 ROS2 節點

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

### 檢查特定主題的資料

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /fmu/out/vehicle_attitude
```

### 檢查 MicroXRCEAgent 執行狀態

```bash
ps aux | grep MicroXRCEAgent
```

每個 PX4 實例對應一個 MicroXRCEAgent 程序，分別監聽不同的 UDP 埠（8888、8889、8890...）。

### 重新啟動 ROS2 相關程序

```bash
# 重新啟動 MicroXRCEAgent（指定實例編號）
bash scripts/restart_agent.sh 0
```

## 參考資料

- 連接埠對應表請參閱 `references/ros2-commands.md`
- 常見問題排除請參閱 `references/ros2-troubleshooting.md`
