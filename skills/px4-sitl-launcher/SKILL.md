---
name: px4-sitl-launcher
description: "啟動與管理多台 PX4 SITL 模擬器，搭配 uXRCE-DDS MicroXRCEAgent 橋接以整合 ROS2。用於：(1) 啟動指定數量的 PX4 SITL 實例（使用 SIH 內建模擬器），(2) 將 PX4 透過 uXRCE-DDS 連接至 ROS2，(3) 執行多無人機模擬（無需 Gazebo），(4) 設定 MAVLink 廣播讓 QGC 地面站連線，(5) 停止或重新啟動 PX4 SITL 實例。不適用於 ROS2 除錯或問題排除，請改用 ros2-tools 技能。"
---

# PX4 SITL 啟動器

本技能提供啟動與管理多台 PX4 SITL 模擬器的功能，使用 SIH（Software-In-The-Loop）內建模擬器，不需要安裝 Gazebo。

## 前置需求

- PX4-Autopilot 已編譯（`px4_sitl_default` target），請使用 px4-builder 技能建構
- ROS2 Humble 安裝於 `/opt/ros/humble`
- MicroXRCEAgent 已安裝（`/usr/local/bin/MicroXRCEAgent`）
- WSL2 Ubuntu 22.04 環境

## 技能目錄結構

```
skills/px4-sitl-launcher/
├── SKILL.md              # 本檔案
├── agents/openai.yaml    # UI 中繼資料
├── references/           # 參考文件
├── assets/               # 圖示資源
└── scripts/
    ├── launch_all.sh           # 批次啟動（指定數量）
    ├── launch_windows.ps1      # Windows 終端機啟動器
    ├── start_single_instance.sh # 啟動單一實例
    ├── start_single_instance_foreground.sh # 前景啟動（終端機視窗用）
    └── stop_all.sh             # 停止所有實例
```

## 啟動無人機

### 在 WSL 內啟動（指定數量）

```bash
# 啟動 3 台，修改數字可調整數量
bash scripts/launch_all.sh 3
```

腳本會自動使用 `$HOME/PX4-Autopilot/` 路徑，適應不同使用者環境。

### 透過 Windows PowerShell 啟動（獨立終端機視窗）

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_windows.ps1 -Count 3
```

每個視窗會顯示 PX4 的即時輸出，按 Ctrl+C 可停止該實例。

## 檢查執行狀態

```bash
# 檢視所有 PX4 與 MicroXRCEAgent 行程
ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep

# 檢視特定實例的日誌
tail -5 /tmp/px4_instance_0/px4_0.log

# 附加到 tmux 工作階段
tmux attach -t sitl_0     # PX4 實例 0
tmux attach -t agent_0    # MicroXRCEAgent 實例 0
```

## 停止所有模擬

```bash
bash scripts/stop_all.sh
```

不須指定數量，會自動停止所有執行中的 PX4 與 MicroXRCEAgent。

## 連接埠對應

| 實例編號 | uXRCE-DDS 埠號 | MAVLink GCS 埠號 | MAV_SYS_ID | ROS2 命名空間 |
|----------|---------------|------------------|------------|----------------|
| 0        | 8888          | 18570            | 1          | fmu1           |
| 1        | 8889          | 18571            | 2          | fmu2           |
| 2        | 8890          | 18572            | 3          | fmu3           |
| N        | 8888+N        | 18570+N          | N+1        | fmu{N+1}       |

## 位置設定

- 基準位置：`(25.03996592258936, 121.51195163336611)` — 台北
- 實例由左往右排列，間距約 3 公尺（經度偏移）
- Instance 0 在最左側（基準點），Instance N 在右側 N×3m
- MAVLink 使用廣播模式（`MAV_0_BROADCAST=1`，`PX4_NET_INTERFACE=eth0`）

## 清除暫存後重啟

```bash
rm -rf "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/"
bash scripts/launch_all.sh 3
```

## 參考資料

- 連接埠對應詳細說明請參閱 `references/port-mapping.md`
- 建構 PX4 環境請使用 px4-builder 技能
- ROS2 問題排除請使用 ros2-tools 技能