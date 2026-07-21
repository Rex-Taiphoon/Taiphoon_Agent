---
name: px4-sitl-launcher
description: "Launch and manage multiple PX4 SITL simulators with uXRCE-DDS MicroXRCEAgent bridges for ROS2 integration. Use when: (1) starting 1-3 PX4 SITL instances with SIH (Software-In-The-Loop) built-in simulator, (2) connecting PX4 to ROS2 via uXRCE-DDS agents, (3) running multi-vehicle simulation without Gazebo, (4) setting up MAVLink broadcast for QGC connection, (5) stopping or restarting PX4 SITL instances."
---

# PX4 SITL Launcher

啟動多個 PX4 SITL 模擬器（SIH 模式）與對應的 MicroXRCEAgent，將 PX4 資料橋接至 ROS2。

## 前置需求

- PX4-Autopilot 已編譯 `px4_sitl_default` target
- ROS2 Humble 已安裝 (`/opt/ros/humble`)
- MicroXRCEAgent 已安裝 (`/usr/local/bin/MicroXRCEAgent`)
- WSL2 Ubuntu 22.04 環境

## 啟動 3 臺模擬器

### 方式一：從 PowerShell 啟動（推薦）

Powershell 模式下，使用 `[System.Diagnostics.Process]::Start()` 啟動 WSL 中的 `setsid -w` 程序，確保模擬器在 WSL 中持久執行。

```powershell
# 啟動 3 臺 PX4 SITL + MicroXRCEAgent
& {
    $path = "\\wsl.localhost\Ubuntu-22.04\home\taiphoon\Taiphoon_Agent\skills\px4-sitl-launcher\scripts"
    for ($i = 0; $i -le 2; $i++) {
        $psi = [System.Diagnostics.ProcessStartInfo]::new()
        $psi.FileName = "wsl"
        $psi.Arguments = "-d Ubuntu-22.04 bash $path/start_single_instance.sh $i"
        $psi.UseShellExecute = $false
        $psi.CreateNoWindow = $true
        $null = [System.Diagnostics.Process]::Start($psi)
        Start-Sleep -Seconds 3
    }
}
```

**注意**：啟動命令中「不可以」加尾隨 `&`，否則 WSL 退出時會殺死子程序。

### 方式二：在 WSL 內直接執行

```bash
bash /home/taiphoon/Taiphoon_Agent/skills/px4-sitl-launcher/scripts/launch_all.sh
```

## 確認模擬器執行狀態

```bash
# 在 WSL 內檢查程序
ps aux | grep -E "MicroXRCEAgent|bin/px4" | grep -v grep

# 檢查日誌
tail -5 /tmp/px4_instance_0/px4_0.log
```

## 檢視 ROS2 Topics

若 `ros2 topic list` 卡住無響應，通常是 ROS2 daemon 狀態異常。請先重啟 daemon：

```bash
source /opt/ros/humble/setup.bash
ros2 daemon stop    # 先停止舊 daemon
ros2 daemon start   # 重新啟動 daemon
ros2 topic list     # 應該能正常顯示
```

啟動 daemon 後即可看到 PX4 釋出的 topics，例如：
- `/fmu/out/vehicle_attitude` - 姿態資訊
- `/fmu/out/vehicle_global_position` - GPS 位置
- `/fmu/out/sensor_combined` - IMU 資料
- `/fmu/in/vehicle_command` - 指令輸入

## 關閉所有模擬器

```bash
bash /home/taiphoon/Taiphoon_Agent/skills/px4-sitl-launcher/scripts/stop_all.sh
```

## 架構說明

每臺 Instance 的配置：

| Instance | uXRCE-DDS Port | MAVLink GCS Port | MAV_SYS_ID | ROS2 Namespace |
|----------|---------------|------------------|------------|----------------|
| 0        | 8888          | 18570            | 1          | fmu1           |
| 1        | 8889          | 18571            | 2          | fmu2           |
| 2        | 8890          | 18572            | 3          | fmu3           |

- 模擬器: PX4 SIH (Software-In-The-Hoop), built-in quadcopter dynamics, no Gazebo required
- 起點座標: (25.03996592258936, 121.51195163336611) - 臺北
- MAVLink: 已啟用 broadcast (`MAV_0_BROADCAST=1`, `PX4_NET_INTERFACE=eth0`)
- QGC: 可透過 UDP 連線 WSL IP (`172.20.10.2`) 到對應 GCS Port

## 重啟模擬器（清空引數狀態）

PX4 每次啟動會保留前次飛的引數。若需要完全重置：

```bash
rm -rf /home/taiphoon/PX4-Autopilot/build/px4_sitl_default/rootfs/
bash /home/taiphoon/Taiphoon_Agent/skills/px4-sitl-launcher/scripts/launch_all.sh
```

## 注意事項

- 檢視 tmux 輸出: `tmux capture-pane -t sitl_0 -p`
- 附加到 tmux: `tmux attach -t sitl_0`
- 若 ros2 topic list 卡住，先執行 `ros2 daemon stop && ros2 daemon start`
- WSL 內路徑: `/home/taiphoon/PX4-Autopilot/build/px4_sitl_default`
