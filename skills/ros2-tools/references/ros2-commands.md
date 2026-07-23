# ROS2 常用指令參考

## 基本指令

| 指令 | 說明 | 範例 |
|------|------|------|
| `ros2 topic list` | 列出所有活躍的主題 | `ros2 topic list` |
| `ros2 topic echo` | 監聽特定主題的訊息 | `ros2 topic echo /fmu/out/vehicle_attitude` |
| `ros2 topic info` | 顯示主題的詳細資訊 | `ros2 topic info /fmu/out/vehicle_attitude` |
| `ros2 topic hz` | 顯示主題的發布頻率 | `ros2 topic hz /fmu/out/vehicle_attitude` |
| `ros2 node list` | 列出所有節點 | `ros2 node list` |
| `ros2 node info` | 顯示節點的詳細資訊 | `ros2 node info /px4_0` |
| `ros2 service list` | 列出所有服務 | `ros2 service list` |
| `ros2 action list` | 列出所有動作 | `ros2 action list` |

## ROS2 Daemon 管理

```bash
# 檢查 daemon 狀態
ros2 daemon status

# 停止 daemon（必要時強制清理殘留行程）
ros2 daemon stop

# 啟動 daemon
ros2 daemon start

# 完整重啟流程
ros2 daemon stop && sleep 1 && ros2 daemon start && ros2 topic list
```