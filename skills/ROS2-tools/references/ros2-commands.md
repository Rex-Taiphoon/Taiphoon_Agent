# ROS2 常用指令參考

## 基本指令

| 指令 | 說明 | 範例 |
|------|------|------|
| `ros2 topic list` | 列出所有活躍主題 | `ros2 topic list` |
| `ros2 topic echo` | 監聽特定主題資料 | `ros2 topic echo /fmu/out/vehicle_attitude` |
| `ros2 topic info` | 查詢主題詳細資訊 | `ros2 topic info /fmu/out/vehicle_attitude` |
| `ros2 node list` | 列出所有節點 | `ros2 node list` |
| `ros2 node info` | 查詢節點詳細資訊 | `ros2 node info /px4_0` |
| `ros2 service list` | 列出所有服務 | `ros2 service list` |
| `ros2 action list` | 列出所有動作 | `ros2 action list` |

## ROS2 Daemon 管理

```bash
# 檢查 daemon 狀態
ros2 daemon status

# 停止 daemon（遇到主題列表問題時先執行）
ros2 daemon stop

# 啟動 daemon
ros2 daemon start

# 完整重啟流程
ros2 daemon stop && sleep 1 && ros2 daemon start && ros2 topic list
```

## WSL2 網路注意事項

在 WSL2 環境中，ROS2 使用 DDS 發現協定，需注意：

1. 如果使用 `ros2 topic list` 出現連線拒絕錯誤，請重啟 daemon
2. 若跨 WSL2 與 Windows 主機通訊，需設定 `ROS_DOMAIN_ID` 環境變數
3. 建議在同一個 WSL2 執行個體中執行所有 ROS2 節點
4. 可使用 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 指定 DDS 實作
