# ROS2 常見問題排除

## 問題 1：ros2 topic list 無法列出或回應緩慢

**症狀**：ROS2 daemon 未執行或狀態異常。
**解決方案**：
```bash
# 步驟 1：檢查 daemon
ros2 daemon status

# 步驟 2：強制重啟 daemon
ros2 daemon stop
ps aux | grep -E "ros|daemon" | grep -v grep
ros2 daemon start

# 步驟 3：驗證
ros2 topic list
```

## 問題 2：ROS2 topic 訊息遺失或延遲

**症狀**：部分 topic 訊息未收到或延遲過高。
**解決方案**：
- 檢查 DDS 實作設定：`echo $RMW_IMPLEMENTATION`，預設應為 `rmw_fastrtps_cpp`
- 調整 DDS 的 QoS 設定為 Best Effort 模式
- 檢查是否有節點崩潰：`ros2 node list` 對比預期節點數

## 問題 3：ROS2 node 無法被發現

**症狀**：已啟動的節點在 `ros2 node list` 中未顯示。
**解決方案**：
- 確認 ROS_DOMAIN_ID 設定一致（所有節點必須使用相同的 domain ID）
- 重新啟動 daemon：`ros2 daemon stop && ros2 daemon start`
- 檢查節點執行行程是否存活：`ps aux | grep <節點名稱>`

## 問題 4：ROS2 訊息類型不相符

**症狀**：topic echo 報錯「message type mismatch」。
**解決方案**：
- 使用 `ros2 topic info <topic>` 確認訊息類型
- 確認發布者與訂閱者使用相同的訊息定義