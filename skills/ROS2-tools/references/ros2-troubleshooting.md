# ROS2 常見問題排除

## 問題 1：ros2 topic list 無輸出或顯示空白

**原因**：ROS2 daemon 未正確啟動或當機。

**解決方法**：
```bash
# 步驟 1：停止 daemon
ros2 daemon stop

# 步驟 2：確認所有相關程序已終止
ps aux | grep -E "ros|daemon" | grep -v grep

# 步驟 3：重新啟動 daemon
ros2 daemon start

# 步驟 4：驗證
ros2 topic list
```

## 問題 2：MicroXRCEAgent 連接失敗

**原因**：PX4 無法連接到 MicroXRCEAgent，或埠號被占用。

**解決方法**：
```bash
# 檢查 MicroXRCEAgent 是否執行中
ps aux | grep MicroXRCEAgent

# 檢查埠號占用情況
ss -tulpn | grep -E "8888|8889|8890"

# 重新啟動代理程序
pkill -f MicroXRCEAgent
# 或使用 restart_agent.sh 重新啟動特定實例
```

## 問題 3：無法看到 PX4 發布的主題

**原因**：PX4 的 uXRCE-DDS 客戶端未正確初始化。

**解決方法**：
```bash
# 檢查 PX4 實例是否執行中
ps aux | grep bin/px4

# 確認 PX4 啟動參數中的 DDS 埠號設定
# PX4_UXRCE_DDS_PORT 必須與 MicroXRCEAgent 的埠號一致

# 必要時清除暫存檔後重啟 PX4
rm -rf "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/"
```

## 問題 4：ROS2 發布延遲或掉封包

**原因**：WSL2 網路效能問題或系統資源不足。

**解決方法**：
- 減少同時運行的模擬實例數量
- 在 WSL2 中執行 `sudo sysctl -w net.core.rmem_max=26214400`
- 確認 WSL2 有足夠的記憶體分配（建議至少 8GB）
