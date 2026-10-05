# 樹莓派 release 運行驗證（2026-10-05）

主角色：Docker／部署維護人員；兼任測試與驗證責任。

## 結論

最新已發布版本 `v1.3.0-rc.3`（預發布版）的完整 ARM64 成品可在 Raspberry Pi 5 上成功運行。
未接飛控時，容器健康、必要節點正常、uXRCE-DDS UDP 8888 等待連線，規劃器持續發布定位未就緒狀態。
測試結束保留新版 Agent 運行。此結果不涵蓋實體飛控、飛行命令、定位、感測器或跨主機 GCS 資料接收驗收。

## 環境與成品

- SSH 使用 Wi-Fi `192.168.0.110`，帳號 `taiphoon`；未使用 Ethernet 登入。
- Raspberry Pi 5 Model B Rev 1.1、4 核 Cortex-A76、16 GB RAM。
- Ubuntu 24.04.4 LTS、kernel 6.8.0-1062-raspi、aarch64。
- Docker Engine 29.7.2、Compose v5.5.0；磁碟可用約 205 GB。
- 實測 wlan0 為 `192.168.0.110/24`，default route 使用 wlan0；當時未顯示 Ethernet IPv4。
- 透過已認證 GitHub Releases API 確認最新發布成品，下載 asset 606146202：
  `taiphoon_agent-v1.3.0-rc.3-linux-arm64.tar.gz`，853,130,569 bytes。
- 下載檔與樹莓派接收檔 SHA256 相同：
  `c04cf7dd06d098ce09d5318aa60b7515cc03c4e1825d58321096b3e35f722c99`。
- 原生 launcher 回報 `v1.3.0-rc.3`，manifest 同版；runtime image 確認為 linux/arm64。
- 使用 release 隨附的 runtime 與 dev 映像，沒有 Docker build、原始碼建置或主機 ROS2 指令。

## 測試項目、步驟與結果

測試前提出單機 deploy、無飛控啟動、DDS 唯讀觀測與啟停驗證計畫。
設定為 deploy、Agent 1、ROS domain 0、UDP、Wi-Fi，GCS IP 留空，使用獨立設定目錄。

| 項目 | 實際結果 |
|---|---|
| 首次啟動 | launcher 自動載入 release runtime 映像，start exit 0，容器 healthy |
| 網路自動選擇 | 取得 `192.168.0.110`，MicroXRCEAgent 監聽 `0.0.0.0:8888` |
| 必要節點 | DDS 發現 `/uav_1/basic_controller_node`、`/uav_1/ego_swarm_planner`、`/uav_1_agent/agent_supervisor` |
| 獨立 DDS 觀測 | release dev 容器經 host network、CycloneDDS、domain 0 觀測 runtime |
| 規劃器狀態 | 30.063 秒收到 301 筆；時間戳嚴格增加，localization_valid 全為 false |
| 規劃器降級 | state 全為 2；reason 全為 `Local WGS84 Home unavailable` |
| 無飛控 state／attitude | 各有 1 個 publisher，實際訊息數為 0；符合 odometry 前置條件 |
| 控制命令觀測 | 兩次有效觀測窗內 `/uav_1/fmu/in/vehicle_command` 訊息數皆為 0 |
| 重複 start | exit 0，容器 ID 相同，未建立額外 Agent |
| 正常 stop | 容器 exited、exit 0、error 空白；UDP 8888 不再監聽 |
| 再次 start | exit 0，恢復 healthy |
| 啟動後再次 DDS 觀測 | 30.058 秒再收 301 筆；相同降級狀態，觀測 exit 0 |
| 最終健康與日誌 | running、healthy、RestartCount 0；重啟後日誌未見 ERROR／FATAL／Traceback／程序崩潰匹配 |

### state／attitude 的驗收說明

首次觀測曾以「未接飛控仍發布 state／attitude」作為預期，實際 30 秒收到 0 筆，因此初版 probe 未通過。
隨後唯讀核對 `v1.3.0-rc.3` 的 `BasicControllerNode::state_publish_cb()`：
`has_odometry_data_` 為 false 時直接 return。未接飛控尚無 odometry，故不能用這兩個 topic 判定 Agent 離線。
依 release 實作補做 planner 狀態觀測，確認 DDS 實際資料傳輸與定位未就緒行為。
沒有為了讓測試通過而修改生產程式碼或注入假飛控資料。

Probe 中對 state 訊息計算的 online／armed／GPS 等彙總，在訊息數為 0 時不適用，不能解讀為實際飛控狀態。

### 資源快照

- 運行初期 CPU 約 1.65–1.66%，記憶體約 51.4–53.25 MiB。
- 最後三次相隔約 11 秒的樣本：CPU 1.67%、1.62%、18.03%；記憶體 53.06、53.07、53.12 MiB，46 個 PID。
- status 查詢後另有 CPU 35.88%、記憶體 58.23 MiB 的短暫樣本；這些快照不代表長期平均。
- 最終量測溫度約 68.3°C。vcgencmd 因 `/dev/vcio` 不存在，未能確認 throttling。

## 保留位置與操作

樹莓派：`/home/taiphoon/Taiphoon_Agent_release_test/v1.3.0-rc.3/`。
內含 `release.tar.gz`、`package/`、`config/`、`no-fc-observer.py` 與各階段 `.log`。

```bash
cd /home/taiphoon/Taiphoon_Agent_release_test/v1.3.0-rc.3/package
export TAIPHOON_AGENT_HOME=/home/taiphoon/Taiphoon_Agent_release_test/v1.3.0-rc.3/config
./taiphoon_agent status
./taiphoon_agent logs
# 需要停止／再次啟動時：
./taiphoon_agent stop
./taiphoon_agent start
```

依使用者補充授權，移除已停止的舊版 `fmu1_agent`（1.2.1）容器。
舊版映像與 bind mount 原始檔保留；三個測試觀測容器均已退出，新版 Agent 為唯一運行中的測試容器。
既有 hello-world 容器未改動，未新增開機自啟服務，未修改網路或安全設定，未提交 Git。

## 後續設定：開機自動運行（2026-10-05 09:36，臺北時間）

使用者已由 GCS 確認 Agent 功能正常，並授權將樹莓派設定為開機後自動運行。
新增 `/etc/systemd/system/taiphoon-agent.service`，沿用上述 release 與獨立設定目錄。
此節更新前述「未新增開機自啟服務」的初次測試狀態。

- systemd 使用 `User=taiphoon`；`Requires=docker.service`，並在 Docker 與 network-online.target 之後啟動。
- NetworkManager-wait-online.service 與 docker.service 原已 enabled，沒有修改網路設定。
- ExecStart 執行 `taiphoon_agent start`；ExecStop 執行 `taiphoon_agent stop`。
- Type=oneshot、RemainAfterExit=yes；`active (exited)` 代表啟動工作完成，Agent 持續在 Docker 中運行。
- 啟動工作失敗時每 10 秒重試，300 秒內最多 6 次；這不是容器執行期間崩潰的持續重啟監控。
- `systemd-analyze verify` 通過，`systemctl enable --now` 成功。
- `is-enabled` 回報 enabled，`is-active` 回報 active，ExecStart exit 0/SUCCESS。
- 原容器仍 running／healthy，RestartCount 0，StartedAt 維持 `2026-10-05T01:29:58.032168547Z`，未中斷既有 Agent。
- 未重開樹莓派；已驗證服務設定與即時啟動，尚未執行真正 reboot 驗收。

後續建議使用 systemd 管理啟停，避免與 launcher 手動啟停狀態不同步：

```bash
sudo systemctl status taiphoon-agent.service --no-pager
sudo journalctl -u taiphoon-agent.service -b --no-pager
sudo systemctl stop taiphoon-agent.service
sudo systemctl start taiphoon-agent.service
sudo systemctl restart taiphoon-agent.service
```

服務固定指向目前的 release 路徑；升級時須同步調整服務的 WorkingDirectory、Environment、ExecStart 與 ExecStop。
