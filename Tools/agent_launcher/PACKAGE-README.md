# Taiphoon Agent：下載後啟動

## 選擇平台

- 樹莓派 64 位元 Ubuntu：`linux-arm64`。
- Intel／AMD Ubuntu：`linux-amd64`。
- Windows Intel／AMD：`windows-amd64`。
- macOS Intel：`macos-amd64`；Apple Silicon：`macos-arm64`。

原生工具不需要另外安裝 Python、CMake 或 ROS2。Agent 在 Docker 的 Linux 容器執行。
Windows／macOS 請先安裝 Docker Desktop（Windows 使用 WSL2／Linux 容器），並啟動 Docker。
Ubuntu 請先安裝 Docker Engine、Compose plugin，並設定目前使用者的 Docker 存取權。
每個平台包內含 runtime 與 dev 的預編譯映像；首次啟動會自動載入，不需要再登入或手動校驗。
正常使用本機 Docker daemon；若指定不同架構的 daemon，需下載與 daemon 架構一致的平台包。
下載包與映像包含完整版本配對，請勿混用不同版本的 release.manifest。

## 最簡單的啟動

解壓縮至一般使用者可讀取的資料夾：

- **Windows**：雙擊 `taiphoon_agent.exe`，或在 PowerShell 執行 `./taiphoon_agent.exe start`。
- **macOS**：雙擊 `Start Taiphoon Agent.command`，或在 Terminal 執行 `./taiphoon_agent start`。
- **Ubuntu／樹莓派**：在終端執行 `./taiphoon_agent start`。

首次執行自動顯示互動設定，保存模式後啟動；後續只需要同一個啟動指令。
macOS 套件尚未 Apple notarization；若系統要求確認開發者，使用系統提供的開啟核准流程，
不要關閉 Gatekeeper 或停用系統安全機制。

可選擇以 `bash install.sh`（Ubuntu／macOS）或 `./install.ps1`（Windows）安裝至使用者目錄。
安裝工具會設定使用者 PATH；Ubuntu／macOS 重新開啟終端後可使用 `taiphoon_agent start`。
Windows 若腳本遭 execution policy 阻擋，直接使用 exe，不需要改變安全政策。

## 模式與必要設定

- `deploy`：一個 Agent，輸入 Agent 編號及 UDP／serial 連線。沒有原始碼掛載、不編譯、不啟動模擬感測器。
- `dev`：輸入原始碼目錄；`start` 先在容器編譯，build／install／log 使用依 CPU 架構分開的 volume。
- `sim`：預設四個 Agent 與四個 PX4 SIH；按 Agent 程序健康順序啟動，不自動起飛。

UDP 飛控須自行設定 uXRCE-DDS client，指向 Agent 主機 IPv4，埠為 `8887 + Agent 編號`，
system ID 與 namespace `/uav_{N}` 需對應 Agent 編號；預設第一台為 UDP 8888。
本工具不會更改飛控參數，也不會發布控制命令給其他 Agent 的飛控。
serial 部署目前支援 Ubuntu／樹莓派的 `/dev/...` 裝置；Docker Desktop 請優先使用 UDP。

Docker Desktop 的 deploy／dev 使用 host networking：需 Docker Desktop 4.34+，
在 Settings → Resources → Network 啟用 **Enable host networking**。
連接 LAN 的 DDS 主機時，首次設定填入本機實體 LAN IPv4；必要時填入其他主機的 discovery peer IPv4。
這些設定仍使用 ROS2／DDS，不增加其他資料鏈。Docker Desktop 的網路與實體 Linux 不完全相同，
跨機資料收發需在目標 LAN 驗證。sim 使用隔離的 bridge network，不需要 host networking。
ARM64 上的 PX4 SITL 使用 AMD64 模擬，需要 Docker 的 CPU 模擬支援；樹莓派部署本體是原生 ARM64。

## 日常指令

```text
taiphoon_agent start
taiphoon_agent stop
taiphoon_agent restart
taiphoon_agent status
taiphoon_agent logs
taiphoon_agent config
```

dev 模式另外提供 `build`、`test`、`shell`。所有 ROS2 指令與測試在開發容器執行。
`status` 區分容器健康與 `px4_state_valid`，有效飛控狀態仍不代表定位與障礙感測器已就緒。
`stop` 只停止本工具管理的容器，不刪除原始碼、volume 或使用者資料。
修改模式、工作目錄或版本前先 stop，避免執行中設定與容器不一致。
新工具使用獨立 Compose project `taiphoon_agent`；請勿同時以舊流程啟動相同 Agent／飛控。

本機設定存於 Ubuntu／macOS 的 `~/.config/taiphoon_agent`，或 Windows 的
`%LOCALAPPDATA%\TaiphoonAgent\config`。進階設定在 `settings.ini`：
`domain_id`（0～101）、`lan_address`、`dds_peers`。
`TAIPHOON_AGENT_HOME` 可指定其他絕對目錄；設定檔不會作為 shell 執行。
不要在服務執行中手動更改設定；先 stop 再設定與 start。

## 驗收

1. 啟動後執行 `status`；無飛控時應清楚顯示等待狀態。
2. 連接飛控，確認有有效 PX4 狀態，且 GCS／其他 Agent 可透過既有 ROS2 topic 收發資料。
3. 檢查 logs、stop、再次 start 與重複 start。
4. 飛行驗收另行依專案流程操作；本套件啟動不包含 Arm／起飛。
