# Taiphoon Agent 開發與驗證指南

給開發者與維護人員的補充說明。下載、安裝與日常操作見 [README](../README.md)；README 只放使用者需要的內容，版本狀態、網路細節與驗證流程集中在本文件。容器內建置與測試指令見 [AGENTS.md](../AGENTS.md)。

## 版本與發布

- 目前版本：`v1.3.0-rc.9` 預發布版，詳見 [版本說明](releases/v1.3.0-rc.9.md)；各版本說明在 [docs/releases/](releases/)。
- 提交／PR 自動驗證 Ubuntu AMD64／ARM64 與 Windows／Mac 編譯，版本標籤通過後才發布五個完整平台包。流程見 [CI/CD 與版本發布](ci-cd.md)。
- 使用者以 GitHub Releases 取得版本資訊；README 不寫目前版本、驗證計畫或未完成項目。

## 編隊與任務生命週期

README 只保留使用者需要知道的行為；以下為完整規則與驗證背景。

- 編隊與同儕避碰的預設機體中心最小距離為 **2 m**，規劃額外裕度維持 1 m。這些參數不會自動限制場域邊界、飛行高度或 PX4 RTL 路徑；小場域三機驗證與測試取捨詳見 [驗證計畫](codex/small-field-three-agent-validation-plan.md)。
- 確認落地或解除解鎖後，Agent 會清除編隊、移動目標與舊本機軌跡，停止任務規劃。地面移動／編隊命令不會排隊到起飛後執行；再次起飛後須重新下達編隊命令。已降落的機體不再發布佔位，其他 Agent 依有效落地狀態移除其舊軌跡與編隊成員資格。降落生命週期回歸與 Docker 測試方式詳見 [驗證報告](codex/landing-mission-lifecycle-report.md)。
- 編隊高度一律以 Home 為基準，先移動再下達編隊也不會疊加目前飛行高度。Agent 關閉後，存活成員會自動重組；飛行中重新啟動的 Agent 需重新下達 `FORMATION_*` 才會加入，期間其他成員維持目前編隊。規則詳見 [介面文件](taiphoon-agent-interface.md) 3.3 節。

## 單元測試選項

預設單元測試只建置現行控制鏈與必要回歸；舊 FormationCalculator、waypoint planner 與觀測式防撞測試可由 `-DTAIPHOON_BUILD_LEGACY_TESTS=ON` 啟用，原始測試檔仍保留。

## 小場域三機驗證（Windows）

Windows 開發環境可用 PowerShell 執行小場域驗證（需 Docker Desktop、原生 Compose、Git Bash、既有 dev 映像與可寫入的開發建置 volumes；Git Bash 僅調度 Docker，ROS2 在容器內執行）：

```powershell
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 1
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 2
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 3
./AutoTest/scripts/small_field_three_agent_validation.ps1 -Scenario crossing
```

測試使用三機、2 m 起飛高度、2 m 圓半徑與嚴格 2 m 中心距離驗收。編隊情境只執行前五步；兩機重編隊須持續收斂 15 秒，不下達 RTL。交叉情境讓兩端飛機互換位置，中間飛機保持原位。

記錄輸出於 `AutoTest/Log/small-field-日期/`；即時 monitor 成功後，仍須在 Docker 中以 `analyze_small_field_px4_logs.py`（需要 `pyulog`）檢查包含失效機的三台 PX4 ULog。離線查核使用共同 UTC 時鐘與 20 Hz 取樣，不等同實體碰撞模擬或實機零碰撞保證。

其他 SITL 自動驗收腳本見 [AutoTest/scripts/](../AutoTest/scripts/)。

## 啟動器的 dev 模式

`taiphoon_agent config` 選 dev 後，填入原始碼目錄並選擇 sim 或 real；啟動前先在容器內編譯。sim 可選 1～4 組，統一透過 `Taiphoon_Net` 通訊。多組模擬（sim 或 dev → sim）由啟動器依「先 Agent 後 PX4」的順序啟動。

多機對時工具 `setup_time_sync.sh` 隨 Linux 平台包發布於根目錄，原始碼在 `Tools/time_sync/`。

```text
taiphoon_agent build    dev：容器內編譯
taiphoon_agent test     dev：先編譯，再執行測試
taiphoon_agent shell    dev：進入開發容器
```

- 原始碼只在 dev 掛載；build／install／log 用 volume 隔離 CPU 架構。所有 ROS2 指令與測試在容器內執行。
- 既有 `docker-compose.yml` 與 AutoTest 腳本保留作為進階開發流程；一般操作使用原生入口。

## 容器退出清理

原生啟動器啟動時會在主機註冊容器退出清理，Agent 遠端關閉或自行退出後便移除容器；`stop`／`restart` 也會移除舊容器，保留命名 volumes 與設定。Windows 清理程序不開啟視窗。此行為需要重新編譯的啟動器（v1.3.0-rc.4 起的 Release 已包含），更早的 Release 執行檔不會自動更新；直接執行 `docker compose up` 不會註冊此清理程序。容器移除後無法使用 `docker logs` 讀取舊 stdout，需要保留的記錄應先匯出；小場域驗證腳本會在移除前自動儲存 Agent 日誌。

## 網路與 DDS 細節

### 實機

實機預設飛控 IP 為 **192.168.1.14**；這不是 Agent 主機 IP。工具每次啟動自動取得 Agent 主機 IPv4，輸出飛控所需 UDP 埠與 namespace。無人機 N 使用 `8887 + N` 與 `/uav_N`。

Linux 實機使用 host network；Desktop 自動映射 UDP，不需要開啟全域 host networking。Wi-Fi GCS 使用相同 ROS domain（預設 0）與介面；若網路阻擋 discovery，填入 GCS IPv4。

### 模擬網路

模擬模式（sim 或 dev → sim）一律接入外部 Docker 網路 `Taiphoon_Net`。首次使用前，若網路不存在，執行 `docker network create Taiphoon_Net`。每個模擬 Agent 獨立接入網路，PX4 也接入同一網路並連向對應 Agent，Agent 之間以 multicast 互相發現。Linux Docker Engine 的 GCS 接入 `Taiphoon_Net` 並使用相同 ROS domain（預設 0）即可通訊，不映射主機 DDS 埠。

網路不存在時啟動會失敗；stop 不會移除此外部網路。不要混用相同 UAV 編號與 domain 的模擬及實機。ARM64 的 PX4 SITL 使用 AMD64 模擬，Docker 須具備相應 CPU 模擬支援；樹莓派實機 Agent 為原生 ARM64。首次模擬會自動下載 PX4 映像，需要網路；Agent 映像已包含在平台包內。

### Docker Desktop 模擬與原生 Taiphoon QGCS

Docker Desktop（Windows／macOS）的容器位於 NAT 之後，主機收不到容器 multicast，因此 `taiphoon_agent` 在 Desktop 模擬時一律產生供同機原生 Taiphoon QGCS 連線的單播設定，不需額外選項：

- 啟動時自動取得主機區網 IPv4（`lan_address=auto`）；多網卡選錯或離線時，在 `settings.ini` 指定 `lan_address`。
- 每台 Agent 使用各自的 `cyclonedds_N.xml`：公告主機 IPv4、專屬單播埠段，並向 `192.168.65.254`（host.docker.internal）的 QGCS 埠（domain 0 為 7510）送出 discovery。
- 只映射該 Agent 的埠段並綁定主機 IPv4：domain 0 時 agent_1 7410-7433、agent_2 7434-7457、agent_3 7458-7481、agent_4 7482-7505；domain 每加 1，所有埠號加 250。
- 啟動成功後印出 QGCS 參數，例如：`Taiphoon_QGCS.exe --interface <主機IP> --external-address 192.168.65.254 --no-multicast --peer <主機IP> --participant-index 50`
- QGCS 在**其他電腦**時，`config` 選 sim 後填入該電腦 IPv4（儲存於 `dds_peers`，多個以逗號分隔，輸入 `-` 清除）。每台 Agent 會再主動向 `<GCS IP>:7510`（domain 0）送出 discovery，啟動後另印出該電腦的參數：`Taiphoon_QGCS.exe --interface <GCS IP> --no-multicast --peer <主機IP> --participant-index 50`。QGCS 必須固定 participant index 50，且該電腦需允許 UDP 7510／7511 入站。

主機 IP 改變（例如更換 Wi-Fi）後執行 `taiphoon_agent restart`。Linux Docker Engine 模擬不使用 `dds_peers`；GCS IPv4 不可填 `127.x`。

### 同儕轉送（dds_peer_relay）

所有 Agent 公告同一個主機 IP 時，CycloneDDS 會把同儕位址改寫成自己的容器 IP，Agent 之間便互相看不到、各自組成單機編隊。因此此模式下每台 Agent 另啟動 `dds_peer_relay`，在本機容器 IP 綁定其他 Agent 的埠段，把同儕封包轉給 `Taiphoon_Net` 上的對應 Agent 容器（無法解析時退回主機埠映射）；不需要特權，實機與 Linux Docker Engine 不受影響。此模式可用於 GCS 連線與編隊功能測試，不作為編隊時序驗證。套用此修正需要重新編譯的 Agent（開發 compose 會自動編譯；Release 映像需重新建置）。

## 本機原生 QGCS 測試（開發 compose）

Windows Docker Desktop 上，主機的原生 Taiphoon QGCS 收不到容器 multicast，需改用單播疊加檔 `docker-compose.local-qgcs.yml`：每台 Agent 公告主機區網 IPv4、使用各自的 CycloneDDS 埠段（uav_1 7410-7433、uav_2 7434-7457、uav_3 7458-7481、uav_4 7482-7505）並只映射該埠段。QGCS 固定使用 participant index 50（7510/7511），Agent 經 `host.docker.internal` 回送，不需修改 Windows 防火牆。

```powershell
./AutoTest/scripts/start_local_qgcs_agents.ps1 -Count 3
```

腳本會自動取得主機 IP、啟動 Agent，並印出 PX4 SITL 與 QGCS 啟動指令，例如：

```powershell
Taiphoon_QGCS.exe --interface <主機IP> --external-address 192.168.65.254 --no-multicast --peer <主機IP> --participant-index 50
```

Windows 沒有可掛進 Linux 容器的 SSH agent socket，`docker-compose.yml` 的 `SSH_AUTH_SOCK` 掛載會讓 Agent 容器無法建立，因此腳本另外套用 `AutoTest/scripts/compose.no-ssh-agent.yml`，只掛載專案目錄（需 Docker Compose 2.24.4 以上）。在 Windows 手動執行 `docker compose up` 時（包含預設的多機啟動）也要加上 `-f AutoTest/scripts/compose.no-ssh-agent.yml`。

此模式的 Agent 之間單播經 `dds_peer_relay` 轉送（見上節），可用於 GCS 連線與編隊功能測試，不作為編隊時序驗證；預設 `docker compose up -d` 與實機部署不受影響。同一時間只能有一組使用相同 UAV 編號的模擬環境。

## 維護文件

- [平台套件與進階設定](../Tools/agent_launcher/PACKAGE-README.md)
- [CI/CD 與版本發布](ci-cd.md)
- [各版本 Release 說明](releases/)
- [部署設計](deployment-cross-platform.md)／[驗證紀錄](deployment-verification.md)
- [ROS2 Node／Topic 介面](taiphoon-agent-interface.md)
- [飛行命令](agent-commands.md)／[ROS2 參數](agent-configuration.md)
- [多機時間同步](time-sync.md)：設計、失效情境與驗證
- [實機障礙物地圖來源](obstacle-source.md)：設計、失效情境與舊版（rc.8）暫時做法
- [SITL 自動驗收腳本](../AutoTest/scripts/)

## README 撰寫規則

- README 是給使用者的產品頁，版面與 Taiphoon QGCS 的 README 一致；不放版本狀態、開發計畫、驗證流程與網路技術細節。
- 地面站一律寫 Taiphoon QGCS；QGCS 只出現在連線說明與機群飛行畫面，不介紹 QGCS 的操作介面。
- 畫面素材放在 `docs/images/`，一律取自模擬環境（起飛點大佳河濱公園：`launch_all.sh --home-lat 25.0756 --home-lon 121.5375`），不可出現公網 IP、帳號或個人路徑。
