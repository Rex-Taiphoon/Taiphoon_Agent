# Taiphoon Agent

目前版本：`v1.3.0-rc.7` 預發布版，詳見 [版本說明](docs/releases/v1.3.0-rc.7.md)。

以 ROS2 Humble 整合飛控、任務規劃、感測器與影像。每個 Agent 只控制自己對應的飛控；
日常啟動統一使用 `taiphoon_agent`，不需要附加參數，不會自動 Arm 或起飛。

編隊與同儕避碰的預設機體中心最小距離為 **2 m**，規劃額外裕度維持 1 m。
這些參數不會自動限制場域邊界、飛行高度或 PX4 RTL 路徑；小場域三機驗證與
測試取捨詳見 [驗證計畫](docs/codex/small-field-three-agent-validation-plan.md)。

預設單元測試只建置現行控制鏈與必要回歸；舊 FormationCalculator、waypoint planner
與觀測式防撞測試可由 `-DTAIPHOON_BUILD_LEGACY_TESTS=ON` 啟用，原始測試檔仍保留。

確認落地或解除解鎖後，Agent 會清除編隊、移動目標與舊本機軌跡，停止任務規劃。
地面移動／編隊命令不會排隊到起飛後執行；再次起飛後須重新下達編隊命令。
已降落的機體不再發布佔位，其他 Agent 依有效落地狀態移除其舊軌跡與編隊成員資格。
降落生命週期回歸與 Docker 測試方式詳見 [驗證報告](docs/codex/landing-mission-lifecycle-report.md)。

編隊高度一律以 Home 為基準，先移動再下達編隊也不會疊加目前飛行高度。Agent 關閉後，
存活成員會自動重組；飛行中重新啟動的 Agent 需重新下達 `FORMATION_*` 才會加入，
期間其他成員維持目前編隊。規則詳見 [介面文件](docs/taiphoon-agent-interface.md) 3.3 節。

Windows 開發環境可用 PowerShell 執行小場域驗證（需 Docker Desktop、原生 Compose、
Git Bash、既有 dev 映像與可寫入的開發建置 volumes；Git Bash 僅調度 Docker，ROS2 在容器內執行）：

```powershell
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 1
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 2
./AutoTest/scripts/small_field_three_agent_validation.ps1 -FailedAgent 3
./AutoTest/scripts/small_field_three_agent_validation.ps1 -Scenario crossing
```

測試使用三機、2 m 起飛高度、2 m 圓半徑與嚴格 2 m 中心距離驗收。
編隊情境只執行前五步；兩機重編隊須持續收斂 15 秒，不下達 RTL。
交叉情境讓兩端飛機互換位置，中間飛機保持原位。
記錄輸出於 `AutoTest/Log/small-field-日期/`；即時 monitor 成功後，仍須在 Docker 中以
`analyze_small_field_px4_logs.py`（需要 `pyulog`）檢查包含失效機的三台 PX4 ULog。
離線查核使用共同 UTC 時鐘與 20 Hz 取樣，不等同實體碰撞模擬或實機零碰撞保證。

## 下載與啟動

先安裝 Docker：Ubuntu 22.04 使用 Docker Engine＋Compose plugin；Windows／macOS
使用 Docker Desktop 的 Linux 容器。從 [GitHub Release](https://github.com/Rex-Taiphoon/Taiphoon_Agent/releases)
下載並解壓完整平台包，內含原生工具與預編譯 Agent 映像，不必自己編譯或手動校驗。

| 平台 | 套件後綴 | 最簡單啟動 |
|---|---|---|
| 樹莓派／Ubuntu 22.04 ARM64 | linux-arm64.tar.gz | `./taiphoon_agent` |
| Ubuntu 22.04 Intel／AMD | linux-amd64.tar.gz | `./taiphoon_agent` |
| Windows Intel／AMD | windows-amd64.zip | 雙擊 `taiphoon_agent.exe` |
| macOS Apple Silicon | macos-arm64.tar.gz | 雙擊 `Start Taiphoon Agent.command` |
| macOS Intel | macos-amd64.tar.gz | 雙擊 `Start Taiphoon Agent.command` |

Windows PowerShell 可執行 `./taiphoon_agent.exe`，macOS Terminal 可執行 `./taiphoon_agent`。
選用 `install.ps1` 或 `bash install.sh` 安裝後，各平台可直接使用 `taiphoon_agent`。
macOS 首次開啟依系統提供的核准流程操作；目前套件未公證。

原生啟動器啟動時會在主機註冊容器退出清理，Agent 遠端關閉或自行退出後便移除容器；
`stop`／`restart` 也會移除舊容器，保留命名 volumes 與設定。Windows 清理程序不開啟視窗。
此行為需要重新編譯的啟動器，既有 Release 執行檔不會自動更新；直接執行
`docker compose up` 不會註冊此清理程序。容器移除後無法使用 `docker logs` 讀取舊 stdout，
需要保留的記錄應先匯出；上述驗證腳本會在移除前自動保存 Agent 日誌。

## 一次設定，後續一鍵

首次執行會顯示設定；之後沿用保存內容。切換情境用 `stop` → `config` → `start`。

| 情境 | 首次選擇 | 啟動內容 |
|---|---|---|
| 樹莓派單機實機 | deploy → 無人機編號 → 選填 GCS IP | 一個 Agent，Wi-Fi／UDP 飛控 |
| 單組本機模擬 | sim → 1 | 一個 PX4＋一個 Agent |
| 多組本機模擬 | sim → 2～4 | 對應數量 PX4＋Agent，先 Agent 後 PX4 |
| 模擬與 QGCS | sim → 1～4 → 選填其他電腦 QGCS IP | Docker Desktop 上由同機或其他電腦的原生 Taiphoon_QGCS 以單播連線 |
| 開發 | dev → 原始碼目錄 → sim 或 real | 容器內編譯；sim 可選 1～4 組 |

實機預設飛控 IP 為 **192.168.1.14**；這不是 Agent 主機 IP。工具每次啟動自動取得
Agent 主機 IPv4，輸出飛控所需 UDP 埠與 namespace。無人機 N 使用 `8887 + N` 與 `/uav_N`。
PX4 的 uXRCE-DDS client 必須事先指向 Agent 主機，例如無人機 1：

```sh
uxrce_dds_client start -t udp -h <Agent主機IPv4> -p 8888 -n uav_1
```

以上指令在飛控 console 設定；system ID 須與無人機編號一致。啟動工具不改寫飛控參數。
Linux 實機使用 host network；Desktop 自動映射 UDP，不需要開啟全域 host networking。
Wi-Fi GCS 使用相同 ROS domain（預設 0）與介面；若網路阻擋 discovery，填入 GCS IPv4。
模擬模式（sim 或 dev → sim）一律接入外部 Docker 網路 `Taiphoon_Net`。
首次使用前，若網路不存在，執行 `docker network create Taiphoon_Net`。
每個模擬 Agent 獨立接入網路，PX4 也接入同一網路並連向對應 Agent，Agent 之間以
multicast 互相發現。Linux Docker Engine 的 GCS 接入 `Taiphoon_Net` 並使用相同 ROS domain
（預設 0）即可通訊，不映射主機 DDS 埠。

Docker Desktop（Windows／macOS）的容器位於 NAT 之後，主機收不到容器 multicast，因此
`taiphoon_agent` 在 Desktop 模擬時一律產生供同機原生 Taiphoon_QGCS 連線的單播設定，不需額外選項：

- 啟動時自動取得主機區網 IPv4（`lan_address=auto`）；多網卡選錯或離線時，在 `settings.ini` 指定 `lan_address`。
- 每台 Agent 使用各自的 `cyclonedds_N.xml`：公告主機 IPv4、專屬單播埠段，並向
  `192.168.65.254`（host.docker.internal）的 QGCS 埠（domain 0 為 7510）送出 discovery。
- 只映射該 Agent 的埠段並綁定主機 IPv4：domain 0 時 agent_1 7410-7433、agent_2 7434-7457、
  agent_3 7458-7481、agent_4 7482-7505；domain 每加 1，所有埠號加 250。
- 啟動成功後印出 QGCS 參數，例如：
  `Taiphoon_QGCS.exe --interface <主機IP> --external-address 192.168.65.254 --no-multicast --peer <主機IP> --participant-index 50`
- QGCS 在**其他電腦**時，`config` 選 sim 後填入該電腦 IPv4（保存於 `dds_peers`，多個以逗號分隔，
  輸入 `-` 清除）。每台 Agent 會再主動向 `<GCS IP>:7510`（domain 0）送出 discovery，啟動後另印出該電腦的參數：
  `Taiphoon_QGCS.exe --interface <GCS IP> --no-multicast --peer <主機IP> --participant-index 50`。
  QGCS 必須固定 participant index 50，且該電腦需允許 UDP 7510／7511 入站。

所有 Agent 公告同一個主機 IP 時，CycloneDDS 會把同儕位址改寫成自己的容器 IP，Agent 之間便互相看不到、
各自組成單機編隊。因此此模式下每台 Agent 另啟動 `dds_peer_relay`，在本機容器 IP 綁定其他 Agent 的
埠段，把同儕封包轉給 `Taiphoon_Net` 上的對應 Agent 容器（無法解析時退回主機埠映射）；不需要特權，
實機與 Linux Docker Engine 不受影響。此模式可用於 GCS 連線與編隊功能測試，不作為編隊時序驗證。
套用此修正需要重新編譯的 Agent（開發 compose 會自動編譯；Release 映像需重新建置）。
主機 IP 改變（例如更換 Wi-Fi）後執行 `taiphoon_agent restart`。Linux Docker Engine 模擬不使用 `dds_peers`；
GCS IPv4 不可填 `127.x`。
網路不存在時啟動會失敗；stop 不會移除此外部網路。不要混用相同 UAV 編號與 domain 的模擬及實機。
ARM64 的 PX4 SITL 使用 AMD64 模擬，Docker 須具備相應 CPU 模擬支援；樹莓派實機 Agent 為原生 ARM64。
首次模擬會自動下載 PX4 映像，需要網路；Agent 映像已包含在平台包內。

## 本機原生 QGCS 測試（開發 compose）

Windows Docker Desktop 上，主機的原生 Taiphoon_QGCS 收不到容器 multicast，需改用單播疊加檔
`docker-compose.local-qgcs.yml`：每台 Agent 公告主機區網 IPv4、使用各自的 CycloneDDS 埠段
（uav_1 7410-7433、uav_2 7434-7457、uav_3 7458-7481、uav_4 7482-7505）並只映射該埠段。
QGCS 固定使用 participant index 50（7510/7511），Agent 經 `host.docker.internal` 回送，
不需修改 Windows 防火牆。

```powershell
./AutoTest/scripts/start_local_qgcs_agents.ps1 -Count 3
```

腳本會自動取得主機 IP、啟動 Agent，並印出 PX4 SITL 與 QGCS 啟動指令，例如：

```powershell
Taiphoon_QGCS.exe --interface <主機IP> --external-address 192.168.65.254 --no-multicast --peer <主機IP> --participant-index 50
```

此模式的 Agent 之間單播經 `dds_peer_relay` 轉送（見上節），可用於 GCS 連線與編隊功能測試，不作為編隊時序驗證；
預設 `docker compose up -d` 與實機部署不受影響。同一時間只能有一組使用相同 UAV 編號的模擬環境。

## 常用指令

```text
taiphoon_agent          使用保存設定啟動；等同 start
taiphoon_agent stop     停止自有服務，保留資料
taiphoon_agent restart  重新套用目前網路與設定
taiphoon_agent status   容器健康與飛控連線狀態
taiphoon_agent logs     最近的日誌
taiphoon_agent config   停止後切換設定
taiphoon_agent build    dev：容器內編譯
taiphoon_agent test     dev：先編譯，再執行測試
taiphoon_agent shell    dev：進入開發容器
```

直接使用解壓工具時，Linux／Mac 在指令前加 `./`，Windows 使用 `./taiphoon_agent.exe`。
原始碼只在 dev 掛載；build／install／log 用 volume 隔離 CPU 架構。所有 ROS2 指令與測試在容器內執行。
更換 Wi-Fi 後使用 `restart`；飛控尚未連線不會阻止 Agent 啟動。`healthy` 不代表可以起飛。
Windows 若找不到 Docker，先開啟 Desktop；工具會查 PATH 與標準安裝目錄。

## 維護文件

- [平台套件與進階設定](Tools/agent_launcher/PACKAGE-README.md)
- [CI/CD 與版本發布](docs/ci-cd.md)
- [各版本 Release 說明](docs/releases/)
- [部署設計](docs/deployment-cross-platform.md)／[驗證紀錄](docs/deployment-verification.md)
- [ROS2 Node／Topic 介面](docs/taiphoon-agent-interface.md)
- [飛行命令](docs/agent-commands.md)／[ROS2 參數](docs/agent-configuration.md)
- [SITL 自動驗收腳本](AutoTest/scripts/)

既有 `docker-compose.yml` 與 AutoTest 腳本保留作為進階開發流程；一般操作使用原生入口。
提交／PR 自動驗證 Ubuntu AMD64／ARM64 與 Windows／Mac 編譯，版本標籤通過後才發布五個完整平台包。
