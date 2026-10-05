# Taiphoon Agent

目前版本：`v1.3.0` 正式版，詳見 [版本說明](docs/releases/v1.3.0.md)。

以 ROS2 Humble 整合飛控、任務規劃、感測器與影像。每個 Agent 只控制自己對應的飛控；
日常啟動統一使用 `taiphoon_agent`，不需要附加參數，不會自動 Arm 或起飛。

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

## 一次設定，後續一鍵

首次執行會顯示設定；之後沿用保存內容。切換情境用 `stop` → `config` → `start`。

| 情境 | 首次選擇 | 啟動內容 |
|---|---|---|
| 樹莓派單機實機 | deploy → 無人機編號 → 選填 GCS IP | 一個 Agent，Wi-Fi／UDP 飛控 |
| 單組本機模擬 | sim → 1 → local | 一個 PX4＋一個 Agent |
| 多組本機模擬 | sim → 2～4 → local | 對應數量 PX4＋Agent，先 Agent 後 PX4 |
| 模擬與 Wi-Fi GCS | sim → 1～4 → wifi → 選填 GCS IP | 本機 PX4／Agent，DDS 連向 GCS |
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
本機模擬只連接模擬飛控；要讓主機或其他電腦的 GCS 存取，選 wifi。不要混用模擬與實機飛控。
ARM64 的 PX4 SITL 使用 AMD64 模擬，Docker 須具備相應 CPU 模擬支援；樹莓派實機 Agent 為原生 ARM64。
首次模擬會自動下載 PX4 映像，需要網路；Agent 映像已包含在平台包內。

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
