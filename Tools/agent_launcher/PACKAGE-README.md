# Taiphoon Agent 平台套件

## 下載後啟動

先開啟 Docker：Ubuntu 22.04 使用 Engine＋Compose plugin；Windows／macOS 使用 Desktop 的 Linux 容器。
下載包內含原生工具與預編譯映像，首次自動載入，不需要 Python、ROS2 或自行編譯。

| 平台 | 選擇套件 | 開啟方式 |
|---|---|---|
| 樹莓派 64 位元 Ubuntu | linux-arm64 | `./taiphoon_agent` |
| Intel／AMD Ubuntu | linux-amd64 | `./taiphoon_agent` |
| Windows Intel／AMD | windows-amd64 | 雙擊 `taiphoon_agent.exe` |
| Mac Apple Silicon／Intel | macos-arm64／macos-amd64 | 雙擊 `Start Taiphoon Agent.command` |

Windows 指令為 `./taiphoon_agent.exe`；Mac Terminal 為 `./taiphoon_agent`。
`bash install.sh` 或 `./install.ps1` 可選擇安裝至使用者目錄，之後直接使用 `taiphoon_agent`。
首次 macOS 開啟依系統核准流程操作，套件尚未公證；不必停用安全機制。

## 三種模式

- **deploy**：填無人機編號，啟動一個 Agent；Wi-Fi 飛控預設 `192.168.1.14`，GCS IP 選填。
- **sim**：選 1～4 組，預設 1；每組一個 PX4＋Agent。每個 Agent 獨立接入 `Taiphoon_Net`；Windows／macOS Docker Desktop 上可直接由同機安裝的 Taiphoon_QGCS 連線。
- **dev**：填原始碼目錄，選 sim 或 real；先在容器內編譯。模擬同樣可選 1～4 組，統一透過 `Taiphoon_Net` 通訊。

設定保存後，日常只要開啟工具。工具不自動 Arm／起飛，實機模式不啟動模擬感測器。

```text
taiphoon_agent          啟動（等同 start）
taiphoon_agent stop     停止並移除自有容器，保留設定與命名 volumes
taiphoon_agent config   停止後切換模式或機數
taiphoon_agent restart  重新取得目前網路並啟動
taiphoon_agent status   程序健康、PX4 狀態
taiphoon_agent logs     最近日誌
taiphoon_agent build    dev：容器內建置
taiphoon_agent test     dev：建置與測試
taiphoon_agent shell    dev：進入開發容器
```

未安裝至 PATH 時加上 `./`，Windows 改用 `./taiphoon_agent.exe`。

## 實機與 Wi-Fi

`192.168.1.14` 是飛控位址；Agent 主機位址會自動取得。無人機 N 對應 UDP `8887 + N`、
PX4 namespace `/uav_N` 與 Agent `/uav_N_agent`。飛控 system ID 必須相同，uXRCE-DDS client
需先設定連向 Agent 主機；例如無人機 1 在飛控 console 執行：

```sh
uxrce_dds_client start -t udp -h <Agent主機IPv4> -p 8888 -n uav_1
```

Desktop 自動映射飛控 UDP 與 DDS 埠；Linux 實機使用 host network。
GCS 使用相同 ROS domain（預設 0）；填入 GCS IPv4 可使用 unicast discovery。防火牆／Wi-Fi
須允許 UDP；domain 0 的 Desktop DDS 範圍為 7410～7491。更換 Wi-Fi 後執行 restart。
模擬模式（包含 dev → sim）需要外部 Docker 網路 `Taiphoon_Net`；若不存在，先執行
`docker network create Taiphoon_Net`，否則啟動會失敗。每個 Agent 獨立接入網路，
PX4 也接入同一網路並連向對應 Agent。stop 不會移除此網路。舊設定中的 wifi 與 `dds_peers` 在模擬模式不生效。
不要讓實體飛控連到模擬模式的 Agent，也不要混用相同 UAV 編號與 domain 的模擬及實機。
ARM64 SITL 需要 Docker 的 AMD64 CPU 模擬支援；樹莓派實機 Agent 為原生 ARM64。
首次模擬會自動下載 PX4 映像，需要網路；Agent 映像已包含在平台包內。

## 模擬與本機 Taiphoon_QGCS

Windows／macOS Docker Desktop 的模擬模式會自動取得主機區網 IPv4，讓每個 Agent 公告此位址並使用
專屬 DDS 單播埠段（domain 0：Agent 1～4 依序為 7410-7433、7434-7457、7458-7481、7482-7505），
只映射該段到主機。啟動成功後畫面會列出 QGCS 參數，例如：

```text
Taiphoon_QGCS.exe --interface <主機IP> --external-address 192.168.65.254 --no-multicast --peer <主機IP> --participant-index 50
```

主機有多張網卡或取錯位址時，於 `settings.ini` 設定 `lan_address=<主機IPv4>`；更換網路後執行 restart。
此模式的 Agent 間資料經主機埠轉送，只用於 GCS 連線測試，不作為編隊時序驗證。
Linux Docker Engine 的模擬維持原設定：GCS 接入 `Taiphoon_Net` 並使用相同 ROS domain，不映射主機 DDS 埠。

## 設定與排查

設定在 Windows `%LOCALAPPDATA%\TaiphoonAgent\config`，Linux／Mac `~/.config/taiphoon_agent`。
舊設定自動補預設值；Docker 或飛控暫時離線不會清除設定。容器 healthy 不代表可以起飛。
進階 `settings.ini` 可設定 `domain_id`、`flight_ip`、`dds_peers`；`lan_address=auto` 每次啟動重新取得
位址，必要時可指定 IPv4。Linux serial 可設定 `transport=serial`、`serial_device`、`serial_baud`。
`TAIPHOON_AGENT_HOME` 僅供使用者隔離不同設定目錄；一般操作不需要環境參數。

找不到 Docker 時先安裝／開啟 Desktop 或 Engine；Windows／Mac 工具會自動查標準 CLI 位置。
Windows exe 無法直接使用僅安裝在 WSL Ubuntu 內的 CLI；請使用 Docker Desktop 的 Windows CLI 與 Linux Engine。
啟動失敗先看 logs；不同版本的工具、manifest 與映像請一起下載，不要單獨混用。
