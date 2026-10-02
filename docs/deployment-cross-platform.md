# 跨平台部署與 Release 計畫

## 介面與範圍

- 主角色：Docker／部署維護人員；協作責任：系統架構、測試與文件審查。
- 原生 C++ 啟動工具統一命名為 `taiphoon_agent`；Windows 為 `taiphoon_agent.exe`。
- Agent 與 ROS2 仍在 Ubuntu 22.04／Humble 容器執行，維持既有 Node、Topic、訊息與飛行邏輯。
- 原生工具支援 Ubuntu AMD64／ARM64、Windows AMD64、macOS universal（Intel／Apple Silicon）。
- Release 僅附五個自包含平台下載包，各包內含原生工具及相應架構的 runtime、dev Docker 映像。
  macOS 原生工具為 universal binary；包內映像則分 ARM64／AMD64，避免下載不必要架構。
- 首次 `init` 保存模式與必要參數；日常使用 `start`、`stop`、`status`、`logs`。
- 開發提供 `build`、`test`、`shell`；ROS2 建置與測試只在開發容器執行。

## 資料流

`taiphoon_agent → 本機設定／Release manifest → Docker Compose → MicroXRCEAgent + 既有 ROS2 launch`

發布工具依 Docker daemon 架構選擇包內映像，校驗 SHA256 後 `docker load`。
儲存庫為私有，因此使用者只在 GitHub 下載平台包時登入，不另要求 CLI token 或 gh 登入。
本機設定不執行 shell，使用者輸入透過程序 argv 傳遞；不混入命令文字。
部署模式不掛載原始碼、不在啟動時編譯、不啟動模擬感測資料，也不自動 Arm／起飛。
開發模式掛載原始碼，建置與安裝產物使用容器 volume，隔離不同 CPU 架構。
模擬模式使用獨立 Compose 專案與網路；先 Agent 後 PX4，不發送起飛命令。
新入口不修改原本的 `docker-compose.yml` 與開發 entrypoint。

## 失效情境

- Docker 未啟動、Compose 不可用、daemon 架構不支援：啟動前停止並顯示處理方式。
- Release 下載失敗、校驗不符、映像 ID／架構不符：禁止啟動該映像。
- 設定非法、實體設備不存在、工作目錄不存在：不產生不完整服務。
- ROS2 程序／橋接死亡：容器以失敗結束；不靜默宣稱已就緒。
- Docker Desktop 連接實機需啟用 host networking；LAN DDS 另設定公告 IPv4 與 discovery peers。
- SITL 若沒有 ARM64 映像，使用 AMD64 模擬；效能與硬體驗證必須在目標平台確認。
- `status` 區分容器程序健康與飛控連線，不把容器 running 當成可起飛。
- 不刪除使用者資料、volume、原始碼或既有服務；stop 僅停止本工具所屬容器。

## 驗證方式

1. 在既有 Docker 開發映像內先建置 workspace，再執行套件與啟動工具測試。
2. 啟動工具測試：設定驗證、SHA256、Compose 產生、Windows argv 引用、模式安全預設。
3. 在容器內驗證 runtime entrypoint 的語法與程序失敗回傳，不發送飛行命令。
4. 發布 CI 分別建置 AMD64／ARM64，執行 workspace 測試與 runtime 無飛控啟動檢查。
5. Windows／macOS 原生工具由相應 GitHub runner 編譯；目標平台啟動驗證由使用者完成。
6. 全部必要工作通過才建立 prerelease 並上傳五個完整平台下載包；任一建置失敗不發布。
   使用者不需要另外下載或操作校驗檔；校驗資訊只內含於工具包。

## 發布與人工 Gate

CI/CD 使用同一份可重用 workflow：一般分支 push／PR 執行雙架構容器與原生入口驗證；
版本標籤才匯出大型映像、組裝套件並發布。失敗保留診斷日誌，不發布不完整套件。
驗證：檢查 YAML、可重用 workflow 輸入與權限，實際追蹤 GitHub Actions；維護方式記錄於
`docs/ci-cd.md`。此自動化僅操作 CI 容器與產物，不改變 Agent 飛行邏輯。

原工作區的 `src/nodes/ego_swarm_planner_node.cpp` 尚未提交，禁止混入部署提交。
Git commit／push 與 Docker 映像建置遵守根目錄 AGENTS.md：先列出具體內容供開發者確認。
本次開發者已授權部署提交、推送、GitHub Actions 建置與 `v1.3.0-rc.1` 發布；
後續版本仍依專案規約由維護者確認提交與標籤。
不修改 `msg/` 或 ROS2 對外契約，因此不需要同步外部 Taiphoon_Agent_Interfaces。
