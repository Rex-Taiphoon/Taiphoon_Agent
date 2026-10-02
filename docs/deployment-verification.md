# 部署驗證紀錄

分支：`codex/cross-platform-agent-launcher`。角色：Docker／部署維護，含測試與文件責任。
本輪目標：Ubuntu 22.04 ARM64／AMD64、Windows、Mac；零參數啟動、單／四機、Wi-Fi 與精簡文件。
飛控／規劃／安全邏輯未修改，既有 `src/nodes/ego_swarm_planner_node.cpp` 工作區變更不納入發布。

## 本輪本機結果

| 驗證 | 結果與限制 |
|---|---|
| Docker Desktop | Linux amd64 Engine 29.8.1；工具查標準 CLI 路徑，不依賴目前程序的舊 PATH |
| workspace | 既有 Release dev 映像內建置成功；沒有在 host 執行 ROS2 或 Docker build |
| 原生工具 | Linux C++ 與 Windows MSVC 編譯成功 |
| 三組 CTest | 全部通過：設定／Compose／argv、CLI 生命週期、五個平台打包 |
| Windows 真正 exe：1 組 local | PX4＋Agent 健康；Docker 內收到至少 3 筆有效、未解鎖、地面且時間前進的 state |
| Windows 真正 exe：4 組 local | 四組全部健康，每台均通過唯讀狀態檢查；共用網路命名空間，8888～8891 分流 |
| Windows 真正 exe：4 組 wifi | 自動公告本機 IPv4、映射 DDS 7410～7491；四台都收到有效狀態；尚未驗收外部 GCS |
| Windows 真正 exe：單機 deploy | ID 7、飛控預設 192.168.1.14、UDP 8894；僅一個 Agent 健康，沒有模擬器；實體飛控尚未連接 |
| 模式切換／停止 | stop → config → start 成功；正常停止測試服務，保留資料 |
| 長 manifest | 線性解析 1000 個 layer digest；修正 Windows regex 堆疊耗盡，容許無關 Config 差異 |
| 文件 | README 約 90 行；命令／ROS2 參數各有獨立參考，部署與 CI 操作集中整理 |

實際 Windows 啟動檢查使用本輪新編譯的測試版 launcher 與已發布的 rc.1 Agent 映像，
只新增測試標籤與隔離設定，不替換既有映像。本輪 ROS2 本體沒有改動；新 Release 映像另由 CI 編譯驗證。
全部 ROS2 觀測都在 Docker dev 容器執行；沒有 Arm、Takeoff 或飛行任務。

## 跨平台 CI／Release

`agent-build.yml` 會建置 Ubuntu 22.04 AMD64／ARM64、Windows x64、macOS Intel＋Apple Silicon，
Linux 各執行套件測試與三組 CTest；AMD64 額外執行真正 CLI 的 1／4 組 PX4 模擬與停止。
新版本預定 `v1.3.0-rc.2`，本段在實際完成後填入 run 與 Release 連結，不能以本機成功代替。

歷史版本：[rc.1 Release](https://github.com/Rex-Taiphoon/Taiphoon_Agent/releases/tag/v1.3.0-rc.1)
與[建置 run](https://github.com/Rex-Taiphoon/Taiphoon_Agent/actions/runs/37006614126) 全數完成，
雙架構各 328 個 ROS2 測試、三組 CTest 通過；五個完整平台包已發布。
Docker containerd 會為同一 archive 回傳不同 ID；相同檔案層應正常使用，不做完整 Config 一致性 Gate。

## 尚待目標機驗收

- 樹莓派實體飛控／UART、Wi-Fi、感測器與效能；ARM64 SITL 需 Docker AMD64 CPU 模擬支援。
- macOS 新 launcher 的下載開啟、Desktop／Wi-Fi 與 GCS；編譯不等同硬體驗收，套件尚未 Apple 公證。
- 外部 GCS／防火牆／多網卡與飛控 firmware 的訊息相容性；healthy 不代表可以起飛。

較早的 GCS 運行測試另見 [flight-gcs-check](deployment-flight-gcs-check.md)，只屬該次環境的證據。
日常操作見 [README](../README.md)；維護與發布見 [ci-cd.md](ci-cd.md)。
