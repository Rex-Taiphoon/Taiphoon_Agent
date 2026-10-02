# 跨平台部署設計

主角色：Docker／部署維護人員，負責介面、失效、測試與文件。
本輪已授權自行決定實作、提交／推送與新 Release；飛行邏輯與既有 planner 未提交變更不納入。

## 介面與資料流

`原生 C++ taiphoon_agent → 保存設定／套件 manifest → Docker Compose → MicroXRCEAgent → ROS2 Agent`

- 支援 Ubuntu 22.04 AMD64／ARM64、Windows x64、macOS Intel／Apple Silicon。
- 零參數等同 start；首次設定 deploy、sim 或 dev，之後重用設定。stop／config 切換情境。
- deploy 啟動一個 Agent，可指定無人機編號。飛控預設 `192.168.1.14`，UDP client 需指向 Agent 主機。
- sim 預設一組，支援 1～4 組 PX4＋Agent；dev 可選模擬或實機，先容器建置再啟動。
- 啟動不等待實機資料、不自動 Arm／起飛，實機不注入模擬感測資料；資料鏈仍使用 ROS2／DDS。

## 網路與持久化

- Linux 實機使用 host network；Desktop 使用 bridge，自動映射 XRCE UDP 與 DDS unicast port。
  不要求 Desktop 全域 host networking、不固定 context、網卡、Docker API 或安裝版本。
- 多機模擬 Agent 共用 agent_1 網路命名空間，避免 DDS participant 的 NAT 埠互相衝突；
  各 PX4 仍獨立執行，分別連向 agent_1 的 UDP 8888～8891，先等待對應 Agent 健康。
- local 模擬只使用內部網路；wifi 可設定 GCS IPv4。每次啟動以不傳封包的 UDP 路由查詢取得
  公告 IPv4，網路偵測失敗只提示，仍使用 DDS 自動探索。Wi-Fi 改變後 restart 套用新設定。
- GCS 與 Agent 使用一致 domain；DDS participant auto index 0～40，映射 `7400 + 250 × domain + 10～91`。
- dev 掛載原始碼，共用按 CPU 分開的 build／install／log volumes；deploy 不掛載或編譯原始碼。
- 舊設定補預設值、保留未知進階欄位。stop 保留容器、映像、設定、volume，不操作其他 Compose 專案。

## 必要檢查與失效

- 拒絕非法 ID、1～4 以外的機數、損毀套件、工具／manifest 版本不符、非 Linux／錯誤 CPU 映像。
- 首次載入校驗 archive SHA256；既有映像依 ID 或完整檔案層確認內容，忽略儲存方式造成的 Config 差異。
  runtime entrypoint 由 Compose 固定，保留已發布 manifest 相容性。
- Docker／Compose 不可用時提供處理方式，不清除設定；飛控離線不阻止啟動。
- 執行中設定變更需先 stop；重複 start 保留健康服務，部分失效需明確 restart。
- 不修改 Docker Desktop 全域設定、容器權限、飛控參數、ROS2 Node／Topic／訊息或安全門檻。

## 驗證與發布

既有 Docker dev 映像內先編譯 workspace，再驗證設定遷移、Desktop／Linux Compose、1／4 組模擬、
Wi-Fi 埠與映像儲存差異。實際 SITL 僅讀取狀態，不發布飛行命令。
GitHub Actions 編譯 AMD64／ARM64、Windows x64、macOS universal，通過後發布五個自包含平台包。
樹莓派硬體、macOS 實機 Wi-Fi 與外部 GCS 需目標平台驗收，不由編譯成功推論。

操作見 [README](../README.md)，CI 維護見 [ci-cd.md](ci-cd.md)，結果見 [deployment-verification.md](deployment-verification.md)。
