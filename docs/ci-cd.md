# CI/CD 維護與發布

## 自動流程

| 觸發 | 工作 | 產物 |
|---|---|---|
| 分支 push／PR | Ubuntu 22.04 AMD64／ARM64、Windows x64、Mac universal 編譯；Docker 內測試 | 成功狀態／失敗診斷 |
| 新 `vX.Y.Z` 或 `vX.Y.Z-rc.N` tag | 共用相同驗證，再匯出映像與打包 | 五個 GitHub Release 平台包 |
| Actions 手動執行 Release workflow，Use workflow from 選擇既有版本標籤 | workflow 與原始碼皆使用該標籤的固定 commit 重新建置 | 完整五平台 Release；不移動標籤 |

`agent-ci.yml` 與 `agent-release.yml` 共用 `agent-build.yml`，維護建置邏輯只改一份。
一般 CI 不匯出大型映像。Docker layer cache 按 CPU 分開，cache 失效不影響完整建置。
CI 使用唯讀權限；Release 呼叫端提供 contents write，用於未公開草稿傳輸與最後發布，不需要個人發布 token。
共用 workflow 繼承呼叫端權限，不會提高 CI 的權限。

## 產物傳輸與儲存

- 不使用 `actions/upload-artifact`／`actions/download-artifact`，也不建立 Docker build record artifact。
- Release 的原生工具、映像與內部 manifest 直接暫存在同版 GitHub Release 草稿，跨 runner 下載時核對 GitHub SHA256 digest。
- staging 名稱包含 run ID、attempt 與平台；只選取同一 run 的完整平台集合，支援只重跑失敗工作。
- 五個完整包上傳並校驗後，publish 工作清除草稿中的 staging 檔案，再公開 Release。任一必要工作失敗時保留未公開草稿供重跑。
- 已公開版本拒絕修改；原始碼一律由既有版本標籤解析成固定 commit，不以目前分支替代。
- 一般 CI 只建置與測試，不上傳產物。失敗診斷留在工作 log；Docker layer cache 仍沿用既有 cache 設定。
- 手動重建範例：執行 `gh workflow run agent-release.yml --ref <尚未公開的版本標籤>`，或在 Actions 頁面的 Use workflow from 選擇該標籤。
  `v1.3.0-rc.4` 及更早的標籤仍保留舊 workflow，需另外加上 `-f release_tag=<同一標籤>`。
  在分支上手動執行 Release workflow 會直接失敗；標籤在執行期間被移動時也會停止。
- 重建使用標籤當時的 workflow 與發布工具，發布流程的修正只會套用到之後的新標籤。

## 驗證範圍

- AMD64／ARM64 原生 ROS2 workspace 建置與套件測試；ARM64 不以 QEMU 編譯 Agent。
- Docker 內三組 CTest：設定／Compose／程序、CLI 生命週期與映像儲存方式、五個平台打包。
- Linux 兩種架構在隔離 DDS domain 執行降落／重飛的 planner 節點回歸，並驗證 cubic 曲線邊界判定。
- runtime 無飛控健康檢查；AMD64 再以真正 CLI 啟動 1／4 組 PX4 SIH，只讀取狀態並正常停止。
- Windows MSVC 與 macOS Intel／Apple Silicon universal 原生工具編譯；不在 host 執行 ROS2。
- 硬體 UART、樹莓派負載、Mac／Pi Wi-Fi、外部 GCS 與飛行行為仍需目標平台驗收。

## 發布

發布前先撰寫 `docs/releases/<版本標籤>.md`，例如 `docs/releases/v1.3.0-rc.3.md`。
Release workflow 讀取該版文件，補上版本與原始碼連結；缺少文件時停止發布。

### Release 說明格式

沿用 [Taiphoon_GCS](https://github.com/Rex-Taiphoon/Taiphoon_GCS/releases) 的版本資訊、平台下載表、
主要變更與 Full Changelog，修正清單參考 [Betaflight](https://github.com/betaflight/betaflight/releases) 的簡短條列。

- 章節順序固定為「平台下載 → 使用方式 → 主要變更 → 驗證與限制」，每版只保留一組主要變更標題。
- 平台表使用「平台／完整套件」兩欄，連結指向該版實際發布的檔案。
- 每項變更用一個條列，直接寫「修正／新增／調整」的對象、發生情境與結果；項目多時再分為修正、新增與改善。
- 以最近一個已發布版本為比較基準；首次發布列出新增功能。補上對應的 Full Changelog 連結。
- 直接說明行為與適用條件，避免「不等於／不代表」的對比句，以及只有「提升體驗／全面優化」的空泛描述。
- 驗證只列已完成的項目；待驗收的平台、連線或飛行情境另列一點，詳細紀錄留在驗證文件。

### 版本標籤

依專案規約確認部署變更與 CI 授權後，提交／推送分支，再推送尚未使用的版本標籤，例如：

```sh
git tag -a v1.3.0-rc.4 -m "Taiphoon Agent v1.3.0-rc.4"
git push origin v1.3.0-rc.4
```

版本標籤通過所有必要工作後才發布；`-rc.N` 為 prerelease，其餘為正式版本。
範例請改成下一個未使用版本，不移動或覆寫標籤、既有 Release 或資產。
每個包包含原生工具、README、安裝工具、對應 CPU 的 runtime／dev 映像與內部 manifest；
使用者只下載一包，不另發布校驗檔。Mac 工具為 universal，映像依下載包架構區分。

## 失敗與維護位置

先查看 Actions 的失敗步驟；Linux 將容器、runtime 與 colcon 診斷摘要輸出至工作 log。
暫時的 runner／網路錯誤可重跑 failed jobs；程式問題提交新版本。發布失敗先確認既有 Release 資產狀態。
需要足夠的 GitHub Actions runner 額度；產物不占用 Actions Artifact 儲存，所有測試完成前不發布不完整套件。

| 要修改的內容 | 位置 |
|---|---|
| ROS2 依賴、預編譯映像 | Dockerfile、docker/runtime-*.sh |
| runner／PX4 訊息 commit／建置參數／CI 驗證 | .github/workflows/agent-build.yml |
| PR 觸發／發布條件 | agent-ci.yml、agent-release.yml |
| Release 內容與格式 | docs/releases/*.md、docs/ci-cd.md |
| 平台打包與安裝 | Tools/agent_launcher/package_release.py、install.* |
| 模式、網路、程序與設定 | Tools/agent_launcher/*.cpp |
| CTest、真實模擬與唯讀觀測 | test/agent_launcher*、test/deployment_* |

內部 manifest 的 ID／完整 Config 可能受 Docker 儲存方式影響；啟動工具以 ID 或完整檔案層確認
映像內容，再確認 CPU／OS。舊 Config 欄位保留相容性，不當作環境一致性 Gate。
新增平台時同步修改 matrix、打包與測試、README 平台表。
