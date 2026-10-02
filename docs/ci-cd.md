# CI/CD 維護與發布

## 自動流程

| 觸發 | 工作 | 產物 |
|---|---|---|
| 分支 push／PR | Ubuntu 22.04 AMD64／ARM64、Windows x64、Mac universal 編譯；Docker 內測試 | 成功狀態／失敗診斷 |
| 新 `vX.Y.Z` 或 `vX.Y.Z-rc.N` tag | 共用相同驗證，再匯出映像與打包 | 五個 GitHub Release 平台包 |
| Actions 手動執行 Release workflow | 同上，使用 CI 版本 | 七天保留的測試 artifacts，不發布 |

`agent-ci.yml` 與 `agent-release.yml` 共用 `agent-build.yml`，維護建置邏輯只改一份。
一般 CI 不匯出大型映像。Docker layer cache 按 CPU 分開，cache 失效不影響完整建置。
CI 使用唯讀權限；只有 Release publish 使用 GitHub token 寫入，不需要個人發布 token。

## 驗證範圍

- AMD64／ARM64 原生 ROS2 workspace 建置與套件測試；ARM64 不以 QEMU 編譯 Agent。
- Docker 內三組 CTest：設定／Compose／程序、CLI 生命週期與映像儲存方式、五個平台打包。
- runtime 無飛控健康檢查；AMD64 再以真正 CLI 啟動 1／4 組 PX4 SIH，只讀取狀態並正常停止。
- Windows MSVC 與 macOS Intel／Apple Silicon universal 原生工具編譯；不在 host 執行 ROS2。
- 硬體 UART、樹莓派負載、Mac／Pi Wi-Fi、外部 GCS 與飛行行為仍需目標平台驗收。

## 發布

依專案規約確認部署變更與 CI 授權後，提交／推送分支，再推送尚未使用的版本標籤，例如：

```sh
git tag -a v1.3.0-rc.2 -m "Taiphoon Agent v1.3.0-rc.2"
git push origin v1.3.0-rc.2
```

版本標籤通過所有必要工作後才發布；`-rc.N` 為 prerelease，其餘為正式版本。
範例請改成下一個未使用版本，不移動或覆寫標籤、既有 Release 或資產。
每個包包含原生工具、README、安裝工具、對應 CPU 的 runtime／dev 映像與內部 manifest；
使用者只下載一包，不另發布校驗檔。Mac 工具為 universal，映像依下載包架構區分。

## 失敗與維護位置

先查看 Actions 的失敗步驟；Linux 上傳 `diagnostics-linux-ARCH`，包含容器、runtime 與 colcon 日誌。
暫時的 runner／網路錯誤可重跑 failed jobs；程式問題提交新版本。發布失敗先確認既有 Release 資產狀態。
需要足夠的 GitHub Actions runner／artifact 額度，所有測試完成前不發布不完整套件。

| 要修改的內容 | 位置 |
|---|---|
| ROS2 依賴、預編譯映像 | Dockerfile、docker/runtime-*.sh |
| runner／PX4 訊息 commit／建置參數／CI 驗證 | .github/workflows/agent-build.yml |
| PR 觸發／發布條件 | agent-ci.yml、agent-release.yml |
| 平台打包與安裝 | Tools/agent_launcher/package_release.py、install.* |
| 模式、網路、程序與設定 | Tools/agent_launcher/*.cpp |
| CTest、真實模擬與唯讀觀測 | test/agent_launcher*、test/deployment_* |

內部 manifest 的 ID／完整 Config 可能受 Docker 儲存方式影響；啟動工具以 ID 或完整檔案層確認
映像內容，再確認 CPU／OS。舊 Config 欄位保留相容性，不當作環境一致性 Gate。
新增平台時同步修改 matrix、打包與測試、README 平台表。
