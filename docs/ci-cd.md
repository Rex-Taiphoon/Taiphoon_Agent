# CI/CD 維護指南

## 日常開發

提交任何分支或建立 PR，GitHub Actions 自動執行 `Taiphoon Agent CI`。
Linux AMD64／ARM64 都在原生 runner 的 Docker 開發容器內完成 workspace 建置、
ROS2 單元測試、原生啟動工具與打包測試，並驗證 runtime 無飛控啟動。
Windows／macOS runner 編譯原生入口；不在 host 執行 ROS2 測試。
同一分支的新提交取消舊 CI；Release 不因新提交取消。

建置步驟集中於 `.github/workflows/agent-build.yml`，CI 與 Release 共用它。
一般 CI 傳入 `verification_only: true`，省略大型映像匯出與套件上傳。
CI 僅使用唯讀 repository 權限；只有 Release 的 publish 工作有寫入權限。
不要為 PR 加入部署 token，也不要改用 `pull_request_target` 執行 PR 程式碼。

## 發布版本

確認 CI 成功、變更已提交，依專案 Git 操作規約取得人類確認後：

```bash
git tag -a v1.3.0-rc.1 -m "Taiphoon Agent v1.3.0-rc.1"
git push origin v1.3.0-rc.1
```

範例版本需換成下一個尚未使用的版本。`vX.Y.Z-rc.N` 自動發布為 prerelease；
`vX.Y.Z` 為正式 Release。標籤觸發 `Taiphoon Agent Release`：
雙架構容器與 Windows／macOS 編譯全部通過後，才組裝並發布五個自包含套件。
不另外上傳校驗檔。每個套件的校驗 manifest 與對應 Docker 映像一起提供。
不需要維護個人 access token；發布使用該次工作的 GitHub token。

Actions 的手動執行可產生測試套件，版本為 `v0.0.0-ci.RUN`，不建立 Release。
Artifacts 保留七天，供開發者驗證；正式下載請使用 Release assets。
GitHub repository 必須啟用 Actions 並允許 workflow 建立 Release，且帳戶需有足夠 runner／artifact 額度。

## 失敗處理

在 Actions 選擇失敗 run，先查看失敗步驟。Linux 工作失敗會上傳
`diagnostics-linux-ARCH`，包含容器狀態、runtime 日誌與可取得的 colcon 日誌。
網路或 runner 暫時失敗可使用 GitHub 的 Re-run failed jobs；程式修正請提交新版本。
不移動或覆寫已推送的版本標籤，也不覆寫既有 Release 資產。
如果發布 API 中途失敗，先檢查 Release 是否存在及五個資產是否完整，再由維護者決定處置。
編譯通過不代表飛控／DDS 實機驗收通過；樹莓派與 macOS 驗收另記錄於部署驗證文件。

## 維護位置

| 變更 | 位置 |
|---|---|
| ROS2／系統依賴與 runtime | `Dockerfile`、`docker/runtime-*.sh` |
| PX4 訊息固定 commit、runner、建置參數 | `agent-build.yml` |
| 提交／PR 觸發與取消規則 | `agent-ci.yml` |
| 發布條件、說明與權限 | `agent-release.yml` |
| 平台套件內容／大小限制 | `Tools/agent_launcher/package_release.py` |
| CLI 與安裝流程 | `Tools/agent_launcher/` |

新增平台時同步修改 build matrix、打包程式、打包測試與 README 平台表。
更新 PX4 訊息 commit 前，在 Docker 內重新建置與測試，並確認現場飛控版本相容。
ARM64 runner 是原生建置；macOS 原生入口包含 Intel／Apple Silicon，但映像仍按下載包架構區分。
GitHub 的可重用 workflow 規則參考 [官方文件](https://docs.github.com/en/actions/how-tos/reuse-automations/reuse-workflows)。
