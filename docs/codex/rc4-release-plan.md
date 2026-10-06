# v1.3.0-rc.4 發布計畫

主角色：部署與發布工程師；協作責任：測試驗證與介面維護，由同一代理執行。人類已要求將目前版本發布為 rc.4 並執行多平台建置，授權必要的版本提交、分支／標籤推送與既有 Release workflow。

發布內容包含目前已驗證的降落任務生命週期、2 m 預設間距、Agent 關閉後清理、測試精簡與小場域驗證工具，也包含 rc.3 之後已提交的垂直編隊高度偏移修正。ROS 套件版本維持合法數字版本 1.3.0；原生 launcher、映像與 manifest 由標籤 v1.3.0-rc.4 產生。README 指向本次預發布版。

資料流沿用 agent-release.yml → 共用 agent-build.yml → Linux AMD64／ARM64 容器與原生工具 → Windows／macOS 原生工具 → 五平台打包 → GitHub prerelease。新增降落 ROS2 節點回歸與 cubic 曲線邊界回歸至 Linux 容器驗證；Docker context 保留 AutoTest/scripts，仍排除 AutoTest/Log。新回歸使用 DDS domain 77，不接觸目前人工 GCS 操作機群。

相容性與失效：任一必要工作失敗時既有流程不發布；不覆寫已存在的 tag 或資產。只有實際完成的驗證寫入版本說明。硬體、實際降落再起飛與失效後完整三機飛行安全驗收仍待測；既有失效 Agent 2 後 2 m 距離不足與交叉路徑越界證據明列於 rc 說明。

驗證：沿用已完成的 254 個單元測試與節點回歸，再在獨立 Docker 容器執行新增 workflow 命令與 launcher CTest。確認 Git diff、版本文件與打包路徑後提交「發布: 準備 v1.3.0-rc.4 多平台套件」，推送新版本標籤，追蹤 Actions 各工作與 Release 五個資產及其來源 commit。測試紀錄與臨時操作工具不提交。
