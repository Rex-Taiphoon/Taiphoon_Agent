# v1.3.0-rc.4 發布計畫

主角色：部署與發布工程師；協作責任：測試驗證與介面維護，由同一代理執行。人類已要求將目前版本發布為 rc.4 並執行多平台建置，授權必要的版本提交、分支／標籤推送與既有 Release workflow。

發布內容包含目前已驗證的降落任務生命週期、2 m 預設間距、Agent 關閉後清理、測試精簡與小場域驗證工具，也包含 rc.3 之後已提交的垂直編隊高度偏移修正。ROS 套件版本維持合法數字版本 1.3.0；原生 launcher、映像與 manifest 由標籤 v1.3.0-rc.4 產生。README 指向本次預發布版。

資料流沿用 agent-release.yml → 共用 agent-build.yml → Linux AMD64／ARM64 容器與原生工具 → Windows／macOS 原生工具 → 五平台打包 → GitHub prerelease。新增降落 ROS2 節點回歸與 cubic 曲線邊界回歸至 Linux 容器驗證；Docker context 保留 AutoTest/scripts，仍排除 AutoTest/Log。新回歸使用 DDS domain 77，不接觸目前人工 GCS 操作機群。

相容性與失效：任一必要工作失敗時既有流程不發布；不覆寫已存在的 tag 或資產。只有實際完成的驗證寫入版本說明。硬體、實際降落再起飛與失效後完整三機飛行安全驗收仍待測；既有失效 Agent 2 後 2 m 距離不足與交叉路徑越界證據明列於 rc 說明。

驗證：沿用已完成的 254 個單元測試與節點回歸，再在獨立 Docker 容器執行新增 workflow 命令與 launcher CTest。確認 Git diff、版本文件與打包路徑後提交「發布: 準備 v1.3.0-rc.4 多平台套件」，推送新版本標籤，追蹤 Actions 各工作與 Release 五個資產及其來源 commit。測試紀錄與臨時操作工具不提交。

發布期間遇到 Actions Artifact 儲存額度已滿。人類另核准刪除 7 個 2026-10-02 舊大型 artifacts（約 13.1 GB），並要求後續流程避免占用 Artifact 額度。既有 Release、原始碼、工作 log 與小型診斷保留。已完成清理；停止受額度阻擋的舊 Release 工作，重新執行採 Release 草稿傳輸的流程。

新增 release_transfer.py 與隔離 Docker 回歸：CI 保持 readonly，Release 繼承呼叫端 contents write；只對未公開且 source commit 一致的草稿傳送檔案。使用 run／attempt／平台命名及 SHA256 校驗，全部平台成功後才組包；五包完整校驗後清除 stage 並公開。已發布版本禁止修改。新的 workflow 支援從 master 指定現有 rc.4 標籤重建，來源仍固定為 ff706d62f44d20eb0986bcd1c979306346d0685c，不移動 rc.4 標籤。

2026-10-06 人類重新授權執行多平台 Workflow。主角色為 Docker／部署維護，協作責任為測試驗證，由同一代理完成。前次 Linux CI 的 278 個 colcon 測項皆通過，整合回歸因指定未安裝的 Fast DDS 而失敗；僅修改 Workflow 測試環境，使用既有 Cyclone DDS、domain 77、loopback 與 localhost peer。Release 草稿查找改為分頁列出可見版本，再依數字 ID 讀取，防止 REST tags endpoint 的 404 造成重複草稿。測試先以獨立 Docker 執行傳輸 Gate、打包及真實 planner 回歸，預期全部通過；再提交流程修正並從 master dispatch rc.4，持續追蹤四組 runner 與五平台下載包。既有 tag 與飛行程式碼保持來源 ff706d6。

重跑前本機驗證：隔離 Docker workspace 建置成功、7 個 Release 傳輸 Gate 測試通過、五平台打包 fixture 通過、Cyclone DDS 真實 planner 回歸通過（19.932 s）。顯式 lo 配置與 ROS_LOCALHOST_ONLY 會重複選取介面，改為 ROS_LOCALHOST_ONLY=1 並清空 CYCLONEDDS_URI 使用 RMW 內建 localhost 設定；測試失敗不改預期。官方 gh CLI 的 Release 查找支援草稿，可沿用 upload／download。
