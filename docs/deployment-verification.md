# 跨平台部署驗證紀錄

日期：2026-10-02（Asia/Taipei）

## 審查範圍

- 分支：`codex/cross-platform-agent-launcher`。
- 基準：`6488cc4`；既有 `src/nodes/ego_swarm_planner_node.cpp` 未提交修改保持原狀，不納入部署提交。
- 主角色：Docker／部署維護人員；協作責任包含架構、測試與文件審查。
- 新增原生 C++ 入口、模式設定、映像載入、runtime entrypoint、Release CI 與五個平台下載包。
- 沒有新增／修改 ROS2 Node、Topic 或訊息定義；飛行命令與安全門檻保持既有實作。

## 已執行

全部 ROS2 建置與測試在既有 `taiphoon-agent:dev` Docker 開發容器執行；沒有執行 Docker build。

| 項目 | 實際結果 |
|---|---|
| 目前工作區 workspace 編譯 | 成功 |
| 目前工作區套件測試 | 328 tests，0 errors，0 failures，0 skipped |
| HEAD 快照加部署變更，排除既有未提交修改 | 編譯成功 |
| 乾淨快照 Release 編譯與套件測試 | 328 tests，0 errors，0 failures，0 skipped |
| 原生 Linux C++ 入口 | 編譯成功 |
| 設定、SHA256、模式、DDS、串列與 Windows argv 引用 | 通過 |
| 零參數首次啟動、重複啟動、停止、設定變更、舊入口衝突與損毀映像 | 隔離程序替身測試通過 |
| 真正 CI 打包步驟 | 五個自包含平台包通過內容驗證；沒有獨立校驗檔或映像下載檔 |
| Ubuntu／macOS 安裝工具 | 中文與空白路徑、重複安裝、PATH 不重複設定通過 |
| runtime 無飛控啟動 | 程序健康，PX4 狀態仍為等待／無效 |
| 橋接程序終止 | runtime 回傳非零並停止自有程序 |
| Shell 語法與 git diff 空白檢查 | 通過 |

未最佳化快照曾出現兩個 planner 時間預算案例逾時（約 38～40 ms），當時另有編譯工作。
停止其他編譯後，只重跑失敗 suite 即成功；完整 Release 編譯後全部測試通過。
未更改 planner 演算法、測試預期、時間預算或安全門檻。

## GitHub CI 與 Release 已完成

第一輪 GitHub CI：Windows exe 與 macOS universal binary 編譯成功；ARM64 的新 dev／runtime
映像建置成功，328 個 ROS2 測試全部通過，啟動工具的單元與程序整合測試通過。
打包測試因 workflow 拆分後仍讀取舊位置而失敗，已改讀共用 `agent-build.yml`，
Docker 內重跑三組 CTest 全數通過；完整 CI 重跑與 runtime 驗證亦已通過。

- [完整 CI](https://github.com/Rex-Taiphoon/Taiphoon_Agent/actions/runs/37003026441)：全部必要工作成功。
- [版本發布流程](https://github.com/Rex-Taiphoon/Taiphoon_Agent/actions/runs/37006614126)：全部工作成功。
- AMD64／ARM64 各有 328 個 ROS2 測試通過，三組啟動與打包測試通過，runtime 無飛控啟動檢查成功。
- Windows 實際產物為 x64 PE；macOS 實際產物含 Intel 與 Apple Silicon，皆包含正確版本字串。
- [v1.3.0-rc.1](https://github.com/Rex-Taiphoon/Taiphoon_Agent/releases/tag/v1.3.0-rc.1)
  已發布五個自包含平台包，大小約 814～844 MiB，所有資產狀態為 uploaded，沒有獨立校驗檔。
- 版本標籤指向 `a6ff15e`；後續驗證文件更新不改動已發布的二進位檔。

## 尚待目標平台驗收

- 樹莓派實體 UART、PX4／DDS、感測器與 CPU／記憶體效能。
- Docker Desktop host networking、LAN 公告位址與跨機 DDS。
- Windows／macOS 的下載後啟動與作業系統核准流程；macOS 尚未 Apple notarization。

上述驗收不包含自動 Arm 或起飛。CI 任一必要建置／測試失敗不發布 Release。
Release 發布需人類確認本次 Git commit／push 與 CI Docker build 授權；符合根目錄 AGENTS.md 的 Gate。
