# Windows Docker 三機小場域驗證結果

日期：2026-10-06。主角色為安全監控工程人員，兼任測試與部署驗證。

## 結論

2 m 預設間距及測試精簡已完成，但現有實作未通過指定小場域的完整安全驗收。
停止 Agent 2 後，剩餘兩機重編隊出現低於 2 m 的距離；交叉移動產生超出場域的曲線。
這些是模擬與幾何驗收結果，不代表已證明實體接觸碰撞，也不能當作實機零碰撞保證。

## 本輪範圍與變更

- 依開發者最後決定，只改 2 m 間距、精簡必要測試及驗證前五步。
- Node、PlannerConfig、LocalGoalSelector、Launch、dev／runtime entrypoint 預設一致改為 2 m。
- 1 m 規劃額外裕度維持原值；沒有修改避碰演算法、PX4 失聯參數或 RTL 控制流程。
- 沒有新增正式場域 hard gate。測試觀測器使用共同起飛中心的 North／East ±4.5 m、限高 8 m。
- 預設排除三組未接入現行 runtime 的舊模型測試，保留原始檔案，以
  `TAIPHOON_BUILD_LEGACY_TESTS=ON` 選配；保留交叉衝突、動態避障與必要回歸。
- 新增 2 m 門檻與三機轉兩機槽位幾何測試。既有 3 m 阻塞路線案例明確指定 3 m，
  保留原場景與原斷言，避免部署預設變動使案例失去作用。
- 三機起始一字間距 3.2 m；起飛 2 m；圓半徑 2 m；前／後各移動 1 m。
- 本輪未下達 RTL 命令；停止 Agent 後 PX4 的自主處置仍會觸發 RTL／降落。
- 所有建置、ROS2 與飛行測試均在 Docker；Windows PowerShell／Git Bash 僅調度 Docker。
  全程不使用 WSL，不掛載 Docker socket 到 Agent 或測試容器。

## 驗證結果

| 項目 | 結果 | 證據與限制 |
|---|---|---|
| ROS2 workspace 建置 | PASS | Docker dev 容器內 `colcon build --symlink-install` |
| 預設單元測試 | PASS | 23 個 gtest target、249 個案例，0 失敗／錯誤／略過 |
| 部署工具驗證 | PASS | 設定／CLI 整合／套件三個 CTest 全數通過 |
| 邊界驗收器回歸 | PASS | 兩個案例：控制點超界但曲線在內不誤判；實際曲線超界留下證據 |
| Windows 原生編譯 | PASS | Docker 內 MinGW 交叉編譯 v1.3.0 啟動器 |
| Windows 退出清理 | PASS | 實際隱藏程序等候測試容器退出，程序回傳 0，容器隨後不存在 |
| 起飛／三機環形／前後移動 | PASS | 停止 Agent 1、2、3 的測試均完成前三步 |
| 停止 Agent 2 後繼續編隊 | FAIL | 多次重跑仍失敗；最終即時 1.993 m，PX4 共同時間軸最小 1.800 m（Agent 1–3） |
| 停止 Agent 1 後繼續編隊 | INCONCLUSIVE | 剩餘兩機連續收斂 15 秒；已記錄區間最小 2.772 m，失效機落地後停止 ULog，最後約 10 秒缺少位置資料 |
| 停止 Agent 3 後繼續編隊 | INCONCLUSIVE | 剩餘兩機連續收斂 15 秒；已記錄區間最小 2.734 m，失效機落地後停止 ULog，資料未涵蓋最後觀察區間 |
| 小場域交叉移動 | FAIL | Agent 1／3 互換 East 位置，中間機保持；Agent 3 曲線 North 極值 4.604 m，超出 4.5 m 邊界 |
| 第六步 RTL 指令 | NOT RUN | 依開發者決定暫不執行 |

Agent 2 故障情境的三份 ULog 均涵蓋驗收期間，最大位置取樣間隔約 8 ms；
離線距離以 20 Hz 共同 UTC 時間軸查核，使用各 ULog 的 `boot_time_utc_us`，
不直接比較不同機體的開機時間戳。最終重跑最大觀測高度為 3.314 m，水平軸最大絕對值 3.116 m。
失效機仍飛行期間也納入距離計算。Agent 1／3 的日誌最後確認落地與上鎖，
但不能以最後位置直接外插缺失區間並宣告通過。

交叉移動的失敗是「發布的未來曲線超界」，停止測試時實際機體水平軸最大絕對值 3.108 m，
沒有把未執行完的未來軌跡誤稱為已發生實際越界。
邊界驗收對 cubic B-spline 的每個 knot span 求座標極值；控制點凸包超界本身不判 FAIL。

實際 PX4 記錄參數為 COM_OBL_RC_ACT=0、COM_OF_LOSS_T=1 s、RTL_RETURN_ALT=30 m、
RTL_DESCEND_ALT=10 m。原始 console 同時記錄 Failsafe activated、RTL start、land at destination。
沒有因預設參數為 30 m 就宣稱本輪已飛到 30 m；以實際軌跡結果為準。

## 原始記錄

專案內 `AutoTest/Log/small-field-20261006/`：

- `unit-retest.log`：249 個單元案例的完整結果。
- `launcher-tests.log`：原生部署工具三項 CTest 結果。
- `formation-failed-2-141917/`：可重現的間距失敗，`monitor.jsonl`、三份 PX4 ULog、Agent／PX4 console、`px4-analysis.json`。
- `formation-failed-2-143318/`：最終間距失敗重跑，另確認 Agent 2 遠端關閉後立即保存日誌與移除容器。
- `formation-failed-1-141717/`、`failed-3-141520/`：15 秒重編隊與資料覆蓋限制。
- `crossing-failed-2-142809/`：精確曲線邊界的越界證據與 PX4 日誌。
- `removed-container-inventory.txt`：人工授權清理的停止容器盤點。
- `launcher-windows/taiphoon_agent.exe`：容器編譯的 Windows 驗證產物；不是完整 Release 套件更新。
- `monitor-unit-tests.log`：控制點凸包與實際曲線的兩個回歸案例。

第一輪起飛超時、短時間觀察與控制點凸包版本的記錄仍保留，僅採上述最終重跑作為主要證據。

## 容器退出清理

開發者另行授權移除無需保留的停止 Agent 容器，並要求 Agent 關閉後移除。
已清理 25 個先前測試容器；最新驗證確認 Agent、observer、one-off build 不再留下停止容器。
日誌、映像、命名 build／install／log volumes 與 GCS 保留。

資料流為主機清理程序 `docker wait 完整ID` → 確認已停止 → `docker rm 完整ID`。
Agent 不取得 Docker 權限。只使用完整 ID，避免名稱被新容器重用後誤刪；
不使用強制移除，若 Docker 不可用或容器仍執行，清理程序不刪除。
原生啟動器 `stop`／`restart` 使用 Compose down，保留命名 volumes。
Windows 清理程序隱藏執行；驗證腳本另外在移除前保存 Agent console。

此行為適用重新編譯的原生啟動器與新驗證腳本。舊 Release 執行檔不會自動更新，
直接 `docker compose up` 也不會自動註冊主機清理程序。

## 尚未改動的問題

現有安全流程仍需處理重編隊過程的追蹤誤差／間距、失效機空間占位與場域約束。
依本輪授權範圍保留既有飛行邏輯，不自行加入場域 gate、失聯處置或返航協調。
實機定位誤差及包含槳葉的最大外徑尚未量測；7 吋機與「高精度 GPS」不足以推出可保證的碰撞裕度。
