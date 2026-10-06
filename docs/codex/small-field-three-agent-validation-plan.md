# 9×9×8 公尺三機驗證與變更計畫

## 已確認需求

- Windows 原生 Docker CLI／Compose 調度；編譯與 ROS2 測試在 Docker 容器內執行，不使用 WSL。
- 三機一字起飛，飛行場域為 9×9×8 公尺，起飛高度 2 公尺。
- 環形編隊後全體前移 1 公尺，再後移 1 公尺。
- 第一階段失效注入只停止其中一個 Taiphoon_Agent；PX4 SITL 繼續運行。
- 剩餘兩台重編隊；本輪依最後決定暫不執行 RTL。
- 使用者希望編隊與同儕中心距離預設改為 2 公尺。
- 實機預計使用高精度 GPS 與 7 吋機；尚未提供定位誤差或含槳最大外徑。

## 介面與資料流

沿用既有 `/uav_all_agent/command`、各 Agent 的 `/state`、`/planning/status`、
`/planning/trajectory`、`/shutdown` 及共享 FormationEpoch／AgentTrajectory topic。
不新增 message，也不讓任一 Agent 控制其他 Agent 的飛控。

第一階段修改既有 `formation_min_distance_m`、`peer_collision_min_distance_m` 的預設為 2.0，
同步 Node、pure C++ config、Launch、dev／runtime entrypoint 與操作文件。
`peer_planning_margin_m=1.0` 仍保留，正常規劃目標間距為 3.0 公尺；
2.0 公尺是最終中心距離門檻，不是期望貼著門檻飛行，也不是機體外緣距離。

## 測試座標與幾何

測試固定以三台地面起飛位置的共同中心為場域中心，將每台 WGS84 資料投影到此測試座標。
這只供外部驗收使用，不改變各 Agent 的 `planner_ned` 原點。

- 場域：North／East 各為 [-4.5, 4.5] m，地面高度 [0, 8] m。
- 一字起飛的 East 位置：[-3.2, 0, 3.2] m，North=0；沿用既有 launcher 的 3.2 m 間距參數。
- 編隊：`FORMATION_CIRCLE 2 2`，理想三機相鄰中心距離為 2√3 ≈ 3.464 m。
- 前後移動：`MOVE_FORWARD 1`、`MOVE_BACKWARD 1`，使用既有全體固定 NED 軸語意。
- 移動中心最遠為 North=1 m，圓形外包範圍不超過 North=3 m；但不能由目標幾何推論整條實際軌跡均在場域內。
- 每階段依狀態與幾何收斂推進，避免以固定等待時間代替成功條件。

## 已發現的安全缺口

1. 目前 Agent 沒有 9×9 公尺邊界與 8 公尺限高的 trajectory hard gate。
   改機間距不能取代場域約束；外部監測失敗也不等於能在越界前阻止飛行。
2. `MODE_RTL` 由 BasicController 直接送 PX4 NAV_RETURN_TO_LAUNCH，返航軌跡不經 EGO 同儕避碰。
   現有 SITL 映像的 `rc.mc_defaults` 設定 RTL_RETURN_ALT=30 m、RTL_DESCEND_ALT=10 m，均超過場域上限。
   實際有效 PX4 參數仍須在啟動後讀取確認。
3. 停止 Agent 後 PX4 會依自己的 Offboard-loss failsafe 動作，並不等於飛機消失或已安全落地。
   現有 discovery 在成員集合穩定後移除該 peer，不能保證其實體不再占用空間。
4. 整台無人機斷電與停止 Agent 是不同情境；本階段 SITL 結果不證明斷電墜落安全。
5. 現有 EmergencyBrake 發布路徑未再套用完整同儕／場域驗證；若新增場域 hard gate，煞車路徑也必須納入。

## 驗收條件

- 正常與失效後仍存活的所有機體中心均不超出 9×9×8 公尺；另行量測機體外緣所需邊界裕度。
- 最小機間距不得低於 2.0 公尺，不沿用舊 monitor 的 0.5 秒短暫違反寬限。
- 三機與兩機編隊均確認 epoch 成員集合、槽位幾何、目標高度與 terminal hold。
- 前移／後移均達到目標，並確認後移後回到原編隊中心。
- 關閉 Agent 後先確認該機 PX4 的實際失聯動作，不能只量測剩餘兩台。
- 分別測試停止 Agent 1、2、3，包含 proposer 失效與非連續成員 ID。
- 兩機重編隊須連續收斂 15 秒，PX4 日誌須涵蓋失效機至觀測結束的全程。
- 另測兩端機互換位置、中間機保持原位；全程維持場域與距離驗收。
- 保留交叉移動：驗證移動中同儕、固定高度、中心距離、煞車／重規劃及到達結果。
- RTL 驗證必須有已確認的低高度設定與多機回程策略；未確認前不執行此階段。

## 單元測試審查方案

保留現行 runtime 的命令、座標轉換、狀態觀測、軌跡取樣、編隊／assignment／epoch、
同儕 freshness、optimized planner、交叉衝突、障礙物與動態煞車測試。
不能因 timeout 失敗便刪除避碰或安全失效案例。

將以下未接入現行 runtime 的舊模型測試移出預設測試集合，保留檔案及選配相容測試開關：

- `test_formation_calculator`
- `test_trajectory_planner`
- `test_collision_safety_monitor`

deterministic EGO baseline 依現有模組規約保留；共用 B-spline／validator 等核心測試不能整包刪除。
另外區分「純功能／幾何正確性」與「指定硬體的 wall-time 效能」：
功能測試應驗證結果的安全與合法性，timeout／iteration-limit 應有獨立失效測試；
20 ms 預算的既有失敗與 runtime 50 ms 預算差異要明確記錄，不得只改預期結果使測試通過。

## 需要開發者決定的後續安全變更

根據根目錄 AGENTS.md 與 CHARACTERS.md 的跨功能／安全 Gate，以下尚不能自行選定：

- 是否新增適用所有移動、起飛與煞車的場域 hard gate，以及場域中心的設定與失效處置。
- 本次 SITL 的 PX4 RTL／Offboard-loss 參數如何配置；這會改變飛行安全設定。
- 是否採取 EGO 規劃回到個別起飛點、依次降落／切換 PX4 RTL，或保留現有直接 PX4 RTL 語意。
- 失效但仍可能移動的飛機如何確認清空，或由感測器保留障礙占位。

本計畫不把設定 2 公尺或通過有限 SITL 情境視為「所有情況零碰撞」的保證。

## 開發者決定

2026-10-06：本輪先改 2 m 間距、精簡預設測試集合並驗證前五步。
不新增場域 hard gate，不更改 PX4 失聯處置或 RTL；RTL 階段暫不執行。
前五步驗證仍監測場域與失效機行為，越界、間距不足或失去必要資料即判定 FAIL／INCONCLUSIVE，
不得因為不執行第六步而忽略失效機自主觸發的 RTL。
