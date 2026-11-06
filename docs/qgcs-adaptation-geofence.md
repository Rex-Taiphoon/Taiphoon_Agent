# QGCS 適配說明：地理圍籬與小場域安全距離

> 對象：Taiphoon_QGCS 維護人員
> 來源：Taiphoon_Agent 分支 `time-sync-offboard-timeout`（v1.4.0-rc.1 之後），2026-11-05
> 相關文件：[介面規格](taiphoon-agent-interface.md)、[參數參考](agent-configuration.md)、[圍籬設計](codex/agent-geofence-design.md)

## 1. 變更摘要

| 項目 | Agent 行為 | QGCS 是否必須修改 |
|---|---|---|
| 地理圍籬（新功能） | Agent 依部署參數的場域多邊形限制目標與軌跡，超出時拒絕命令或改走圍籬內 | 否（相容）；建議顯示新的拒絕原因、在地圖畫出圍籬 |
| 最小機間距離可設為 1.5 m | `peer_collision_min_distance_m`、`formation_min_distance_m` 可由部署改為 1.5 m（預設仍 2.0 m，驗證結果見第 5 節） | 若部署改為 1.5 m，V 字翼間距下限需同步調整 |
| 編隊直接到位、單機移動收尾修正 | 不改變介面，只改善到位精度與避碰 | 否 |

**介面相容性**：沒有新增或修改任何 ROS2 topic、msg、srv、欄位或事件代碼，CDR 排列不變；
舊版 QGCS 可直接搭配本版 Agent。只新增 `detail`／`reason` 字串內容。

## 2. 圍籬設定方式

圍籬由 **Agent 部署端**設定，QGCS 不需要也無法下發。目前提供兩種方式：

| 方式 | 名稱 | 格式 |
|---|---|---|
| Docker 環境變數 | `EGO_SWARM_GEOFENCE_POLYGON` | `lat,lon;lat,lon;...`（WGS84 度，3～64 點，依序相連，不需重複第一點） |
| Docker 環境變數 | `EGO_SWARM_GEOFENCE_MARGIN_M` | 機體中心與邊界的最小水平距離（m），預設 `0.5` |
| ROS2 參數 | `geofence_polygon`、`geofence_margin_m` | 同上 |

- 只限制水平位置，不限制高度；每台 Agent 應設定相同的圍籬。
- 空字串表示停用，行為與舊版完全相同。
- 格式錯誤、自交、頂點距 Home 超過 5 km 時 **fail closed**：所有移動目標都會被拒絕（拒絕原因與越界相同），
  起飛、降落、懸停不受影響。Agent 日誌會出現 `Invalid geofence_polygon` 或 `Geofence polygon is self-intersecting`。
- 實機啟動工具（`settings.ini`）目前尚未提供圍籬欄位；啟用後會另行通知。

**建議**：Agent 不會對外發布圍籬內容。QGCS 若要在地圖上畫出場域或在 GuardModule 預先檢查目標，
請在 QGCS 設定中保存同一份頂點列表與 margin，格式沿用上表，方便直接複製給 Agent 部署。

## 3. 新的拒絕與狀態字串

以下字串都是既有欄位的新內容，建議在 `ui_format.cpp` 依 `detail`／`reason` 轉成中文提示：

| 來源 | 條件 | 英文字串 | 建議中文提示 |
|---|---|---|---|
| `AgentEvent` 400 `PLANNER_COMMAND_REJECTED`，`command` 為 `MOVE_*` | 單機相對移動目標在圍籬（含 margin）外 | `detail = "Goal outside geofence"` | 目標超出場域圍籬，已拒絕 |
| 同上，`command` 為 `MOVE_POINT` | 目標座標無效或在圍籬外 | `detail = "Goal invalid or outside geofence"` | 目標無效或超出場域圍籬，已拒絕 |
| 同上，`command` 為全體 `MOVE_*` | 移動後任一編隊槽位在圍籬外 | `detail = "Formation slot outside geofence"` | 編隊移動後會超出場域圍籬，已拒絕 |
| 同上，`command` 為 `FORMATION` | 新隊形任一槽位在圍籬外（由提案的 Agent 發布） | `detail = "Formation slot outside geofence"` | 隊形超出場域圍籬，已拒絕 |
| `WaypointMissionStatus` `REJECTED` | 任一航點在圍籬外 | `reason = REASON_OUT_OF_RANGE`、`detail = "waypoint outside geofence"`、`current_index` = 第一個越界航點 | 第 N 個航點超出場域圍籬 |
| `AgentEvent` `PLANNER_EMERGENCY_BRAKE` | 執行中軌跡重新驗證失敗且無法改走 | `detail` 含 `geofence_violation` | 軌跡將超出場域圍籬，緊急煞停 |
| `PlannerStatus.reason` | 圍籬內暫時沒有可行路徑（例如同儕擋住唯一通道），原地等待重試 | `local_goal_selection_failed` 或 `optimized_no_feasible_horizontal_path` | 等待可行路徑（場域受限） |
| `PlannerStatus.reason`（`state` 為 RUNNING） | 依進度重新規劃停滯，改為飛完目前安全軌跡（小場域、目標靠近圍籬時較常見） | `Progress stalled; following active trajectory to completion` | 進度停滯，沿目前路線飛往目標 |

- 航點越界沿用 `REASON_OUT_OF_RANGE`，沒有新增 msg 常數；QGCS 若要區分「距離過遠」與「超出圍籬」，請判斷 `detail`。
- 拒絕發生在命令階段，Agent 目標不變（維持原本的懸停或任務）。
- 等待狀態不是錯誤：同儕讓開後 Agent 會自動繼續，QGCS 不需要重送命令。
- 停滯字串會持續到 Agent 換成新軌跡為止（通常 0.5～2 秒），之後回到 `ok`；一次移動可能出現數次，
  都不是錯誤，建議以一般提示顯示，不要跳出警告。
- 既有的 `Emergency brake: target reached while moving`（`state` 為 HOLD）表示飛機到位時仍在移動，
  Agent 原地煞停後進入懸停；小場域幾乎每次到位都會出現。建議 QGCS 對這個字串降低警示等級，
  與真正的避碰煞停（`optimized_emergency_brake: ...`）分開顯示。

## 4. GuardModule 與輸入範圍

- `app_settings.cpp` 的 `V_SHAPE_SPACING` 下限 2 m 對應 Agent `formation_min_distance_m`。部署改為 1.5 m 時，
  請把下限同步改為 1.5 m，否則 QGCS 會擋下 Agent 可接受的隊形；仍維持 2 m 時不需修改。
- `CIRCLE_RADIUS` 下限 1 m 不變。Agent 以槽位中心距離檢查：N 台環形的相鄰距離為 `2·R·sin(π/N)`，
  小於 `formation_min_distance_m` 時拒絕（既有行為）。
- 設定圍籬後，建議 QGCS 在送出前預先檢查：
  - 單機 `MOVE_POINT`、航點：目標點到圍籬邊界的距離 ≥ margin。
  - 全體移動與編隊：中心加上每個槽位偏移都要在圍籬內。
  預檢只是改善使用者體驗，最終仍以 Agent 判定為準。

## 5. 1.5 m 最小間距

開發者已同意小場域可將最小機間距離降為 1.5 m（目標是不在空中相撞）。Agent 端以
`EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M=1.5`、`EGO_SWARM_FORMATION_MIN_DISTANCE_M=1.5` 設定，
正在以 8×8 m 場域三機重複驗證；結果與是否改成預設值會更新在本節。

目前狀態（尚未套用，Agent 預設仍為 2.0 m）：

- 設圍籬的 8×8 m 場域：編隊全部通過。Agent 已新增停滯偵測，交叉測試不再逾時；
  但 5 次交叉中仍有 1 次實測最小機間距離 1.496 m，略低於 1.5 m（2.0 m 設定也有 1 次實測 1.91 m）。
  都發生在交叉機繞過中央懸停機時，原因是 PX4 追蹤誤差，Agent 端尚未處理。
- planner 只保證計畫軌跡不低於設定值，不另外預留 PX4 追蹤誤差；不設圍籬的情境下，
  1.5 m 的實際距離可能低於 1.5 m。因此 QGCS 暫時不要把 1.5 m 當成一般預設值，
  `V_SHAPE_SPACING` 下限維持 2 m，待本節更新後再調整。

## 6. QGCS 驗證建議

以 Agent `docker compose`（或 `AutoTest` 腳本的 `-GeofencePolygon`）啟動三機與圍籬後：

1. 單機 `MOVE_POINT` 到圍籬外 → 顯示「目標超出場域圍籬」，飛機不動。
2. 環形編隊後全體向圍籬方向移動，移動距離足以讓槽位越界 → 顯示編隊越界提示，隊形不變。
3. 上傳含圍籬外航點的任務 → 顯示第 N 個航點越界。
4. 圍籬內正常起飛、編隊、移動、失效重組，地圖軌跡全程在圍籬內。
5. 停用圍籬（環境變數留空）時，所有行為與舊版相同。
