# QGCS 適配說明：地理圍籬與小場域安全距離

> 對象：Taiphoon_QGCS 維護人員
> 來源：Taiphoon_Agent 分支 `time-sync-offboard-timeout`（v1.4.0-rc.1 之後），2026-11-06 更新
> 相關文件：[介面規格](taiphoon-agent-interface.md)（3.7、3.14～3.16、4.4）、[圍籬設計](codex/agent-geofence-design.md)

## 1. 變更摘要

| 項目 | Agent 行為 | QGCS 是否必須修改 |
|---|---|---|
| 地理圍籬改由地面站設定（新介面） | 圍籬只能由 GCS 發布到 `/uav_all_agent/geofence`，飛行中可隨時更新或清除；移除原本的部署參數 | **是**：沒有 GCS 設定時 Agent 不啟用圍籬 |
| 圍籬更新取消越界工作 | 新圍籬讓編隊槽位、航點或移動目標落在外側時，Agent 取消並原地懸停 | 建議顯示事件 403 與任務中止原因 15 |
| `PlannerStatus.state` 常數 | msg 新增 `STATE_IDLE/RUNNING/HOLD/ERROR`，介面文件 3.7 補上數值與 `reason` 分類 | 建議改用常數判斷大類 |
| 最小機間距離可設為 1.5 m | 部署可改為 1.5 m（預設仍 2.0 m，見第 6 節） | 維持 2 m 時不需修改 |
| 停滯偵測、編隊直接到位等規劃改善 | 不改變介面 | 否 |

**介面相容性**：

- 新增 msg：`GeofenceVertex`、`Geofence`、`GeofenceStatus`；新增 topic：`/uav_all_agent/geofence`、
  `/{agent_namespace}/geofence/status`。
- 既有 msg 只新增常數，CDR 排列不變：`PlannerStatus.STATE_*`、`WaypointMissionStatus.REASON_GEOFENCE_UPDATE=15`。
- 新增事件代碼 403 `PLANNER_GEOFENCE_TARGET_CANCELLED`。
- 舊版 QGCS 可以連線使用，但無法設定圍籬，Agent 就沒有場域保護。

## 2. 設定圍籬（QGCS 必須實作）

| 方向 | Topic | 型別 | QoS |
|---|---|---|---|
| QGCS → 全體 Agent | `/uav_all_agent/geofence` | `taiphoon_agent/Geofence` | reliable、transient_local、depth 1 |
| 各 Agent → QGCS | `/{agent_namespace}/geofence/status` | `taiphoon_agent/GeofenceStatus` | reliable、transient_local、depth 1 |

**發布內容**（`Geofence`）：

- `fence_id`：非 0 的版本號，每次修改遞增（QGCS 重啟後建議從 `GeofenceStatus.fence_id` 的最大值接續）。
- `action`：`ACTION_SET=1` 設定或替換；`ACTION_CLEAR=2` 停用。
- `margin_m`：機體中心與邊界的最小水平距離，0～10 m，建議預設 0.5 m。
- `vertices`：WGS84 頂點 3～64 點，依序相連、不可自交、不需重複第一點，每點距 Home 5 km 內。

**建議流程**：

1. 使用者在地圖上畫出多邊形並設定 margin，QGCS 先在本地檢查頂點數、自交與 margin 範圍。
2. 以新的 `fence_id` 發布 `ACTION_SET`。publisher 保持存在（transient_local），重啟或晚加入的 Agent 也能收到。
3. 等各機 `GeofenceStatus` 回報：
   - `state=STATE_ACTIVE` 且 `fence_id` 等於新版本 → 已生效。
   - `state=STATE_PENDING` → 該機 Home 尚未建立，建立後自動生效。
   - `last_request_id` 等於新版本且 `last_result` 為 `RESULT_REJECTED_INVALID`／`RESULT_REJECTED_TOO_FAR`
     → 拒絕，**該機保留原本的圍籬**；顯示 `detail` 並提示重新設定。
   - 數秒內沒有回報 → 該機可能離線或版本過舊。
4. 各機 `fence_id` 不一致時，在介面上明確標示（例如某機仍使用舊圍籬）。
5. 清除圍籬會失去場域保護，Agent 不另外確認；建議 QGCS 加上確認對話框。

**顯示**：

- 地圖上的圍籬以 `GeofenceStatus.vertices`／`margin_m` 為準（QGCS 重啟後也能還原），不要只用本地保存的內容。
- `position_inside=false` 表示該機目前在圍籬（含 margin）外，建議警告；Agent 不會自動飛回，
  之後只接受往內的移動。

## 3. 新的拒絕與狀態字串

以下字串建議在 `ui_format.cpp` 依 `detail`／`reason` 轉成中文提示：

| 來源 | 條件 | 英文字串 | 建議中文提示 |
|---|---|---|---|
| `AgentEvent` 400 `PLANNER_COMMAND_REJECTED`，`command` 為 `MOVE_*` | 單機相對移動目標在圍籬（含 margin）外 | `detail = "Goal outside geofence"` | 目標超出場域圍籬，已拒絕 |
| 同上，`command` 為 `MOVE_POINT` | 目標座標無效或在圍籬外 | `detail = "Goal invalid or outside geofence"` | 目標無效或超出場域圍籬，已拒絕 |
| 同上，`command` 為全體 `MOVE_*` | 移動後任一編隊槽位在圍籬外 | `detail = "Formation slot outside geofence"` | 編隊移動後會超出場域圍籬，已拒絕 |
| 同上，`command` 為 `FORMATION` | 新隊形任一槽位在圍籬外（由提案的 Agent 發布） | `detail = "Formation slot outside geofence"` | 隊形超出場域圍籬，已拒絕 |
| `WaypointMissionStatus` `REJECTED` | 上傳時任一航點在圍籬外 | `reason = REASON_OUT_OF_RANGE`、`detail = "waypoint outside geofence"`、`current_index` = 第一個越界航點 | 第 N 個航點超出場域圍籬 |
| `AgentEvent` 403 `PLANNER_GEOFENCE_TARGET_CANCELLED`，`command` 為 `FORMATION` | 圍籬更新後編隊槽位在外 | `detail = "Formation slot outside updated geofence; holding"` | 新圍籬使隊形越界，已解散並原地懸停 |
| 同上，`command` 為 `MISSION` | 圍籬更新後尚未完成的航點在外 | `detail = "Waypoint mission waypoint N outside updated geofence; holding"`（只載入未開始時結尾為 `loaded mission cleared`） | 新圍籬使第 N 個航點越界，任務已中止 |
| 同上，`command` 為 `MOVE` | 圍籬更新後移動目標在外 | `detail = "Goal outside updated geofence; holding"` | 新圍籬使目標越界，已原地懸停 |
| `WaypointMissionStatus` `ABORTED` | 圍籬更新使航點越界 | `reason = REASON_GEOFENCE_UPDATE`（15）、`detail = "waypoint N outside updated geofence"` | 圍籬更新，任務已中止 |
| `AgentEvent` `PLANNER_EMERGENCY_BRAKE` | 執行中軌跡重新驗證失敗且無法改走 | `detail` 含 `geofence_violation` | 軌跡將超出場域圍籬，緊急煞停 |
| `PlannerStatus.reason` | 圍籬內暫時沒有可行路徑（例如同儕擋住唯一通道），原地等待重試 | `local_goal_selection_failed` 或 `optimized_no_feasible_horizontal_path` | 等待可行路徑（場域受限） |
| `PlannerStatus.reason`（`state` 為 RUNNING） | 依進度重新規劃停滯，改為飛完目前安全軌跡（小場域、目標靠近圍籬時較常見） | `Progress stalled; following active trajectory to completion` | 進度停滯，沿目前路線飛往目標 |

- 上傳時的航點越界沿用 `REASON_OUT_OF_RANGE`；QGCS 若要區分「距離過遠」與「超出圍籬」，請判斷 `detail`。
- 命令階段的拒絕不改變 Agent 目前目標（維持原本的懸停或任務）。
- 等待狀態不是錯誤：同儕讓開後 Agent 會自動繼續，QGCS 不需要重送命令。
- 停滯字串會持續到 Agent 換成新軌跡為止（通常 0.5～2 秒），之後回到 `ok`；一次移動可能出現數次，
  都不是錯誤，建議以一般提示顯示，不要跳出警告。
- `Emergency brake: target reached while moving`（`state` 為 HOLD）表示飛機到位時仍在移動，
  Agent 原地煞停後進入懸停；小場域幾乎每次到位都會出現。建議 QGCS 對這個字串降低警示等級，
  與真正的避碰煞停（`optimized_emergency_brake: ...`）分開顯示。
- `PlannerStatus.state` 各數值與 `STATE_HOLD` 的 `reason` 分類見介面文件 3.7。

## 4. 圍籬更新時的 Agent 行為

| 情況 | Agent 處理 | QGCS 顯示建議 |
|---|---|---|
| 清除圍籬 | 只停用，目前工作照常 | 地圖移除圍籬 |
| 編隊中任一槽位在新圍籬外 | 全體解散編隊、原地懸停，事件 403 | 提示重新下達隊形 |
| 航點任務尚未完成的航點在外 | 任務中止（原因 15）並懸停；未開始的任務直接清除 | 提示修改航點後重新上傳 |
| 單機移動目標在外 | 取消目標並懸停，事件 403 | 提示重新下達目標 |
| 目標仍在內，但目前軌跡越界 | 自動重新規劃，不煞停 | 不需提示 |
| 飛機已在新圍籬外 | 不自動飛回；之後只接受往內的移動 | `position_inside=false` 時警告 |

## 5. GuardModule 與輸入範圍

- `app_settings.cpp` 的 `V_SHAPE_SPACING` 下限 2 m 對應 Agent `formation_min_distance_m`。部署改為 1.5 m 時，
  請把下限同步改為 1.5 m，否則 QGCS 會擋下 Agent 可接受的隊形；仍維持 2 m 時不需修改。
- `CIRCLE_RADIUS` 下限 1 m 不變。Agent 以槽位中心距離檢查：N 台環形的相鄰距離為 `2·R·sin(π/N)`，
  小於 `formation_min_distance_m` 時拒絕（既有行為）。
- 圍籬生效後，建議 QGCS 在送出前預先檢查：
  - 單機 `MOVE_POINT`、航點：目標點到圍籬邊界的距離 ≥ margin。
  - 全體移動與編隊：中心加上每個槽位偏移都要在圍籬內。
  預檢只是改善使用者體驗，最終仍以 Agent 判定為準。

## 6. 1.5 m 最小間距

開發者已同意小場域可將最小機間距離降為 1.5 m（目標是不在空中相撞）。Agent 端以
`EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M=1.5`、`EGO_SWARM_FORMATION_MIN_DISTANCE_M=1.5` 設定。

目前狀態（尚未改成預設值，Agent 預設仍為 2.0 m）：

- 設圍籬的 8×8 m 場域：編隊全部通過，交叉測試不再逾時；交叉時偶有實測最小機間距離略低於設定值
  （1.5 m 設定約 1.49 m、2.0 m 設定約 1.90 m），發生在交叉機繞過中央懸停機時，原因是 PX4 追蹤誤差；
  開發者已接受此誤差。
- planner 只保證計畫軌跡不低於設定值，不另外預留 PX4 追蹤誤差。因此 QGCS 暫時不要把 1.5 m 當成一般預設值，
  `V_SHAPE_SPACING` 下限維持 2 m。

## 7. QGCS 驗證建議

啟動三機 Agent 與 SITL 後：

1. 設定圍籬 → 三台 `GeofenceStatus` 都為 ACTIVE、`fence_id` 一致，地圖顯示圍籬。
2. 送出自交多邊形 → 顯示拒絕原因，各機仍保留原圍籬。
3. 單機 `MOVE_POINT` 到圍籬外 → 顯示「目標超出場域圍籬」，飛機不動。
4. 環形編隊後全體向圍籬方向移動，移動距離足以讓槽位越界 → 顯示編隊越界提示，隊形不變。
5. 上傳含圍籬外航點的任務 → 顯示第 N 個航點越界。
6. 飛行中縮小圍籬讓目前目標落在外側 → 收到事件 403，飛機原地懸停。
7. 清除圍籬 → 三台回報 DISABLED，之後可飛往原本被擋下的位置。
8. 重新啟動 QGCS → 由 `GeofenceStatus` 還原地圖上的圍籬。
