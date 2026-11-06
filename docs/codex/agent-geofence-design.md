# Agent 地理圍籬（Geofence）設計

## 背景與需求

- 實際場域為 8×8 公尺，中心約 24.881051576424703, 121.0696306771756，長軸方位 143°，三機起飛航向約 53°。
- 舊版 PX4 實測：三機編隊恢復時機體中心最遠 3.25 m（未越界），但 planner 發布的軌跡在避讓同儕時
  曾延伸到長軸 4.0～4.6 m（`formation-failed-1-164241`、`formation-failed-3-164501`）。
  原因是 planner 不知道場域邊界：Local Goal、A*、最佳化器與 ContinuousSafetyValidator 只考慮同儕與障礙物。
- 人類開發者決定：Agent 新增可設定的場域多邊形，規劃器以邊界為硬限制，Local Goal、A* 與軌跡都不得超出，
  超出時拒絕或改走他側。圍籬範圍需由地面站設定，不在啟動時固定。

## 範圍

- 只限制水平位置（North／East）。高度沿用既有 TAKEOFF／ALTITUDE 與 PX4 限制，本次不新增限高。
- 圍籬是 Agent planner 的規劃約束，不取代 PX4 自身 geofence（`GF_*`），也不寫入 PX4 參數。
- 圍籬只由地面站在執行期設定（2026-11-06 人類開發者決定）：移除啟動參數 `geofence_polygon`、
  `geofence_margin_m` 與對應環境變數；Agent 啟動時沒有圍籬，直到收到地面站的設定。
  飛行中可以隨時更新或清除。

## 介面

| 方向 | Topic | 型別 | QoS |
|---|---|---|---|
| 地面站 → 全體 Agent | `/uav_all_agent/geofence` | `taiphoon_agent/Geofence` | reliable、transient_local、depth 1 |
| Agent → 地面站 | `/{agent_namespace}/geofence/status` | `taiphoon_agent/GeofenceStatus` | reliable、transient_local、depth 1；狀態改變時發布 |

`Geofence`（地面站命令）：

| 欄位 | 說明 |
|---|---|
| `stamp` | 發布時間（只供日誌） |
| `fence_id` | 地面站指定的版本號，不得為 0；建議每次修改遞增 |
| `action` | `ACTION_SET=1` 設定或替換；`ACTION_CLEAR=2` 停用 |
| `margin_m` | 機體中心與邊界的最小水平距離（m），有限值且 0～10；CLEAR 時忽略 |
| `vertices` | `GeofenceVertex[]`（WGS84 `latitude_deg`、`longitude_deg`），3～64 點，依序相連、不需重複第一點；CLEAR 時忽略 |

`GeofenceStatus`（每台 Agent 回報）：

| 欄位 | 說明 |
|---|---|
| `state` | `STATE_DISABLED=0` 無圍籬；`STATE_PENDING=1` 已接受、等待 planner Home 建立；`STATE_ACTIVE=2` 生效 |
| `fence_id`、`margin_m`、`vertices` | 目前生效（或等待中）的圍籬內容；DISABLED 時為 0／空。地面站重啟後可據此還原顯示 |
| `last_request_id` | 最近一次處理的命令 `fence_id` |
| `last_result` | `RESULT_NONE=0`、`RESULT_ACCEPTED=1`、`RESULT_REJECTED_INVALID=2`（格式、頂點數、margin、自交、面積過小）、`RESULT_REJECTED_TOO_FAR=3`（頂點距 Home 超過 5 km） |
| `position_inside` | 目前位置是否在圍籬（含 margin）內；沒有圍籬或沒有位置時為 true |
| `detail` | 英文補充說明 |

- 拒絕時保留原本的圍籬（或維持無圍籬），不進入 fail closed。
- 同一 `fence_id` 且內容相同的命令（transient_local 重送）只更新 `last_request_id`，不重新套用。
- 新增事件 403 `PLANNER_GEOFENCE_TARGET_CANCELLED`（WARNING）：圍籬更新讓目前目標、編隊槽位或航點落在圍籬外，
  Agent 已停止並原地懸停。`WaypointMissionStatus` 新增 `REASON_GEOFENCE_UPDATE=15`。
- 頂點依序連成簡單多邊形（凹多邊形可，不可自交）；每個頂點距 planner Home 不得超過 5 km（避免誤填座標）。
- 最佳化器另以固定 0.5 m 軟性緩衝（`GEOFENCE_OPTIMIZER_SOFT_MARGIN_M`）把軌跡推離邊界，硬性門檻仍是 `margin_m`。

## 資料流

1. `geofence_cb` 收到命令後，先以 pure C++ `ego_swarm::GeofenceUpdatePolicy`（`src/ego_swarm/GeofenceUpdate.hpp`）
   檢查 `fence_id`、`action`、margin 與頂點（數量、有限值、緯經度範圍）。
2. planner Home 已凍結時，以同一 `LocalNedFrame` 把頂點轉成 planner NED (N, E)，建立 `ego_swarm::Geofence`
   （`src/ego_swarm/Geofence.hpp`），檢查不自交、面積 > 1 m²、距 Home 5 km 內；全部通過才替換目前圍籬。
   Home 尚未凍結時暫存為 PENDING，凍結當下再建立；建立失敗回報 REJECTED 並維持無圍籬。
   PX4 重啟只重設 PX4 transform，planner Home 不變，因此不重建圍籬。
3. 套用（SET 成功或 CLEAR）後立即處理正在進行的工作（見「執行期更新」），再發布 `GeofenceStatus`。
4. `Geofence` 提供 `clearance_m(position)`：在多邊形內為「到最近邊的距離」（正），在外為負值。
   `contains_with_margin(position)` 等同 `clearance_m ≥ margin`。
5. 規劃各階段以同一個 `const Geofence*` 取得邊界（停用時為 nullptr，所有檢查視為通過）：

| 階段 | 位置 | 行為 |
|---|---|---|
| 命令受理 | 單機 MOVE_*、MOVE_POINT、NED target、mission intent | 目標在圍籬（含 margin）外 → `PLANNER_COMMAND_REJECTED`，detail `goal outside geofence` |
| 命令受理 | 全體 MOVE | 新中心加上每個槽位偏移任一在外 → 拒絕 |
| 命令受理 | FORMATION_* 提案 | 任一槽位在外 → 拒絕提案 |
| 命令受理 | 航點任務 | 任一航段目標在外 → 任務拒絕，沿用 `REASON_OUT_OF_RANGE`（不改 msg），detail `waypoint outside geofence`，附航點索引 |
| 執行 | 遠端 epoch 指派給自己的槽位 | 在外 → 不前往，狀態 `Formation slot outside geofence` 並原地 hold |
| Local Goal | `local_goal_endpoint_safe` | 候選點不在圍籬內 → 視為不安全，改選他側或縮短 |
| A* | `SpatiotemporalRouteInitializer` | 圍籬外格點視為阻擋 |
| 最佳化器 | `TrajectoryObjective` | 取樣點低於 margin＋0.5 m 加軟性成本、低於 margin 加硬性成本；硬性接受條件同步納入 |
| 安全驗證 | `ContinuousSafetyValidator` | 每個取樣點檢查；不足時回傳 `GEOFENCE_VIOLATION` |
| 直接到位 | terminal snap | 目前位置到目標的直線段需全程在圍籬內，否則改走一般規劃 |

6. 執行中的 active plan 重驗證得到 `GEOFENCE_VIOLATION` 時，比照同儕／障礙物：作廢並重新規劃，不直接煞車。
7. 小場域的最佳化器補強（未設定圍籬時同樣生效，但只在原本會判定失敗的情況介入）：
   - seed 退回：solver 無法產生合格解時，若 seed 通過全部 hard gate、動力學可行，
     且同儕距離至少保有 `peer_collision_min_distance_m + peer_planning_margin_m`，改用 seed。
   - 時間拉長：solver 的繞行路線已通過同儕、障礙物與圍籬 hard gate，但沿用 seed 的總時間會超出
     速度／加速度上限時（例如同儕與圍籬之間的窄通道），保留路線形狀，依序把 knot interval
     拉長為 1.25、1.5、1.75、2 倍；每個倍率都以 safety sampling 重新評估全部 hard gate 與動力學。

已知限制：目標或起點落在軟性緩衝帶（margin＋0.5 m）內時，最佳化器可能把軌跡前段壓在起點附近、
最後才衝向終點；Node 依進度（預設 30%）重新規劃時每次從頭開始，飛機會停在原地（8×8 m 交叉測試約 4 次出現 1 次，
無安全問題）。曾嘗試把軟性門檻上限設為端點 clearance，實測機間距離低於設定值 0.02 m，因此撤回；
停滯改由 Node 端的停滯偵測（`ProgressStallMonitor`）處理，實作後交叉測試不再逾時。

撤回後的測試仍出現同樣的低於設定值（1.5 m 設定實測 1.496 m、2.0 m 設定實測 1.91 m），時機都在交叉開始後
2.5 秒內、交叉機繞過中央懸停機時，與上限修正無關。原因是 planner 只保證計畫軌跡不低於
`peer_collision_min_distance_m`，沒有替 PX4 追蹤誤差（轉彎時切內側）預留空間；尚未處理。

## 執行期更新

圍籬替換或清除後，Node 依下列順序處理目前工作。所有 Agent 收到同一份圍籬、用同一套規則判斷，
編隊的處理結果因此一致；各機獨立回報 `GeofenceStatus`。

| 情況 | 處理 |
|---|---|
| CLEAR | 只停用圍籬；目前目標、編隊與任務照常執行 |
| 編隊中，任一槽位（中心＋偏移）在新圍籬外 | 全體各自解散編隊、原地懸停（`dissolve_formation_to_hold`），發布事件 403 |
| 航點任務 LOADED／RUNNING／DWELLING，尚未完成的任一航點在新圍籬外 | 任務中止（`REASON_GEOFENCE_UPDATE`），執行中則煞停並原地懸停，發布事件 403 |
| 一般目標（MOVE_*、MOVE_POINT、NED target）在新圍籬外 | 取消目標、煞停並原地懸停，發布事件 403 |
| 目標仍在圍籬內，但執行中的軌跡越界 | 下一個 planner 週期重驗證得到 `GEOFENCE_VIOLATION`，作廢並重新規劃（既有行為） |
| 目前位置已在新圍籬外 | 原地懸停時不強制移動；之後的軌跡只允許往內（既有起點放寬規則），`position_inside=false` 提醒地面站 |
| 已落地或尚未起飛 | 只更新圍籬，之後的命令以新圍籬檢查 |

- 原地懸停的位置可能在新圍籬外；不自動飛回，由操作人員下達往內的移動命令，避免在擁擠場域自動移動。
- 清除圍籬會失去場域保護；Agent 照常接受，由地面站負責確認流程。

## 失效情境

| 情境 | 處理 |
|---|---|
| 地面站從未設定 | 無圍籬，行為與未加入圍籬前相同 |
| 命令格式錯誤、頂點數不足、自交、離 Home 過遠 | 拒絕（`last_result`），保留原本的圍籬；不 fail closed |
| 地面站離線 | Agent 保留最後一次生效的圍籬；地面站重新上線後以 `GeofenceStatus` 還原顯示 |
| Agent 重啟 | 從 transient_local 取得地面站最後發布的命令；地面站也離線時無圍籬，`GeofenceStatus` 為 DISABLED |
| 起飛點或目前位置已在圍籬外／margin 帶內 | 驗證器把該次規劃的門檻放寬為 `min(margin, 起點 clearance − 0.05 m)`：允許往內修正，不允許比起點更外側；新目標仍須在圍籬內 |
| 同儕擋住唯一的圍籬內路徑 | 規劃失敗 → 既有 waiting hold，待同儕移動後重試；不會為了避讓而出界 |
| 緊急煞車 | 不經圍籬驗證（必須優先停下）；煞車距離約 v²/(2a)，以 margin 吸收，文件註明 margin 應 ≥ 預期煞車距離 |
| 同儕解衝突（peer deconfliction）逃逸 | 逃逸目標也須在圍籬內；找不到時維持既有 hold |
| 部分 Agent 拒絕或未收到命令 | 各 Agent 只以自己的圍籬檢查自己的目標與軌跡；地面站比對各機 `GeofenceStatus.fence_id` 找出不一致 |

## 驗證方式

1. 單元測試（`test/`）：
   - `Geofence`：解析、凸／凹多邊形內外判斷、clearance 正負號、自交與退化輸入 fail closed。
   - `ContinuousSafetyValidator`：越界取樣回傳 `GEOFENCE_VIOLATION`；起點在 margin 帶內時允許向內。
   - `LocalGoalSelector`：圍籬外候選點被排除。
   - `OptimizedEgoPlanner`：同儕擋在直線上、單側為圍籬時，軌跡改走圍籬內側且全程 clearance ≥ margin。
   - `OptimizedEgoPlanner`：1.5 m 間距、場域長軸 0° 與 143° 時，同儕與圍籬之間的窄通道仍可規劃出合格繞行。
   - `GeofenceUpdatePolicy`：命令檢查、重複命令、PENDING、拒絕保留舊圍籬，以及更新後目標／槽位／航點的取消判斷。
   - 未設定圍籬時，既有測試行為不變。
2. 舊版 PX4 SITL 8×8 m 場域三機測試（失效機 uav_1、uav_2、uav_3），monitor 起飛前發布場域四角加 margin 0.5 m 的圍籬，
   並等三台 `GeofenceStatus` 都為 ACTIVE；
   monitor 的 `trajectory_boundary_violation` 不得再出現，最小機間距 ≥ 2.0 m。
3. 執行期更新 SITL：飛行中縮小圍籬讓目標落在外側，確認煞停懸停、事件 403 與 `GeofenceStatus`；再清除圍籬確認可繼續移動。
4. 4 機標準回歸（未設定圍籬）確認沒有退化。
