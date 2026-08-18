# EGO-Swarm 滾動時間視窗局部軌跡規劃開發規格

## 文件狀態

- 狀態：開發參考規格
- 日期：2026-08-18
- 適用範圍：`src/ego_swarm/`、`src/nodes/ego_swarm_planner_node.*`、
  `src/controllers/BasicController.*`、`src/perception/` 與相關測試
- 本規格不修改現有 ROS2 自訂訊息格式
- 本規格不重新啟用 legacy `MissionPlanner` 或 `TrajectoryController`

## 1. 目的

將目前一次規劃至最終目標的行為，改成以固定未來時間為範圍的滾動局部規劃：

1. 任務期間保留不變的最終目標。
2. 每次只產生一條未來有限時間內可執行的局部 B-spline 軌跡。
3. 局部終點由可通行候選空間選擇，不直接沿最終目標方向截斷。
4. 以週期與安全事件觸發重新規劃。
5. 每個 Agent 對外只廣播目前實際交付 BasicController 的單一局部軌跡。
6. 多個局部軌跡依序推進，直到 UAV 實際到達最終目標。
7. 無法持續產生安全軌跡時，使用既有 emergency braking trajectory 安全煞停。

## 2. 不在本次範圍

- 不修改 `msg/AgentTrajectory.msg`、`msg/PlannerStatus.msg` 或其他 `.msg/.srv/.action`。
- 不新增第二個 PX4 `TrajectorySetpoint` publisher。
- 不讓 Planner Node 直接發布 PX4 setpoint。
- 不要求新舊局部軌跡滿足嚴格的速度或加速度連續性。
- 不建立 active/pending 雙軌跡廣播協定。
- 不建立新的 Formation State Machine。
- 不以 Z 軸位移繞過障礙物或其他 UAV。
- 不提供無限距離或無限時間的路徑搜尋。

## 3. 統一名詞

### 3.1 最終目標（Mission Goal）

GCS、MOVE 命令或 Formation assignment 產生的真正任務目標。完成任務前不得因局部
規劃失敗、等待或 emergency brake 而被目前位置覆寫。

### 3.2 局部軌跡（Trajectory Segment）

單次規劃、驗證、交付 BasicController 並透過既有 `AgentTrajectory` 廣播的時間化
B-spline。每條局部軌跡都有新的單調遞增 `trajectory_id`。

### 3.3 局部終點（Local Endpoint）

單條局部軌跡的終點。局部終點只表示本輪可安全執行的進度，不表示任務已完成。

### 3.4 規劃時間視窗（Planning Horizon）

從本次 planning snapshot 起算，規劃器要描述的未來時間範圍。初始預設值：

```text
planning_horizon_s = 5.0
```

有效可達距離由當前狀態、最大速度、最大加速度、最大 jerk 與路徑限制推導，不能把
`planning_horizon_s * max_velocity_mps` 直接視為必然可達距離。

### 3.5 任務終點佔位（Mission Terminal Hold）

UAV 已實際到達最終目標後發布的固定佔位軌跡，沿用既有
`AgentTrajectory.terminal_hold=true` 語意。

### 3.6 重新規劃等待（Replanning Hold）

尚未抵達最終目標，但目前沒有可發布移動軌跡時的暫時固定佔位。它必須使用有限時間、
重複控制點且 `terminal_hold=false` 的合法局部軌跡表示，不得被 Formation 或其他
Agent 誤判為任務完成。

## 4. 核心行為規格

### 4.1 最終目標與局部終點分離

`PlanningSnapshot` 必須同時保有：

- 不變的 Mission Goal。
- 本輪由 planner implementation 選出的 Local Endpoint。

Mission Goal 用於衡量候選是否朝任務方向前進；Local Endpoint 才是本輪 B-spline 的
實際終點。ContinuousSafetyValidator 必須驗證整條將要發布的 Trajectory Segment，
不得只驗證前綴後仍發布通往 Mission Goal 的完整未驗證軌跡。

### 4.2 Local Endpoint 選擇

Local Endpoint 必須由本輪時間視窗內的可達候選空間選擇。候選評分至少包含：

- 朝 Mission Goal 的有效進展。
- 已知障礙物 clearance。
- 同儕未來軌跡 separation。
- 路徑平滑度與動力學可行性。
- 與目前移動方向的一致性，降低左右反覆切換。
- 未觀測空間的 soft penalty。

已知落在障礙物 clearance 或同儕碰撞距離內的候選必須拒絕。未觀測候選可以保留，
但不能獲得與已觀測 free space 相同的成本，也不能繞過第 7 節的動態安全條件。

若 Mission Goal 已落在時間視窗的可達範圍內，Local Endpoint 應直接設為 Mission Goal，
產生最後一條局部軌跡。

### 4.3 水平路徑限制

本功能的 MOVE 與 Formation transition 局部軌跡只允許在 XY 平面繞行：

- 使用 `PathConstraint::FIXED_Z`。
- 所有 B-spline 控制點必須維持相同 Z。
- optimizer、候選產生器及任何 recovery path 都不得加入 above/below candidate。
- PX4 實際高度的微小 tracking error 不等於 planner 使用 Z 軸繞行；驗收以發布軌跡
  的控制點 Z span 為主要判據。

### 4.4 週期觸發

局部軌跡執行時間比例達以下預設值時，開始計算下一條局部軌跡：

```text
replan_trigger_ratio = 0.70
```

70% 是重新規劃觸發點，不是無條件切換點。新軌跡只有在 planner success 且通過
ContinuousSafetyValidator 後才能取代目前軌跡。剩餘 30% 是規劃、驗證、DDS 傳輸與
失敗重試餘裕。

### 4.5 事件觸發

下列事件不得等待 70% 週期點：

- 新障礙物使目前軌跡不再安全。
- 同儕軌跡更新後與目前軌跡發生時間對齊衝突。
- live AgentState 顯示 UAV 進入 peer collision warning distance。
- 目前軌跡偏離有效範圍或取樣失敗。
- obstacle map、peer trajectory、frame 或自身狀態變成 stale/invalid。

事件發生後先重新驗證目前軌跡。若仍安全，可保留目前軌跡並開始重規劃；若已不安全，
立即撤銷移動軌跡並進入 emergency brake，不等待連續失敗次數。

為避免同一批 PointCloud 或 peer heartbeat 造成過度規劃，implementation 可加入
`minimum_replan_interval_s`，但不得延後已確認 unsafe 的 emergency brake。

### 4.6 單一 active segment

系統任何時間只維護與廣播一條目前實際執行的 Trajectory Segment：

1. 新 segment 以最新 planning snapshot 的自身位置為第一控制點。
2. 新 segment 完成規劃與 safety validation 前，不取代舊 segment。
3. 通過後立即成為 active segment、增加 `trajectory_id` 並使用目前時間作為
   `start_time` 發布。
4. BasicController 沿用現有「新合法軌跡取代舊軌跡」行為。
5. 不預先發布 pending segment。

不要求新舊 segment 的速度或加速度連續，但每條新 segment 自身仍必須滿足現有速度、
加速度、jerk hard limits，且所有輸出必須為有限值。

### 4.7 最終到達判斷

不得因 UAV 到達 Local Endpoint 就清除 Mission Goal。只有同時滿足以下條件，才可視為
任務完成並發布 Mission Terminal Hold：

1. Local Endpoint 與 Mission Goal 的距離小於 global arrival tolerance。
2. UAV 實際位置與 Mission Goal 的距離小於對應 local/global arrival tolerance。
3. 一般 MOVE 任務的實際速度低於既有停止門檻。

Formation 到位判斷保留現有 FormationEpoch、slot occupancy 與 convergence contract，
但中途 Local Endpoint 不得發布 `terminal_hold=true`。

## 5. 不修改 AgentTrajectory 的跨 Segment 優先權

### 5.1 決策

本功能不新增 `route_id` 或 `priority_stamp`。移動軌跡之間的固定通行優先權改用既有
`AgentTrajectory.agent_index`：

```text
較小 agent_index 具有較高通行優先權
```

因此同一 Agent 不論產生多少個局部 segment，其優先權都保持不變，不受每段
`start_time` 或 `trajectory_id` 影響。

### 5.2 安全規則

- `terminal_hold=true` 永遠表示實體佔位，不得因 Agent ID priority 忽略。
- 路線尚未開始前的 peer 目前位置仍須視為佔位。
- Priority 只能解決兩條已驗證 moving segment 的時間／空間仲裁，不得放行低於
  `peer_collision_min_distance_m` 的軌跡。
- 同一 Agent 的 `trajectory_id` 仍必須單調遞增，舊封包不得覆寫新 segment。
- live AgentState collision gate 與 ContinuousSafetyValidator 仍是最終安全門檻。

### 5.3 已知限制

固定 Agent ID priority 可能使較大 ID 的 Agent 較常等待。Implementation 必須記錄
連續讓路時間與朝 Mission Goal 的進展；超過 progress timeout 時只能回報無進展、
重新選擇 Local Endpoint 或安全煞停，不得為消除飢餓而突破碰撞距離。

## 6. Formation Reservation

- Reservation 只檢查目前要發布的 Trajectory Segment。
- 不保留超出目前 segment 的未來 reservation。
- FormationEpoch、persistent assignment、safe transition order 與 convergence contract
  保持不變。
- 每次新 segment 都必須重新通過 reservation 與 ContinuousSafetyValidator。
- 若 reservation 失敗但目前 segment 仍安全，繼續目前 segment 並重試。
- 若 reservation 失敗且已到最晚安全煞停時點，執行 emergency brake。

## 7. 未觀測空間與動態安全

允許 planner 對目前感測範圍外的候選方向進行規劃，但「可規劃」不代表可以假設該區域
沒有障礙物。障礙物 map seam 應能區分：

- observed free
- observed occupied
- unknown

在 UAV 任一執行時刻，沿 active segment 的前方已知可通行距離應滿足：

```text
known_free_distance
>= stopping_distance
 + sensor_latency_distance
 + planning_and_dds_latency_distance
 + control_reaction_margin
```

現階段不得移除上述煞停與延遲餘裕。未來若測試證明可縮減，必須提出替代安全保證、
風險與驗證證據，再由人類開發者確認。

若障礙物 map 尚未能提供 coverage 查詢，第一版可使用 sensor range/FOV 與 UAV pose
保守推導 observed region；不得把 PointCloud 中沒有點直接解釋為全域 free space。

## 8. 規劃失敗與煞停

初始預設：

```text
max_consecutive_planning_failures = 3
```

行為：

1. 新 segment 規劃失敗，但目前 segment 仍安全且仍有足夠剩餘時間：繼續舊 segment
   並重試。
2. 連續三次失敗：產生 emergency braking trajectory。
3. 目前 segment 已不安全、資料 stale/invalid，或剩餘距離／時間不足以安全煞停：
   不等待三次，立即 emergency brake。
4. Emergency brake 必須根據目前速度與加速度產生有限 jerk 煞車軌跡，不得用重複目前
   位置假裝瞬間停止。
5. 煞停完成但尚未抵達 Mission Goal：發布 Replanning Hold，保留 Mission Goal 並持續
   嘗試重新規劃。

「停在原地」在本規格中定義為停在 emergency braking trajectory 的安全終點，不保證
等於觸發煞車瞬間的位置。

## 9. 大型與 U 形障礙物

Local Endpoint 搜尋與 optimizer 必須有高但有限的上限。建議初始參數：

```text
max_endpoint_search_radius_m = 50.0
no_progress_timeout_s = 30.0
minimum_progress_m = 0.5
```

`max_endpoint_search_radius_m` 是候選搜尋上限；單一 segment 的實際終點仍受
`planning_horizon_s` 與動力學可達範圍限制。

為降低單一直線 seed 在 U 形障礙物內陷入 local minimum 的風險，候選產生器應至少能
建立不同水平拓撲方向，例如直行、左繞與右繞。每個候選仍受 optimizer wall-time、
iteration budget、XY-only 與共同 safety validator 約束。

若在 `no_progress_timeout_s` 內無法取得至少 `minimum_progress_m` 的淨 Mission Goal
進展，回報 unreachable/no-progress，並進入安全煞停或 Replanning Hold；不得無限搜尋。

## 10. 建議 module 與 interface

### 10.1 Pure C++ rolling-horizon module

滾動規劃生命週期應留在 `src/ego_swarm/` 的 pure C++ 深層 module。建議對 Node 只暴露
一個決策 interface，例如：

```cpp
RollingPlanDecision update(const RollingPlanningSnapshot& snapshot);
```

module implementation 內部負責：

- 70% 週期觸發。
- 事件觸發與 debounce。
- Local Endpoint 選擇。
- 呼叫 `ITrajectoryPlanner`。
- 呼叫共同 safety validator。
- 失敗計數與 progress watchdog。
- 保留舊 segment、發布新 segment、Replanning Hold 或 emergency brake 的決策。
- Mission Goal 完成判斷。

### 10.2 Node 責任

`EgoSwarmPlannerNode` 只負責：

- ROS2 message validation 與轉換。
- 收集 immutable planning snapshot。
- timer 與 callback 事件轉譯。
- 呼叫 rolling-horizon module。
- 發布 module 已決定的單一 AgentTrajectory 與 PlannerStatus。

主要路徑演算法、失敗計數與狀態轉移不得散落在 ROS2 callback 中。

### 10.3 保留既有 seam

- `ITrajectoryPlanner` 繼續作為 trajectory backend seam。
- `OptimizedEgoPlanner` 產生局部 B-spline 候選。
- `ContinuousSafetyValidator` 仍是所有 planner output 的最終 safety gate。
- `EmergencyBrakePlanner` 仍負責動態可行煞車軌跡。
- `BasicController` 仍是唯一 PX4 `TrajectorySetpoint` publisher。
- `IObstacleMap` 保持 sensor-agnostic；coverage/unknown 語意應在此 seam 擴充。

## 11. 參數

新增參數建議：

| 參數 | 初始預設值 | 說明 |
|---|---:|---|
| `planning_horizon_s` | `5.0` | 單次局部規劃的未來時間視窗 |
| `replan_trigger_ratio` | `0.70` | active segment 執行至此比例時觸發重規劃 |
| `minimum_replan_interval_s` | `0.20` | 非 unsafe 事件的最短重規劃間隔 |
| `max_consecutive_planning_failures` | `3` | 目前 segment 仍安全時允許的連續失敗次數 |
| `max_endpoint_search_radius_m` | `50.0` | Local Endpoint 候選搜尋的空間上限 |
| `no_progress_timeout_s` | `30.0` | 無有效 Mission Goal 進展的最長時間 |
| `minimum_progress_m` | `0.5` | progress watchdog 的最小有效進展 |

既有參數繼續生效，包含：

- `peer_collision_min_distance_m`
- `obstacle_clearance_m`
- `max_velocity_mps`
- `max_acceleration_mps2`
- `max_jerk_mps3`
- `peer_trajectory_timeout_s`
- `obstacle_cloud_timeout_s`
- `planning_rate_hz`

## 12. 失效情境

| 情境 | 必要行為 |
|---|---|
| obstacle map missing/stale/invalid | 不建立新移動 segment；目前路徑不安全或煞停餘裕不足時 emergency brake |
| peer trajectory missing/stale/invalid | fail-closed；不得假設空中沒有 peer |
| planner timeout/nonfinite/infeasible | 記錄失敗；仍安全時重試，達門檻或餘裕不足時煞停 |
| 新障礙物切入 active segment | 事件觸發；unsafe 時立即煞停 |
| 新 peer segment 與 active segment 衝突 | 依 Agent ID priority 重規劃；不得突破碰撞距離 |
| Local Endpoint 到達但 Mission Goal 未到達 | 不發布 terminal hold；產生下一段或 Replanning Hold |
| no progress / U 形死路 | bounded topological candidates；超時後安全煞停並回報 unreachable |
| trajectory segment 到期且無替代 segment | endpoint 低速時 Replanning Hold；仍有顯著速度時 emergency brake |
| NaN/Inf 或 frame/time 不一致 | 拒絕輸入或輸出，保留最後有限安全 setpoint並進入保守降級 |

## 13. 驗收計畫

正式功能驗收只使用以下兩個情境。所有 ROS2 與 PX4 SITL 操作必須依專案規範在 Docker
環境內執行；實作階段仍須依根目錄 `AGENTS.md` 補齊必要單元測試，但不另外增加本規格
的功能驗收情境。

### 13.1 機群避障功能 1

#### 測試流程

1. 快速起飛；機群應呈現由西到東的水平一字型排列。
2. 對 UAV1 發送 `YAW 90`，使機頭向東。
3. 對 UAV1 發送 `MOVE_FORWARD 20`。

#### 達成條件

- UAV 飛行期間任兩架飛機中心距離低於設定間距的單次連續時間不得超過 0.5 秒；設定
  間距預設為 3 m。
- UAV1 所有已發布移動 Trajectory Segment 只能在 XY 平面移動；控制點 Z span 不得
  超過 0.05 m，且不得產生 Z 軸繞行候選。
- 從 `MOVE_FORWARD 20` 命令被接受起，到 UAV1 距離目標 1 m 內且完成終點 hold 的時間
  必須小於 20 秒。

### 13.2 機群避障功能 2

#### 測試流程

1. 快速起飛；機群應呈現由西到東的水平一字型排列。
2. 執行環形編隊，直徑 10 m、高度 10 m。現有命令第一個參數是半徑，因此發送
   `FORMATION_CIRCLE 5 10`。
3. 等待所有 UAV 回報 formation converged 並移動到位。
4. 發送 `FORMATION_CANCEL`。
5. 對 UAV1 發送 `MOVE_FORWARD 5`，等待到位。
6. 對 UAV1 發送 `MOVE_RIGHT 20`。

#### 達成條件

- UAV 飛行期間任兩架飛機中心距離低於設定間距的單次連續時間不得超過 0.5 秒；設定
  間距預設為 3 m。
- UAV1 所有已發布移動 Trajectory Segment 只能在 XY 平面移動；控制點 Z span 不得
  超過 0.05 m，且不得產生 Z 軸繞行候選。
- 從 `MOVE_RIGHT 20` 命令被接受起，到 UAV1 距離目標 1 m 內且完成終點 hold 的時間
  必須小於 20 秒。

### 13.3 驗收量測

既有 `AutoTest/scripts/ego_swarm_acceptance_monitor.py` 可作為量測基礎，但實作時必須
同步調整其 mode 1：

- 將既有 `MOVE_FORWARD 30`／`MOVE_BACKWARD 30` 流程改為本規格的單次
  `MOVE_FORWARD 20`。
- 保留 `minimum_distance_m=3.0`、`max_short_violation_s=0.5`、XY-only control-point
  Z span 與 20 秒移動時間量測。
- mode 2 使用 `FORMATION_CIRCLE 5 10`，並保留 `MOVE_FORWARD 5`、
  `MOVE_RIGHT 20` 流程。

## 14. Definition of Done

本功能只有在以下條件全部成立時才可視為完成：

- Mission Goal 與 Local Endpoint 的生命週期及到達語意符合本規格。
- 規劃器只發布有限時間的局部 Trajectory Segment。
- 70% 週期與安全事件都能觸發重規劃。
- 每個 Agent 任何時間只廣播目前實際執行的單一 segment。
- 未修改任何現有 ROS2 自訂訊息格式。
- 跨 segment priority 只依 stable Agent ID，且所有 safety gate 維持有效。
- 中途 Local Endpoint、Replanning Hold 與 Mission Terminal Hold 不會互相誤判。
- 規劃失敗、stale data、unknown space 與 no-progress 都有明確 fail-closed 行為。
- 全部新增／修改的單元測試通過。
- 第 13.1 與 13.2 兩項 Docker/PX4 SITL 功能驗收通過。
- README、`docs/taiphoon-agent-interface.md` 與相關架構／行為文件已同步。

## 15. Codex 實作前回報要求

Codex 開始修改程式前，必須先依本規格回報：

1. 預計修改檔案。
2. 是否影響現有 module seam。
3. 不修改 ROS2 自訂訊息的確認。
4. Mission Goal、Local Endpoint、active segment 與 hold 的資料流。
5. Priority、unknown space、規劃失敗與 emergency brake 的失效處理。
6. 預計新增或調整的單元測試。
7. 第 13 節兩項驗收的執行方式與預期結果。

任何安全距離、碰撞檢查、軌跡時序、BasicController 切換行為或 obstacle map seam 的
修改，都必須在實作前交由人類開發者確認。
