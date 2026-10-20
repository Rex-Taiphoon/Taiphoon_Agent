# 多機時間同步（chrony）

主角色：Docker／部署維護人員；協作：EGO-Swarm 規劃與安全驗證。

## 背景

`ego_swarm_planner` 交換的 `AgentTrajectory` 以發送端系統時間標記 `stamp` 與
`start_time`。接收端以自己的系統時間檢查同儕軌跡；`stamp` 與收到時間相差超過
`max_peer_time_skew_s`（預設 1.0 秒）時，該同儕軌跡判為無效。只要有任一台在空中的
同儕資料無效，單機 MOVE 與編隊都會 fail-closed 原地煞停或等待。

SITL 的所有容器共用同一台主機時鐘，因此不會出現這個問題。樹莓派沒有 RTC，外場沒有
網路時開機時間取自上次關機前保存的時間，兩台之間常差數秒以上。

時間差也直接影響避碰精度：以 2.5 m/s 計算，0.1 秒時間差約等於 0.25 m 的同儕位置
誤差。因此本專案不以放寬 `max_peer_time_skew_s` 處理，而是讓所有機上主機對時。

## 架構

```text
  uav_1 的樹莓派（時間主機）              uav_N 的樹莓派（客戶端）
  ┌──────────────────────────┐          ┌──────────────────────────┐
  │ chronyd                  │   NTP    │ chronyd                  │
  │ local stratum 8          │◄─────────│ server <主機IP> iburst    │
  │ allow（提供時間）          │ UDP 123  │ makestep 0.1 3            │
  │ makestep 0.1 3           │          │ maxslewrate 500           │
  ├──────────────────────────┤          ├──────────────────────────┤
  │ taiphoon_agent start     │          │ taiphoon_agent start     │
  │  └ 對時檢查通過才啟動容器   │          │  └ 等待對時完成才啟動容器   │
  ├──────────────────────────┤          ├──────────────────────────┤
  │ Agent 容器（共用主機時鐘）  │◄── DDS ─►│ Agent 容器（共用主機時鐘）  │
  │  └ PeerClockSkewMonitor  │          │  └ PeerClockSkewMonitor  │
  └──────────────────────────┘          └──────────────────────────┘
```

- **第 1 層：主機對時。** chrony 在主機作業系統執行，容器與主機共用核心時鐘，容器內
  不執行對時，也不取得調整時鐘的權限。uav_1 的樹莓派為時間主機（方案 C），不依賴
  地面站或 QGCS；地面站送出的命令字串沒有時間戳，不需要參與對時。
- **第 2 層：啟動前對時檢查。** `taiphoon_agent` 在實機啟動容器前讀取
  `chronyc -c tracking`，確認已同步且系統時鐘誤差不超過門檻；客戶端會等待主機上線。
- **第 3 層：Agent 內部監測。** `PeerClockSkewMonitor` 只用既有的同儕軌跡估計時鐘差，
  輸出 log 與 `PlannerStatus.reason`，不參與任何控制決策。

只在開機前三次更新允許跳時（`makestep 0.1 3`），之後只能以每秒 0.5 ms 以內的速度
微調（`maxslewrate 500`）。飛行中的系統時間不會突然跳動，避免影響 BasicController
的軌跡取樣與同儕軌跡時間比對（PX4 狀態逾時判斷已改用單調時鐘，不受系統時間影響）。

## 介面

### 主機對時設定腳本

`Tools/time_sync/setup_time_sync.sh` 在樹莓派主機（非容器）執行：

| 命令 | 說明 |
|---|---|
| `sudo bash setup_time_sync.sh master` | 設定本機為時間主機（uav_1） |
| `sudo bash setup_time_sync.sh client <主機IPv4>` | 設定本機向時間主機對時 |
| `bash setup_time_sync.sh render master` | 只輸出設定內容，不修改系統 |
| `bash setup_time_sync.sh render client <主機IPv4>` | 只輸出設定內容，不修改系統 |
| `bash setup_time_sync.sh status` | 顯示 `chronyc tracking` 與 `chronyc sources` |

`master`／`client` 會：安裝 chrony（若尚未安裝）、把原本的 `/etc/chrony/chrony.conf`
備份為 `chrony.conf.taiphoon-backup`（只在第一次備份）、寫入新的設定、停用
`systemd-timesyncd`，並重新啟動 chrony。設定完整取代原檔，因此不會再引用 Ubuntu 預設的
公用 NTP pool，確保整群只有一個時間基準。

### 啟動器設定與命令

`settings.ini` 新增欄位：

| 欄位 | 預設 | 說明 |
|---|---|---|
| `time_sync` | `auto` | `auto`：Linux 實機為 `required`，Windows／macOS 實機為 `warn`，模擬不檢查；`required`：未對時則拒絕啟動；`warn`：只警告；`off`：不檢查 |
| `time_sync_max_offset_ms` | `50` | 系統時鐘相對時間來源的允許誤差（1～1000 ms） |

`required` 模式最多等待 60 秒；逾時就停止啟動並說明原因。單機飛行若不需要對時，可設定
`time_sync=off`。

新增命令 `taiphoon_agent timesync`：顯示本機對時狀態，通過回傳 0，未通過回傳 1。
這個命令不需要 Docker，也不讀取 Release 映像。

判定通過的條件：

1. `chronyc -c tracking` 執行成功且格式正確。
2. `Leap status` 為 `Normal`（已選定時間來源，或時間主機使用本機基準）。
3. `stratum` 介於 1～15。
4. `System time` 的絕對值不超過 `time_sync_max_offset_ms`。

### Agent 參數與診斷

| 參數 | 預設 | 說明 |
|---|---|---|
| `peer_clock_skew_warning_s` | `0.1` | 同儕時鐘差估計超過此值時輸出 WARN；只影響診斷 |

- 估計方式：對每台同儕記錄 `收到時間 − AgentTrajectory.stamp`，取最近 5 秒內的最小值。
  網路延遲只會讓差值變大，取最小值可濾除大部分延遲。正值代表同儕時鐘比本機慢。
- 每台同儕每 30 秒輸出一次 INFO 估計值；超過警告門檻時輸出節流 WARN。
- 同儕資料無效且估計值超過 `max_peer_time_skew_s` 時，`PlannerStatus.reason` 改為
  `peer clock skew agent=<N> offset=<±秒> s exceeds <門檻> s; check time sync`，
  取代原本籠統的 `peer trajectory stale or invalid`。控制行為（煞停／等待）不變。
- 只有規劃器在空中時才會發布軌跡，因此地面上的對時檢查以 `taiphoon_agent timesync` 為準。

## 資料流

```text
chronyd(uav_1) ──NTP──► chronyd(uav_N) ──► 主機系統時鐘 ──► 容器 node->now()
                                                         │
AgentTrajectory.stamp（發送端 now）──DDS──► trajectory_cb ─┤
                                                         ▼
                            PeerClockSkewMonitor.record(stamp, 收到時間)
                                                         │
                         planner_loop_cb ── 同儕資料無效 ──► PlannerStatus.reason
```

## 失效情境

| 情境 | 結果 | 處置 |
|---|---|---|
| 客戶端開機時時間主機未上線 | chrony 未同步，`required` 等待 60 秒後拒絕啟動 | 先開 uav_1，再對客戶端執行 `taiphoon_agent start` |
| 時間主機在飛行中斷電 | 客戶端沿用已校正的頻率自行計時，漂移約每分鐘 1 ms 以下 | 可繼續任務；落地後檢查 `timesync` |
| 時間主機重開機 | 主機時間可能回到舊值；客戶端已過跳時窗口，只會緩慢微調，彼此仍一致 | 客戶端重開機前先確認主機時間 |
| 飛行中取得網際網路 | 設定不含公用 NTP，時間不會被外部來源改變 | 無 |
| 未安裝 chrony 或 chronyd 未執行 | `chronyc` 失敗，`required` 拒絕啟動，`warn` 只警告 | 執行設定腳本，或單機飛行時設定 `time_sync=off` |
| Wi-Fi 封包延遲大 | 單筆差值變大，但取最小值的估計不受影響；chrony 也會濾除 | 無 |
| 同儕時鐘差超過 1 秒 | 規劃器維持原有 fail-closed，`reason` 指出時鐘差 | 依 log 檢查兩台的 `timesync` |

## 驗證方式

1. 單元測試：`chronyc -c tracking` 解析與判定、啟動器設定欄位、`PeerClockSkewMonitor`
   的正常、邊界、NaN／Inf、過期、時間倒退與恢復。
2. chrony 容器驗證：以 `render` 產生的主機／客戶端設定執行 `chronyd -x`（不修改系統時鐘），
   用實際 `chronyc` 輸出驗證 `taiphoon_agent timesync`；以 libfaketime 讓主機時間偏移，
   確認客戶端判定失敗。腳本：
   `bash AutoTest/scripts/time_sync_container_validation.sh <Linux 版 taiphoon_agent>`
   （Windows Git Bash 需先設定 `MSYS_NO_PATHCONV=1`）。`-x` 模式無法驗證 chrony 的實際跳時，
   此部分依 chrony 本身行為，需在樹莓派實機確認。
3. SITL 回歸：`AutoTest/scripts/dual_agent_regression_sitl_validation.sh --count 3`（最終展示以
   三機驗收；`--count 2` 為雙機）依序驗證起飛懸停、單機 MOVE、YAW、一字編隊、全體 MOVE、
   燕行隊形切換、取消編隊、偏差注入與全體降落，並從 Agent 日誌確認每對同儕的 `Peer clock offset`
   估計。四機環形編隊另用 `ego_route_planning_sitl_validation.sh --scenario swarm`。
4. SITL 偏差注入：發布一台不存在的同儕（agent 7）、`stamp` 早 3 秒且位於 1 km 外的終點佔位軌跡，
   確認每台 Agent 皆輸出 `Peer clock skew agent=7 offset=+3.000 s` WARN，且受測機不受影響。

## 外場操作順序

1. 第一次：uav_1 執行 `sudo bash setup_time_sync.sh master`；其他機執行
   `sudo bash setup_time_sync.sh client <uav_1 IPv4>`。建議在路由器把各台樹莓派綁定固定 IP。
2. 每次飛行：先開 uav_1 的樹莓派，再開其他機。
3. 各台執行 `taiphoon_agent start`（會自動等待對時），或先用 `taiphoon_agent timesync` 確認。
4. 起飛後，可從 Agent log 的 `Peer clock offset` 確認各同儕的時鐘差。
