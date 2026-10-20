# 實機障礙物地圖來源（obstacle_source）

主角色：Docker／部署維護人員；協作：EGO-Swarm 規劃與安全驗證。

## 背景

`ego_swarm_planner` 只在收到新鮮的障礙物地圖（`/uav_N_agent/perception/obstacles`，
PointCloud2）時才規劃移動；地圖缺少或過期時一律煞停或原地等待。`require_obstacle_cloud`
只影響狀態訊息文字，不會放行。

模擬時 `obstacle_cloud_simulator` 以 `explicit_empty` 模式持續發布空點雲，代表「已確認沒有
障礙物」。實機部署（`taiphoon_agent` 的 deploy、dev real）為避免以假的空地圖代表環境安全，
刻意不啟動模擬器，結果是實機的單機 MOVE、編隊移動與全體 MOVE 一律因
`Required obstacle map is missing` 煞停。起飛、懸停、YAW 與降落由 BasicController 處理，
不受影響。

## 設計

`settings.ini` 新增 `obstacle_source`，只影響實機（非模擬）模式：

| 值 | 行為 |
|---|---|
| `none`（預設） | 維持現行安全行為：不提供障礙物地圖，規劃器不移動 |
| `explicit_empty` | 操作人員明確宣告場地沒有障礙物：啟動 `obstacle_cloud_simulator` 的 `explicit_empty` 模式持續發布空地圖；規劃器只做機間避碰，**不會避開實體障礙物** |

模擬模式不受此欄位影響，維持原本的模擬器設定。

**介面**

- 啟動器驗證 `obstacle_source` 只能是 `none` 或 `explicit_empty`。
- 實機 compose：`ENABLE_OBSTACLE_CLOUD_SIMULATOR` 依設定為 `true`／`false`，並明確設定
  `OBSTACLE_CLOUD_MODE: explicit_empty`，不依賴進入點預設值。
- 每次實機 `start` 都印出目前的障礙物來源：
  - `explicit_empty`：警告規劃器不會避開實體障礙物，僅適用已確認淨空的場地。
  - `none`：提示 MOVE／編隊會因缺少障礙物地圖而原地等待，以及如何明確宣告。
- runtime 健康檢查：啟用模擬器時，`obstacle_cloud_simulator` 程序必須存在，否則容器為
  unhealthy，`start` 會失敗並顯示原因。

**資料流**

```text
settings.ini obstacle_source
  → taiphoon_agent renderCompose（ENABLE_OBSTACLE_CLOUD_SIMULATOR、OBSTACLE_CLOUD_MODE）
  → runtime-entrypoint.sh → basic_controller.launch.py
  → obstacle_cloud_simulator（explicit_empty，15 Hz 空點雲）
  → /uav_N_agent/perception/obstacles → ego_swarm_planner 障礙物地圖 Fresh
```

## 失效情境

| 情境 | 結果 |
|---|---|
| 預設 `none` | 與 rc.8 相同：MOVE／編隊原地等待，狀態為 `Required obstacle map is missing` |
| `explicit_empty` 但場地有障礙物 | 規劃器看不到障礙物，可能撞上；只用於已確認淨空的場地，由操作人員負責 |
| 模擬器程序異常結束 | 地圖過期，規劃器煞停（fail-safe）；健康檢查回報 unhealthy |
| 欄位填錯 | 啟動器拒絕啟動並說明允許值 |

機間避碰（同儕軌跡、即時距離、編隊槽位）不受此設定影響。

## rc.8 以前版本的暫時做法

rc.8 沒有 `obstacle_source` 設定；可在 Agent 啟動後，於該台樹莓派手動啟動空地圖節點
（`N` 換成無人機編號；容器重新啟動後需要再執行一次）：

```bash
docker exec -d "$(docker ps -qf label=com.docker.compose.service=agent_1)" bash -c "source /opt/ros/humble/setup.bash && source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && source /home/taiphoon/Taiphoon_Agent/install/setup.bash && exec ros2 run taiphoon_agent obstacle_cloud_simulator --ros-args -r __ns:=/uav_N_agent -p agent_namespace:=uav_N_agent -p mode:=explicit_empty"
```

這與 `explicit_empty` 有相同的安全取捨：規劃器不會避開實體障礙物。

## 驗證

1. 啟動器單元測試：欄位驗證、實機預設 `none` 與 `explicit_empty` 的 compose 環境變數、
   模擬模式不受影響。
2. 啟動器整合測試：實機 `start` 輸出障礙物來源提示與警告、非法值拒絕。
3. SITL 三機（使用與樹莓派相同的 `runtime-entrypoint.sh`，障礙物環境變數與啟動器實機輸出一致）：
   - `none`：起飛懸停、YAW、降落正常，MOVE 被擋下且狀態為 `Required obstacle map is missing`。
   - `explicit_empty`：完整回歸（MOVE、YAW、一字編隊、全體 MOVE、燕行隊形、取消、降落）。
   - rc.8 暫時做法：容器以 `none` 啟動後執行上述 `docker exec`，完整回歸。
