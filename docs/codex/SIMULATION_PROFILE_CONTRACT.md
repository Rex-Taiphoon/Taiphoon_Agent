# 障礙物 Simulation Profile Contract

## Phase 2 狀態

Phase 2 已建立 obstacle source 與 LocalObstacleMap contract。PointCloud2 仍是
ROS2/DDS 的輸入資料鏈，但 planner 只接收 pure C++ `IObstacleMap`。

## 共用資料契約

- 幾何與 PointCloud 座標使用 `swarm_ned`：`x=North`、`y=East`、`z=Down`。
- PointCloud2 只包含 little-endian、float32、連續 x/y/z 欄位，point step 為
  12 bytes。
- cloud 發布 topic 為 `/{agent_namespace}/perception/obstacles`，目前 QoS 為
  best-effort、depth 10。
- obstacle generator 會拒絕非有限幾何、非法 sensor model 與非有限 sensor pose；
  `explicit_empty` 不需要使用 sensor pose。
- `ground_truth` 與 `synthetic_sensor` 的生成結果都應維持 finite points。
- `explicit_empty` 產生 valid、fresh、zero-point cloud；未收到 cloud 仍是 missing。

## 目前 profiles

| Profile | 目前設定值 | 目前行為 | 限制 |
|---|---|---|---|
| `ground_truth` | `mode: ground_truth` | 發布 YAML／內建幾何的完整表面採樣，不套用 range/FOV | 仍以 PointCloud2 傳輸，planner 端由 RawPointObstacleMap 接收 |
| `synthetic_sensor` | `mode: synthetic_sensor` | 依 UAV current pose、range、horizontal／vertical FOV、noise、dropout 選點 | 尚未模擬真實 sensor latency；pose 尚未取得時不發布 |
| `explicit_empty` | `mode: explicit_empty` | 發布 zero-point、finite、valid map | 必須由明確 source 發布，不能以 silence 取代 |

## Scene 與參數

- Node 讀取簡化 YAML；若 config file 無法讀取或沒有障礙物，且 mode 不是
  `explicit_empty`，會加入內建 box：
  center `[12, 0, -3]`、size `[4, 8, 6]`。
- Node 讀取的主要參數為 Agent namespace/index、固定 frame origin、publish rate、
  mode、range、FOV、noise、dropout、sample spacing 與 config file。
- `config/obstacles/default.yaml` 的目前範例同時包含 box、cylinder、wall。
- cloud header timestamp 在發布 callback 以 Node `now()` 填入；planner 以
  `obstacle_cloud_timeout_s` 做 freshness gate。

## Planner 端目前語意

- 已收到的 finite empty cloud 會被 `RawPointObstacleMap` 視為 valid empty map，
  在 timeout 內 status `map_valid=true`。
- 尚未收到 cloud 是 `missing`；收到後超過 timeout 是 `stale`；兩者都不會被轉成
  empty map，planner 會進入 HOLD 等待 fresh source。
- frame 不等於 `swarm_ned`、timestamp 非法或點包含 NaN/Inf 時，callback 拒絕整朵
  cloud，並保留前一份有效 map。

## 後續替換界線

`RawPointObstacleMap` 是目前 raw-point backend；`IObstacleMap` 保留未來 voxel／
spatial map 的替換 seam。Phase 2 不改變 `EgoSwarmPlanner` 的 candidate algorithm、
BasicController 的 PX4 publisher 責任，亦不繞過 ROS2/DDS 資料鏈。
