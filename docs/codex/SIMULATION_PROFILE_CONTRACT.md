# 障礙物 Simulation Profile Contract

## Phase 0 範圍

本文件先固定目前模擬器的輸入、輸出與已知限制。Phase 0 不新增 obstacle
source，也不把「未收到 cloud」宣稱為有效空環境。

## 共用資料契約

- 幾何與 PointCloud 座標使用 `swarm_ned`：`x=North`、`y=East`、`z=Down`。
- PointCloud2 只包含 little-endian、float32、連續 x/y/z 欄位，point step 為
  12 bytes。
- cloud 發布 topic 為 `/{agent_namespace}/perception/obstacles`，目前 QoS 為
  best-effort、depth 10。
- obstacle generator 會拒絕非有限幾何、非法 sensor model 與非有限 sensor pose。
- `ground_truth_map` 與 `sensor_emulation` 的生成結果都應維持 finite points。

## 目前 profiles

| Profile | 目前設定值 | 目前行為 | 限制 |
|---|---|---|---|
| `ground_truth_map` | `mode: ground_truth_map` | 發布 YAML／內建幾何的完整表面採樣，不套用 range/FOV | 仍是 raw PointCloud2，不是 LocalObstacleMap abstraction |
| `sensor_emulation` | 其他 mode 字串目前都落入此模式 | 依 range、horizontal／vertical FOV、noise、dropout 選點 | `ObstacleCloudSimulatorNode::sensor_pose_` 固定為 `(0,0,0)`、heading/pitch 零，尚未跟隨 UAV |
| `explicit_empty` | 尚未支援 | 不存在明確的 fresh-valid-empty source | 留待 Phase 2 |

## Scene 與參數

- Node 讀取簡化 YAML；若 config file 無法讀取或沒有障礙物，會加入內建 box：
  center `[12, 0, -3]`、size `[4, 8, 6]`。
- 目前 Node 讀取的主要參數為 publish rate、mode、range、FOV、noise、dropout、
  sample spacing 與 config file。
- `config/obstacles/default.yaml` 的目前範例同時包含 box、cylinder、wall。
- cloud header timestamp 在發布 callback 以 Node `now()` 填入；planner callback
  目前只檢查 frame ID 與點值，不使用 cloud timestamp 做 freshness gate。

## Planner 端目前語意

- 已收到的 finite empty cloud 會被視為有效空障礙列表，status `map_valid=true`。
- 尚未收到 cloud 時，`EgoSwarmPlannerNode` 仍以初始空列表規劃，status
  `map_valid=false`；這是目前行為基線，不是 Phase 2 的安全契約。
- frame 不等於 `swarm_ned` 或點包含 NaN/Inf 時，callback 拒絕整朵 cloud，且
  保留前一份 obstacle points。

## 後續替換界線

Phase 2 可在此 contract 上加入 `explicit_empty`、ground-truth／synthetic sensor
來源統一介面、cloud timeout 與 `require_obstacle_cloud`。不得因此改變
`EgoSwarmPlanner` 的 candidate algorithm、BasicController 的 PX4 publisher
責任，或繞過 ROS2/DDS 資料鏈。

