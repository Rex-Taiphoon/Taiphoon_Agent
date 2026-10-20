# AGENTS.md — AutoTest（自動測試）

## 目的

本目錄存放 Taiphoon_Agent 的自動測試腳本。執行模擬測試時，應優先使用本目錄內的腳本啟動 PX4 SITL 模擬器與操作無人機，不要自行拼湊 Docker 指令。

## 目錄結構

| 路徑 | 用途 |
|------|------|
| `scripts/quick_takeoff.sh` | 一鍵起飛：OFFBOARD → ARM → TAKEOFF 8m |
| `scripts/setpoint_stability_monitor.py` | 觀察單機 Takeoff／MOVE 後 PX4 Setpoint 穩定性 |
| `scripts/px4-sitl-launcher/launch_all.sh` | 一次啟動 N 台 PX4 SITL 容器 |
| `scripts/px4-sitl-launcher/start_single_instance.sh` | 啟動單一 PX4 SITL 實例 |
| `scripts/px4-sitl-launcher/stop_all.sh` | 停止所有 PX4 SITL 容器 |
| `scripts/dual_agent_regression_sitl_validation.sh` | 多機（`--count` 為 2、3 或 4，預設 2，展示以 3 機驗收）回歸：起飛懸停、單機 MOVE、YAW、一字編隊、全體 MOVE、燕行隊形切換、取消編隊、時鐘偏差注入與降落 |
| `scripts/dual_agent_regression_monitor.py` | 上述回歸在 Agent 容器內執行的步驟與判定 |
| `scripts/time_sync_container_validation.sh` | 以一次性容器驗證 chrony 主機／客戶端設定與 `taiphoon_agent timesync` |
| `scripts/start_local_qgcs_agents.ps1` | Windows：以 `docker-compose.local-qgcs.yml` 啟動可供本機原生 QGCS 連線的 Agent |
| `Log/` | 測試紀錄（依根目錄 `AGENTS.md` 的 Verification 規範建立） |

## 標準測試流程

前置：Docker 網路 `Taiphoon_Net` 需存在；PX4 映像 `px4io/px4-sitl:latest` 需已 pull；共用映像 `taiphoon-agent:dev` 需已建立。

測試所需 Docker image 不存在或需要更新時，AI Agent 可自行執行 `docker build --target taiphoon-dev -t taiphoon-agent:dev .` 建置後再測試，並在測試紀錄中記錄建置指令與結果。

```bash
# 1. 啟動 Agent 容器（一容器一機，uXRCE-DDS 橋接內嵌於容器）
docker compose up -d

# 2. 啟動 4 台 PX4 SITL（預設多機測試數量）
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --rmw cyclonedds

# 3. 快速起飛（驗證基本控制流程）
bash AutoTest/scripts/quick_takeoff.sh

# 4. 測試結束清理
bash AutoTest/scripts/px4-sitl-launcher/stop_all.sh
```

## 腳本使用說明

### quick_takeoff.sh

透過 `docker exec uav_1_agent` 依序發布三個命令至 `/uav_all_agent/command`（`std_msgs/msg/String`）：

1. `MODE_OFFBOARD` → 進入 Offboard 模式
2. `ARM` → 解鎖
3. `TAKEOFF 8.0` → 起飛至 8 公尺

### launch_all.sh

管理 N 台 PX4 SITL 容器（`drone1` ~ `droneN`），不啟動也不停止 Agent 容器。

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `--count N` | 無人機數量 | 4 |
| `--net NAME` | Docker 網路名稱 | `Taiphoon_Net` |
| `--rmw TYPE` | `fastrtps` 或 `cyclonedds` | `fastrtps` |
| `--agent NAME` | Agent 容器名稱（僅用於提示檢查指令） | `uav_1_agent` |
| `--initial-spacing-m M` | PX4 起始水平間距；必須大於 `peer_collision_min_distance_m` 的預設 3m | `8.0` |
| `--home-lat LAT` | 第一台 PX4 的起始緯度，需與 `--home-lon` 一起使用 | `25.03996592258936` |
| `--home-lon LON` | 第一台 PX4 的起始經度，需與 `--home-lat` 一起使用 | `121.51195163336611` |

執行前會自動清理舊的 `drone*` 容器。啟動完成後會印出各機 QGC 連線 port 與檢查 ROS2 topics 的指令。

### start_single_instance.sh

啟動單一實例，必須指定 `--index N`（從 0 起）。`--fg` 以前景模式執行（除錯用）。容器命名、namespace 與 port 計算規則和 `launch_all.sh` 相同。

### stop_all.sh

停止並移除所有名稱以 `drone` 開頭的容器，不影響 `uav_{N}_agent` 容器。


## 禁止事項

- 不要繞過本目錄腳本自行拼湊指令啟動 PX4，除非腳本無法滿足需求，如果需要自行組合指令，必須得到使用者同意才可以運行
- 不要用 `launch_all.sh` / `stop_all.sh` 管理 Agent 容器；Agent 容器一律用 `docker compose` 管理。
