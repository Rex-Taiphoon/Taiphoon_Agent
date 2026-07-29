# ADR-0002：一容器一機部署模型與 uXRCE-DDS 橋接內嵌

**狀態**：accepted
**日期**：2026-07-29
**取代**：無（新增決策）

## 背景

目前 Taiphoon_Agent 使用單一 Docker 容器執行多台無人機的 ROS2 節點（透過 namespace 隔離），uXRCE-DDS Agent 為獨立容器（`--profile hw`）。此模型存在以下問題：

1. 「一 Agent 一機」僅在 ROS2 namespace 層級實現，容器內 N 組節點共用 process space，故障隔離不足。
2. SITL 與實機的部署拓撲不對稱（SITL：每機一個 uxrce 容器；實機：共用一個）。
3. uXRCE-DDS Agent 與 Taiphoon_Agent 生命週期不同步，啟動順序與崩潰處理無統一策略。
4. 資源競爭：N 台無人機的控制迴圈爭搶同一容器 CPU/記憶體。

## 決策

**每個 Agent 部署為獨立 Docker 容器**，容器邊界即 Agent 邊界。容器內含：

- **MicroXRCEAgent**（uXRCE-DDS 橋接 daemon）：從原始碼編譯入映像，透過 UDP 與飛控通訊
- **Taiphoon_Agent ROS2 節點**：BasicController + MissionPlanner + TrajectoryController

容器命名遵循 `fmu{N}_agent` 慣例，與既有 topic namespace 一致。

### 關鍵設計要點

| 面向 | 決策 |
|------|------|
| 傳輸模式 | 僅 UDP（飛控支援網路協定），埠號可透過環境變數自訂 |
| 啟動順序 | entrypoint 先啟動 MicroXRCEAgent，poll `/fmu/out/vehicle_odometry` 出現後再啟動 Taiphoon 節點 |
| 崩潰處理 | odometry timeout（1 秒）觸發降級：Position Hold → RTL |
| 建構策略 | MicroXRCEAgent 從原始碼編譯入 Dockerfile（`taiphoon-builder` stage），非 binary artifact |
| 跨架構 | x86_64 SITL 映像在開發機建構；ARM64 實機映像在樹莓派上原生 `docker build` |
| compose 定義 | 使用 YAML anchor 宣告 `fmu1_agent`、`fmu2_agent`、`fmu3_agent` 三個獨立 service（非 `--scale`） |
| launch_all.sh | 僅管理 PX4 SITL 容器，不參與 Taiphoon_Agent 啟動 |
| log 聚合 | `docker compose logs -f` 原生支援 |

### 替代方案

| 方案 | 理由 |
|------|------|
| 維持現狀（單容器多機 + uXRCE 分離） | SITL/HW 部署不對稱、故障隔離不足、不符合「一 Agent 一機」語義 |
| uXRCE-DDS Agent 預編譯 binary artifact | 需手動維護版本與多架構、違反單一 Dockerfile 可重現性 |
| `docker compose --scale` 模式 | 無法 per-drone 設定不同 `TARGET_SYSTEM` 與 `UXRCE_PORT` |

## 後果

- `Dockerfile` 需在 `taiphoon-builder` stage 新增 MicroXRCEAgent 的 clone + cmake build 步驟
- `docker/Dockerfile.uxrce-agent` 可廢棄（併入主 Dockerfile）
- `docker/entrypoint.sh` 需加入 MicroXRCEAgent 啟動與 topic polling 邏輯
- `docker-compose.yml` 需重構為三個獨立 service（YAML anchor）
- `skills/px4-sitl-launcher/scripts/launch_all.sh` 移除 uxrce-agent 容器啟動邏輯
- `BasicController` / `PositionSafetyMonitor` 需加入 odometry timeout 與降級行為
- `AutoTest/` 測試腳本需改寫：不再 `ros2 launch`，改為驗證 topic 內容
- 與 ADR-0001 無衝突——Agent-to-Agent 通訊仍透過 AgentState topic
