# Spec: 一容器一機部署模型與 uXRCE-DDS 橋接內嵌

**狀態**：ready-for-agent
**日期**：2026-07-29
**ADR**：[ADR-0002](../adr/0002-agent-container-per-drone.md)
**領域詞彙**：參閱 [CONTEXT.md](../../CONTEXT.md)

---

## Problem Statement

目前 Taiphoon_Agent 在單一 Docker 容器內執行多台無人機的所有 ROS2 節點，uXRCE-DDS Agent 為獨立容器。這導致三個痛點：

1. **部署不對稱**：SITL 模擬時每台無人機有獨立的 uxrce 容器，但實機模式共用一個；同一份程式碼有兩套部署拓撲。
2. **故障隔離不足**：N 台無人機共用同一個 process space，一個節點崩潰可能波及同一容器內的其他無人機。
3. **操作複雜**：開發者需手動 `ros2 launch` 指定 `drone_count:=3` 來啟動多機節點，而非直覺地「啟動一個容器 = 部署一台無人機」。

開發者期望：一個 Docker 容器對應一台無人機的完整軟體堆疊，SITL 與實機使用相同的部署模型。

## Solution

將 MicroXRCEAgent（uXRCE-DDS 橋接 daemon）內嵌至 Taiphoon_Agent Docker 映像中。每個容器包含一台無人機的完整堆疊——uXRCE-DDS 橋接 + BasicController + MissionPlanner + TrajectoryController。容器命名遵循 `fmu{N}_agent` 慣例。

開發者只需：
- `docker compose up -d` 啟動所有 agent 容器
- 個別控制時：`docker compose restart fmu1_agent` / `docker compose stop fmu2_agent`
- 觀測 log：`docker compose logs -f` 聚合所有容器輸出

## User Stories

1. As a 開發者，I want 一個 `docker compose up -d` 命令就能啟動 N 台無人機的完整 agent 堆疊，so that 我不需要手動計算 `drone_count` 或用 `ros2 launch` 管理多機生命週期。

2. As a 開發者，I want 每個 agent 容器被命名為 `fmu1_agent`、`fmu2_agent`、`fmu3_agent`，so that 我可以直覺地對特定無人機進行重啟、停止或除錯。

3. As a 開發者，I want MicroXRCEAgent 在容器啟動時自動先於 Taiphoon 節點啟動，so that PX4 topic 在節點開始訂閱前就已存在。

4. As a 開發者，I want uXRCE-DDS 橋接 crash 時 Agent 自動降級至 Position Hold 再觸發 RTL，so that 無人機不會在失去通訊後繼續執行過時的控制指令。

5. As a 開發者，I want 同一份 Dockerfile 在 macOS（x86_64/ARM64）上建構 SITL 映像、在樹莓派上原生建構實機映像，so that 開發與部署使用相同的建構流程。

6. As a 開發者，I want `launch_all.sh` 只負責啟動 PX4 SITL 容器而不碰 Taiphoon_Agent，so that SITL 模擬器與 agent 的生命週期完全解耦。

7. As a 開發者，I want 透過 `docker compose logs -f` 同時看到所有 agent 容器的 ROS2 log，so that 多機測試時不需要在三個終端機之間切換。

8. As a 開發者，I want 每個 agent 容器可獨立設定 uXRCE-DDS 橋接的 UDP 埠號（透過環境變數），so that 多機 SITL 時每台無人機使用不同埠號連接各自的 PX4 模擬器。

9. As a 測試人員，I want 透過 `docker compose logs` 驗證特定 agent 的里程計 timeout 行為，so that 沉默失效的降級邏輯可以被自動化測試確認。

10. As a 測試人員，I want `AutoTest/` 下的測試腳本不再依賴 `ros2 launch` 來啟動節點，so that 測試流程與實際部署流程一致。

## Implementation Decisions

### 1. MicroXRCEAgent 從原始碼編譯入 Docker 映像

MicroXRCEAgent 在 `taiphoon-builder` stage 內從 GitHub `v2.4.3` tag clone，並以 cmake superbuild 編譯。編譯步驟放在 `COPY . .`（Taiphoon 原始碼）之前，確保修改 Taiphoon 程式碼時 Docker layer cache 不重編 MicroXRCEAgent。

### 2. entrypoint 管控啟動順序

`docker/entrypoint.sh` 先背景啟動 `MicroXRCEAgent udp4 -p ${UXRCE_PORT}`，然後 polling `/fmu/out/vehicle_odometry` topic（透過 `ros2 topic list | grep vehicle_odometry`），直到 topic 出現後才 exec Taiphoon_Agent 節點。若 timeout（30 秒）topic 未出現則 exit 1 讓 Docker restart policy 接管。

### 3. odometry timeout 沉默失效偵測

`PositionSafetyMonitor` 新增 odometry 接收時間戳記。控制迴圈中檢查：若距離上次收到里程計超過 1 秒（`ODOMETRY_TIMEOUT_MS`），則：
1. 首次觸發：發送 Position Hold 命令（`switch_to_hold_mode()`）
2. 持續 5 秒後：發送 RTL 命令（`switch_to_rtl_mode()`）

此為既有模組的擴充，不改變其對外介面。

### 4. docker-compose 使用 explicit service 定義

宣告三個獨立 service：`fmu1_agent`、`fmu2_agent`、`fmu3_agent`，使用 YAML anchor (`&agent-defaults`) 減少重複。每個 service 可獨立設定環境變數：

- `TARGET_SYSTEM`：對應 PX4 MAV_SYS_ID（1, 2, 3）
- `UXRCE_PORT`：uXRCE-DDS 橋接 UDP 埠號（8888, 8889, 8890）
- `AGENT_NAMESPACE`：從 `TARGET_SYSTEM` 推算（`fmu1_agent`, `fmu2_agent`, `fmu3_agent`）

### 5. launch_all.sh 精簡

移除 `drone{N}-uxrce-dds-agent` 容器的啟動邏輯。`launch_all.sh` 僅管理 `drone{N}`（PX4 SITL）容器。Agent 容器由 `docker compose up -d` 獨立管理。

### 6. 跨架構建構策略

開發機（macOS x86_64/ARM64）只建構 SITL 映像。ARM64 實機映像由樹莓派上原生 `docker build` 產出。同一份 `Dockerfile`，不引入 QEMU cross-compilation。

### 7. 容器內節點啟動

由於一容器只有一台無人機，容器內不再需要 `ros2 launch` 指定 `drone_count`。entrypoint 直接以 `ros2 run` 啟動三個節點（BasicController、MissionPlanner、TrajectoryController），`agent_namespace` 從環境變數 `AGENT_NAMESPACE` 讀取。

### 8. docker/Dockerfile.uxrce-agent 廢棄

MicroXRCEAgent 的建構邏輯已併入主 `Dockerfile`，獨立的 `docker/Dockerfile.uxrce-agent` 可移除。

## Testing Decisions

### 什麼構成好的測試

- 只驗證外部行為：topic 出現順序、timeout 觸發、容器啟動狀態
- 不驗證內部實作細節：不定義 MicroXRCEAgent 的內部狀態、不檢查 ROS2 node graph 內部結構
- 使用既有 seam：entrypoint script、PositionSafetyMonitor、docker compose

### 測試模組與策略

| 模組 | 測試方式 | 驗證內容 |
|------|----------|----------|
| entrypoint.sh | shell 整合測試 | MicroXRCEAgent 先啟動、odometry topic polling timeout 行為 |
| PositionSafetyMonitor | 既有 `test_position_safety_monitor` 擴充 | odometry timeout 觸發 → hold 命令 → RTL 命令的狀態轉換 |
| Dockerfile | `docker build` CI | 映像建構成功、`MicroXRCEAgent --help` 可執行 |
| docker-compose.yml | `docker compose up -d` + `docker compose ps` | 三個容器皆 Running、環境變數正確注入 |

### 既有測試先例

- `test_position_safety_monitor`（純邏輯測試，不依賴 ROS2）→ 擴充 timeout 相關測試案例
- `verify_e2e_formation.sh`（端對端 SITL 測試）→ 改寫為 docker compose 模式

## Out of Scope

- **Serial 傳輸支援**：本 spec 僅涵蓋 UDP 傳輸模式。飛控已支援網路協定，serial device mapping 不在本次範圍。
- **多架構 CI/CD**：ARM64 實機映像由樹莓派上原生 `docker build`，不引入 CI cross-compilation pipeline。
- **Candidate 2（PeerRegistry 統整）**：MissionPlanner 與 TrajectoryController 的同儕發現重複，留待後續 spec。
- **Candidate 3（BasicControllerNode 拆分）**：節點職責拆分留待後續 spec。
- **動態縮放**：`docker compose --scale` 模式不在本次範圍，使用 explicit service 定義。

## Further Notes

- 此 spec 與 ADR-0001（Agent-to-Agent 狀態發布架構）無衝突，AgentState topic 機制維持不變。
- `fmu{N}_agent` 命名規則與 CONTEXT.md 既有詞彙一致，從 Docker container name 到 ROS2 topic namespace 形成統一名稱空間。
- 實機部署時，樹莓派只需 `docker run` 一個容器，映像內含完整堆疊，不需額外安裝 ROS2 或 MicroXRCEAgent。
