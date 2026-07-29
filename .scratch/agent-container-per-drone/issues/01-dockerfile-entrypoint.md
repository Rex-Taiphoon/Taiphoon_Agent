# 01 — Dockerfile + entrypoint 重構（單機 Agent 容器）

**What to build:** 建構一個包含 MicroXRCEAgent + Taiphoon_Agent 全堆疊的 Docker 映像，`docker compose up fmu1_agent` 即可啟動一台無人機的完整 agent 容器。容器內 MicroXRCEAgent 先啟動並與 PX4 建立 uXRCE-DDS 連線，確認 odometry topic 出現後自動啟動 Taiphoon 節點（BasicController + MissionPlanner + TrajectoryController）。

**Blocked by:** None — 可立即開始

**Status:** ready-for-agent

## Acceptance Criteria

- [ ] 主 `Dockerfile` 的 `taiphoon-builder` stage 加入 MicroXRCEAgent `v2.4.3` 的 clone + cmake superbuild，編譯步驟在 `COPY . .` 之前（不因 Taiphoon 程式碼變動而重編）
- [ ] `docker/entrypoint.sh` 改寫：背景啟動 `MicroXRCEAgent udp4 -p ${UXRCE_PORT}` → polling odometry topic（30 秒 timeout）→ 出現後依序啟動 BasicController、MissionPlanner、TrajectoryController
- [ ] odometry 判斷邏輯預留替換介面（例如獨立 function `wait_for_odometry()`），未來可改用 health check 或 ROS2 node list 等其他條件，非本次優先
- [ ] 啟動參數（`TARGET_SYSTEM`、`UXRCE_PORT`、`AGENT_NAMESPACE`）從環境變數讀取，無硬編碼
- [ ] `docker/Dockerfile.uxrce-agent` 標記廢棄或移除
- [ ] `docker build` 成功，`MicroXRCEAgent --help` 可在容器內執行
- [ ] 使用既有的 `test_position_safety_monitor` 確保重構不破壞現有邏輯

## Design Notes

- 啟動順序：MicroXRCEAgent daemon → odometry topic → Taiphoon 節點
- odometry topic 名稱格式：`/fmu/out/vehicle_odometry`（由 PX4 uXRCE-DDS client 發布）
- 30 秒 timeout 後仍無 odometry topic → exit 1，由 Docker restart policy 處理
- 不引入 supervisord / s6-overlay，保持 entrypoint script 簡潔
