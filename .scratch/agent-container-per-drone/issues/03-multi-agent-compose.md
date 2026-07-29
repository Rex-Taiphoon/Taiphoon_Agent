# 03 — docker-compose 多機部署（fmu1/2/3_agent）

**What to build:** 重構 `docker-compose.yml` 為三個 explicit service（`fmu1_agent`、`fmu2_agent`、`fmu3_agent`），使用 YAML anchor 減少重複。`docker compose up -d` 啟動三機全棧，搭配 `launch_all.sh --count 3` 啟動 PX4 SITL，同儕發現正常，收到編隊命令後形成指定隊形。

**Blocked by:** #1, #2

**Status:** ready-for-agent

## Acceptance Criteria

- [ ] `docker-compose.yml` 宣告 `fmu1_agent`、`fmu2_agent`、`fmu3_agent` 三個獨立 service，使用 `&agent-defaults` YAML anchor
- [ ] 每個 service 獨立注入環境變數：`TARGET_SYSTEM`（1/2/3）、`UXRCE_PORT`（8888/8889/8890）、`AGENT_NAMESPACE`（`fmu1_agent`/`fmu2_agent`/`fmu3_agent`）
- [ ] 所有 service 共用 `network_mode: host` 與 `ipc: host`，確保 ROS2 DDS discovery 正常
- [ ] `docker compose up -d` 後三個容器皆為 Running 狀態
- [ ] `docker compose logs -f` 可同時看到三個容器的 ROS2 log，每行附 container name 前綴
- [ ] 移除 `taiphoon-agt` 舊 service 定義
- [ ] 移除 `uxrce-agent` service（`--profile hw`）
- [ ] 啟動三台 PX4 SITL（`launch_all.sh --count 3`）後，三個 agent 的同儕發現機制正常掃描到彼此的 AgentState topic
- [ ] 發送編隊命令（`ros2 topic pub /fmu_all_agent/command std_msgs/msg/String "data: FORMATION_CIRCLE 5.0 10.0"`）後，三機在 QGC 上呈現環形編隊
- [ ] **編隊不會自動觸發**：同儕發現成功後，僅訂閱並追蹤狀態，不自動開始編隊。必須收到 `/fmu_all_agent/command` 的外部命令才執行

## Design Notes

- YAML anchor 語法：`<<: *agent-defaults` 繼承共用設定
- 每個 service 的 `container_name` 即為 `fmu{N}_agent`，確保 PX4 SITL 腳本可透過 container name 解析 IP
- 編隊命令 topic `/fmu_all_agent/command` 維持不變（ADR-0001）
