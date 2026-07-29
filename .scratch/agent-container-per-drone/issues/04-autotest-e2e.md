# 04 — AutoTest 腳本改寫 + 端對端驗證

**What to build:** 改寫 `AutoTest/` 下既有測試腳本，使其使用 docker compose 模式進行全棧端對端測試。一鍵啟動 3 台 PX4 SITL + 3 個 agent 容器 → 發送編隊命令 → 驗證 AgentState topic 位置符合預期隊形 → 記錄測試結果。

**Blocked by:** #1, #2, #3

**Status:** ready-for-agent

## Acceptance Criteria

- [ ] `AutoTest/verify_e2e_formation.sh` 改寫為：`docker compose up -d` → `launch_all.sh --count 3` → 等待 topic 就緒 → 發送 `FORMATION_CIRCLE` 命令 → 驗證三機 AgentState 位置形成環形 → 記錄結果
- [ ] 測試腳本不再依賴 `ros2 launch` 或 `docker compose exec taiphoon-agt`（舊容器名）
- [ ] 驗證內容包含：每台 agent 的 AgentState topic 存在、NED 位置偏差在合理範圍、飛行模式正確
- [ ] 測試通過後輸出 PASS/FAIL 摘要至 `AutoTest/` 目錄下以日期命名的 log 資料夾
- [ ] 測試失敗時記錄：失敗條件、重現步驟、實際 vs 預期值
- [ ] 提供獨立的 `docker compose down` + `stop_all.sh` 清理流程，確保測試後環境乾淨
- [ ] 至少一次完整三機環形編隊測試通過並留下記錄

## Design Notes

- 參考既有 `verify_20260728.sh` 的測試結構
- 使用 `ros2 topic echo --once` 或 `ros2 topic pub` 進行驗證
- 等待 topic 就緒的 retry 邏輯需有 timeout（建議 60 秒）
- MAVLink port 已對外暴露，QGC 可同步觀測（非自動化驗證的一部分）
