# 05 — 端對端整合：三機環形編隊 SITL 驗證

**What to build:** 啟動三機 PX4 SITL + 三個 Taiphoon_Agent 執行個體（各自有 MissionPlanner + TrajectoryController + BasicControllerNode），執行完整環形編隊任務，驗證所有模組協同運作正確。

**Blocked by:** 02 — Agent 狀態發布 + 位置安全監控、03 — 任務規劃層重構、04 — 軌跡控制層新增

**Status:** ready-for-agent

- [ ] 三機 PX4 SITL 啟動（`px4-sitl-launcher`），一字型排列，起始 GPS = (25.03996592258936, 121.51195163336611)
- [ ] 三個 Agent 執行個體各自啟動，namespace 分別為 fmu1_agent、fmu2_agent、fmu3_agent
- [ ] 同儕發現：每個 Agent 正確發現另外兩個同儕的 `/fmu*_agent/state`
- [ ] Arm + Takeoff：三機同時解鎖，起飛至 8m
- [ ] 環形編隊：透過 `/fmu_all_agent/command` 發送 `FORMATION_CIRCLE 5.0 8.0`
- [ ] 位置驗證：fmu1 (angle 0°)、fmu2 (angle 120°)、fmu3 (angle 240°)，對照既有期望值（半徑約 5m，高度 -8m NED）
- [ ] 安全邊界驗證：飛行過程中任兩機 NED 位置歐氏距離始終 ≥3m
- [ ] 離線測試：關閉 fmu3 後，fmu1 與 fmu2 自動調整為雙機環形（角度 0° 與 180°）
- [ ] 測試紀錄留存於 `AutoTest/20260728_formation_refactor/`，含 node_list、topic_list、位置軌跡 log
- [ ] 對照 `AutoTest/20260728_formation/test_log.md` 既有基準，偏差在合理範圍內

## Comments
