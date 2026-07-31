# 03 — 文件更新 + SITL 端對端驗證

**What to build:** `gcs-interface-spec.md` 更新至 v1.1，記錄新增欄位。SITL 啟動後驗證 `/fmu1_agent/state` topic 包含合理經緯度（~25.04/121.51）與真實電池資料（電壓 > 0）。

**Blocked by:** 02 — BasicControllerNode 訂閱 GPS/Battery + 上線

**Status:** ready-for-agent

- [ ] `docs/gcs-interface-spec.md` AgentState 欄位表新增 3 欄位，版本號 1.0 → 1.1
- [ ] SITL 驗證 `position_global` 約 [25.04, 121.51, 0~500]
- [ ] SITL 驗證 `battery_voltage_v` > 0
- [ ] SITL 驗證 `battery_remaining` 非固定 1.0
- [ ] 驗證結果記錄於 `AutoTest/` 新目錄
