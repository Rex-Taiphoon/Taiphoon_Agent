# 02 — PX4 SITL 啟動腳本改寫

**What to build:** 改寫 `start_single_instance.sh`、`launch_all.sh`、`stop_all.sh`，PX4 SITL 容器的 uXRCE-DDS client 改連 `fmu{N}_agent` 容器內的 MicroXRCEAgent（不再需要獨立 `drone{N}-uxrce-dds-agent` 容器）。宿主機執行腳本即可啟動 PX4 模擬器，QGC 可透過 MAVLink port 連線觀測。

**Blocked by:** None — 可與 #1 並行

**Status:** ready-for-agent

## Acceptance Criteria

- [ ] `start_single_instance.sh` 移除獨立 uXRCE-DDS Agent 容器的啟動邏輯（`AGENT_NAME` 相關程式碼）
- [ ] PX4 SITL 容器的 entrypoint 中，uXRCE-DDS client 的目標主機改為 `fmu{N}_agent`（從 `AGENT_NAME` 環境變數或 hostname 推導），埠號維持 `--index` 遞增（8888, 8889, 8890）
- [ ] `launch_all.sh` 移除對 `start_single_instance.sh` 傳遞 uXRCE-DDS Agent 相關參數的邏輯，只管理 PX4 SITL 容器
- [ ] `stop_all.sh` 移除 `drone{N}-uxrce-dds-agent` 的清理邏輯，只清理 `drone{N}` PX4 SITL 容器
- [ ] 腳本參數介面不變：`--count`、`--net`、`--rmw` 維持相同語義
- [ ] GPS 起始位置 `(25.03996592258936, 121.51195163336611)` 維持不變，一字型排列間距 3m
- [ ] 單機 SITL 可獨立啟動（不需 agent 容器先存在），PX4 內部 uXRCE-DDS client 會持續重試直到 agent 容器出現
- [ ] QGC 可透過 `localhost:18570`（instance 0）連線

## Design Notes

- 參考既有 `start_single_instance.sh` 的 PX4 容器啟動邏輯
- 關鍵變更：`AGENT_NAME` 從 `drone{N}-uxrce-dds-agent` 改為 `fmu{N}_agent`
- PX4 entrypoint 中的 `uxrce_dds_client start -t udp -h $AGENT_IP` 語法不變，只改目標 container name
