# PX4 與 Taiphoon GCS 運行檢查

## 本次測試計畫

- 主角色：測試與 PX4 SITL 驗證人員；協作責任：Docker／部署維護、ROS2 介面核對。
- 使用者授權：包含飛控的運行檢查，GCS 暫設 `192.168.0.108`，保留環境供另一台電腦查看。
- 環境：Windows Docker Desktop、已發布的 AMD64 Agent 映像、PX4 SIH 軟體飛控；
  本機 Wi-Fi `192.168.0.104`，GCS 使用 Taiphoon GCS／ROS2，domain 0。
- 資料流：PX4 `uav_1` → UDP 8888 uXRCE-DDS → Agent 狀態與姿態 → GCS DDS peer。
  一台 Agent 對應一台軟體飛控，不操作實體飛控，不發送 Arm、Takeoff 或飛行任務。
- Agent 由隔離測試 Compose 管理，使用既有預編譯 runtime entrypoint。
  測試設定存於 `.scratch/flight-gcs-check/`，不改動正式 Compose、Dockerfile 或使用者設定。
- 跨機 DDS 使用本機 LAN 位址公告及 GCS unicast peer，僅對本機 LAN IPv4 映射
  domain 0 的 UDP 7400～7491；不修改 Docker Desktop 的全域 host-network 設定。
- PX4 優先沿用 `AutoTest/scripts/px4-sitl-launcher/start_single_instance.sh` 內容。
  原腳本含強制清理，因此安全副本改為同名容器存在就拒絕啟動，並保留停止後容器。
  按 `AutoTest/AGENTS.md` 先取得替代啟動方式同意，再執行安全副本。

## 檢查步驟與預期結果

1. 開發容器完成 workspace 建置；檢查映像、網路、GCS IPv4 可達性及 Compose 語法。
2. 先啟動 Agent，確認 bridge、控制器、planner、supervisor 與健康檢查成功。
3. 再啟動單台 PX4 SIH，確認 uXRCE-DDS session 及 `/uav_1/fmu/out/...` 資料。
4. 在 Docker 開發容器訂閱 `/uav_1_agent/state` 與 attitude，確認 PX4 狀態有效、
   未解鎖、有有限位置／姿態資料且持續更新；不發布控制命令。
5. 記錄外部 DDS 位址、GCS peer、domain 與 UDP 設定，檢查 GCS 發出的 discovery 流量。
   遠端畫面是否正常仍由使用者在 `192.168.0.108` 驗收，不能只由本機健康宣稱通過。
6. 完成後保留 Agent／PX4 運行；僅停止測試觀測程序，不刪除映像、容器或檔案。

失效情境：PX4 映像版本／DDS 訊息不相容、無效飛控狀態、時鐘不同步、UDP 路由／
防火牆阻擋、GCS domain 或訊息定義不同。逐項記錄失敗，不修改安全門檻使測試通過。
不自行 Docker build、不提交或發布修正。

## 本次結果與保留環境

- 使用者已同意安全啟動副本；PX4 SIH 映像 digest 為
  `sha256:c0fd482ece15bc2eecf0108ae70f8b3fa026faa4c167119f3641702b6fb20d09`。
- `uav_1_agent` 維持 running healthy、重啟次數 0；`drone1` 維持 running、重啟次數 0。
  uXRCE-DDS session 與 PX4 data writer 正常建立，Agent 已取得有效 odometry／GPS。
- 實際訂閱 state 得到 online=true、px4_state_valid=true、armed=false、landed=true，
  有有限 NED／全球位置與電池資料。沒有發送飛行命令。
- 使用者在 `192.168.0.108` 的 Taiphoon GCS 回覆「已看到，資料有更新」，
  確認跨機 ROS2 資料鏈可用。本機公告 IP `192.168.0.104`，ROS domain 0。
- 自動連續狀態檢查曾在資料／姿態檢查後因 float32 JSON 輸出型別失敗，已修正輸出；
  第二次檢查遇到 PX4 狀態無效樣本而未通過。因此不能宣稱所有樣本均有效或完整穩定性
  驗證通過，後續應檢查有效狀態切換、時鐘與接收延遲，不能放寬安全判定使測試通過。
- 指定 GCS 原始 IP 的封包觀測未捕捉 RTPS，不作為失聯結論；Desktop NAT／代理可能
  改寫來源位址，遠端 GCS 的實際資料更新已由使用者確認。
- 依使用者要求盡快收尾，Agent／PX4 保持運行，只有開發觀測容器已停止。
  沒有提交／推送、發布、Docker build 或刪除任何測試資料。
