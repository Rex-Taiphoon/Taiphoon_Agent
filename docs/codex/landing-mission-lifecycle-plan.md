# 降落後任務生命週期修正

主角色：任務規劃與編隊工程師；協作責任：飛控橋接與測試驗證（同一代理執行）。人類已指定降落後停止編隊與規劃，並要求完成 Agent 端修正；GCS 精度由 GCS 專案處理。

資料流維持 PX4 → BasicController → AgentState → EGO planner。依有效的 landed／armed 觀測清除任務目標、保留目標、編隊 Epoch／assignment／center、候選與等待狀態、active plan 及 terminal hold。落地後禁止接收移動或編隊任務；重新起飛也不恢復舊編隊，需要新的編隊命令。落地前的延遲 Epoch 不得重新啟用任務。

不新增 Node、Topic、訊息或編隊 FSM。落地後不執行 optimizer、不發布有效本機控制軌跡；使用既有 local 與 global trajectory 的 valid=false 訊息撤銷舊顯示。依人類最新指定，已降落機體不保留佔位；同儕依有效且新鮮的 landed 狀態排除其編隊資格、避碰與軌跡 freshness 必要名單，並移除舊軌跡。觀測過期時不擅自延用「已降落」結論。BasicController 另依落地／解除解鎖觀測清除快取軌跡；節點拒絕地面或無效觀測期間到達的延遲有效軌跡，避免 DDS callback 時序重新填入快取。LAND／TAKEOFF 的 PX4 模式與命令時序維持既有實作。

失效情境：無效 PX4 狀態禁止規劃，但不當成已確認降落；確認落地或解除解鎖清除任務；DDS 延遲 Epoch 在新編隊命令前拒絕。相容性改變為地面移動／編隊命令不再排隊到起飛後執行。

驗證：Docker 內完成獨立 workspace 建置，以隔離 ROS_DOMAIN_ID 執行單元測試與節點級 ROS2 回歸：空中編隊 → 落地 → 舊 Epoch 重播 → 再起飛，預期不再產生有效本機軌跡；確認地面不再有有效 peer occupancy，已降落同儕不阻擋空中規劃，新的空中命令可重新啟動。使用獨立 build/install/log 路徑，不覆蓋目前人工 GCS 操作的執行檔，不重啟目前 SITL／Agent。完整 SITL 重飛需待人工操作結束後再部署。
