# Taiphoon_Agent

Taiphoon_Agent 以 Docker 容器為部署單元部署於任務電腦（如樹莓派）上。一個 Agent 容器對應一台 PX4 或 ArduPilot 飛控，容器內含完整軟體堆疊（uXRCE-DDS 橋接 + 任務規劃 + 軌跡控制 + 位置監控）。多機協作透過 Agent 對 Agent 指令進行。

## Language

**Agent**：
部署於任務電腦上的完整軟體執行個體，包含飛控橋接、任務規劃、軌跡控制與位置監控等模組。一個 Agent 只對應一台飛控。
_Avoid_：節點、程序、controller

**Agent 容器 (Agent Container)**：
Docker 容器為 Agent 的部署邊界，一個容器對應一台飛控。容器內含 uXRCE-DDS 橋接 daemon 與所有 Taiphoon_Agent ROS2 節點。容器命名遵循 `fmu{N}_agent` 慣例（如 `fmu1_agent`），與 ROS2 topic namespace 一致。
_Avoid_：service instance、pod

**uXRCE-DDS 橋接 (uXRCE-DDS Bridge)**：
內嵌於 Agent 容器中的 MicroXRCEAgent daemon，負責將 PX4 uORB 訊息橋接至 ROS2 DDS 網路。與 Taiphoon_Agent 節點同生命週期，透過 entrypoint 確保啟動順序（先橋接、後節點）。僅使用 UDP 傳輸與飛控通訊。
_Avoid_：uXRCE agent、DDS agent（應明確為 uXRCE-DDS 橋接）

**飛控橋接層 (PX4 Bridge)**：
Agent 與 PX4 飛控之間的通訊層，負責 Offboard 模式啟用、Arm/Disarm、模式切換，以及將 NED 目標位置轉譯為 PX4 TrajectorySetpoint。
_Avoid_：驅動層、硬體層

**位置控制層 (Position Controller)**：
監控 PX4 Offboard 飛行狀態的安全網。不自行計算位置控制輸出（該工作由 PX4 內部控制器負責），而是監控偏差、偵測異常，必要時介入（如強制懸停或模式切換）。
_Avoid_：PID 控制器、姿態控制

**軌跡控制層 (Trajectory Controller)**：
接收目標 NED 位置，規劃符合安全距離的路徑並輸出帶時間戳的 waypoint 序列。第一版輸出的 `/waypoints` 僅供路徑規劃、觀測與後續整合使用，目前不是 PX4 的實際控制來源；不得把 waypoint 輸出視為已對飛控生效的防撞保證。預留外部避障節點介面。
_Avoid_：路徑規劃（路徑規劃是軌跡控制的子功能）

**觀測式防撞 (Observation-based Collision Avoidance)**：
第一版多機防撞策略。依各 Agent 最近發布的有效位置，檢查自身移動路徑與其他機體是否維持至少 3 公尺的三維距離。BasicController 執行控制目標前的防撞關卡是送往 PX4 前的最終控制關卡；發生衝突、仍被探索到的同儕狀態無效，或 GPS 無法轉換至可比較的共同座標時，維持目前位置並保留待執行目標，條件恢復安全後自動續行。Agent 從 ROS2 discovery 消失時，立即退出編隊與防撞集合，該機後續行為交由 PX4 failsafe 處理，其餘 Agent 直接重新編隊。此版本不使用速度或未來軌跡預測，不能視為時間域碰撞預測。
_Avoid_：動態避障、速度預測、PX4 內建避障

**任務規劃層 (Mission Planner)**：
最高層決策模組。解析編隊命令（如 FORMATION_CIRCLE）、自動發現線上有哪些同儕 Agent、根據所有 Agent 狀態計算自身在編隊中的目標位置。發布目標 NED 供軌跡控制層規劃與 BasicController 控制鏈使用；實際送往 PX4 前仍須通過 BasicController 執行前的防撞關卡。
_Avoid_：排程器、任務佇列

**Agent 狀態 (Agent State)**：
每個 Agent 對外發布的自身狀態資訊，包含 NED 位置、姿態、飛行模式、電量等。透過 `/fmu{N}_agent/state` topic 以 10Hz 發布。不同於 PX4 內部 odometry——這是 Agent 層級的抽象，隔離飛控實作細節。
_Avoid_：telemetry、PX4 state

**同儕發現 (Peer Discovery)**：
Agent 會定期掃描 ROS2 網路中所有 `/fmu*_agent/state` topic，動態建立或移除對其他 Agent 的訂閱，無需靜態設定檔。Agent 消失後不保留在活動同儕集合，再次出現時可重新加入。
_Avoid_：手動配置、靜態拓撲

**編隊 (Formation)**：
任務規劃層功能之一。根據編隊類型（環形 CIRCLE、水平直列 HORI_LINEAR、垂直直列 VERT_LINEAR）與參數（半徑/間距、高度），計算自身在編隊中的目標 NED 位置。

**安全邊界 (Safety Margin)**：
第一版觀測式防撞要求兩機之間維持的最小三維距離，固定為至少 3 公尺。此邊界同時適用於編隊目標與一般移動目標；是否允許目標送往 PX4，最後由 BasicController 執行前的防撞關卡判定。
_Avoid_：緩衝區、禁航區

**沉默失效 (Silent Failure)**：
uXRCE-DDS 橋接 crash 或飛控斷線時，PX4 topic 停止更新但無 explicit 錯誤訊號。Agent 透過里程計 timeout（預設 1 秒）偵測此狀態，觸發降級程序（Position Hold → RTL）。
_Avoid_：連線中斷、通訊異常
