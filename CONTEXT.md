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
接收目標 NED 位置，規劃無碰撞路徑並輸出帶時間戳的 waypoint 序列。負責處理多機路線重疊與安全邊界（≥3m）。預留外部避障節點介面。
_Avoid_：路徑規劃（路徑規劃是軌跡控制的子功能）

**任務規劃層 (Mission Planner)**：
最高層決策模組。解析編隊命令（如 FORMATION_CIRCLE）、自動發現線上有哪些同儕 Agent、根據所有 Agent 狀態計算自身在編隊中的目標位置。輸出目標 NED 給軌跡控制層。
_Avoid_：排程器、任務佇列

**Agent 狀態 (Agent State)**：
每個 Agent 對外發布的自身狀態資訊，包含 NED 位置、姿態、飛行模式、電量等。透過 `/fmu{N}_agent/state` topic 以 10Hz 發布。不同於 PX4 內部 odometry——這是 Agent 層級的抽象，隔離飛控實作細節。
_Avoid_：telemetry、PX4 state

**同儕發現 (Peer Discovery)**：
Agent 啟動時自動掃描 ROS2 網路中所有 `/fmu*_agent/state` topic，動態建立對其他 Agent 的訂閱，無需靜態設定檔。
_Avoid_：手動配置、靜態拓撲

**編隊 (Formation)**：
任務規劃層功能之一。根據編隊類型（環形 CIRCLE、水平直列 HORI_LINEAR、垂直直列 VERT_LINEAR）與參數（半徑/間距、高度），計算自身在編隊中的目標 NED 位置。

**安全邊界 (Safety Margin)**：
軌跡規劃時兩機路徑之間的最小允許距離，預設 ≥ 3 公尺。
_Avoid_：緩衝區、禁航區

**沉默失效 (Silent Failure)**：
uXRCE-DDS 橋接 crash 或飛控斷線時，PX4 topic 停止更新但無 explicit 錯誤訊號。Agent 透過里程計 timeout（預設 1 秒）偵測此狀態，觸發降級程序（Position Hold → RTL）。
_Avoid_：連線中斷、通訊異常
