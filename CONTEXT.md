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

**EGO-Swarm 局部規劃器 (EGO-Swarm Local Planner)**：
每個 Agent 本地執行的軌跡規劃模組。以該 Agent 的 `planner_ned` 表示目前狀態、編隊槽位、PointCloud2 障礙物地圖與同儕未來軌跡，產生符合平滑性、動力學限制、障礙物 clearance 與同儕中心距離的時間化 B-spline 軌跡。規劃器只發布本機軌跡，不直接控制其他 Agent 的飛控。
_Avoid_：集中式全機隊控制器、直接發布 PX4 setpoint

**規劃局部 NED (Planner-local NED / `planner_ned`)**：
每個 Agent 以自己的 WGS84 Home 建立並 freeze 的公尺制局部 NED frame。它只供該 Agent 內部規劃與安全計算，不是所有 Agent 共用同一原點的座標系。跨 Agent 的狀態、軌跡與編隊資料以 WGS84 作為共同交換表示，接收端再投影至自己的 `planner_ned`。
_Avoid_：`swarm_ned`、共同 NED、全機群共用原點

**同儕未來軌跡 (Peer Future Trajectory)**：
Agent 透過 `/fmu_all_agent/swarm/agent_trajectories` 廣播的時間化 B-spline 軌跡。短暫缺少新資料時，沿用最後一份有效軌跡，不因資料年齡自動擴張安全半徑；中心距離門檻仍由任務參數決定。
_Avoid_：只代表目前位置的 AgentState

**局部障礙物地圖 (Local Obstacle Map)**：
由 `sensor_msgs/msg/PointCloud2` 建立、並與使用端 `planner_ned` 對齊的局部障礙物表示。PointCloud2 可以由真實感測器或 `ObstacleCloudSimulator` 產生；靜態障礙物與同儕未來軌跡分別處理。

**任務意圖 (Mission Intent)**：
GCS 或其他 Agent 命令轉換後的高階目標，例如移動點、取消編隊或編隊幾何。任務意圖只輸入 EGO-Swarm 規劃器，不直接寫入 PX4 控制 topic。

**Agent 狀態 (Agent State)**：
每個 Agent 對外發布的自身狀態資訊，包含 NED 位置、姿態、飛行模式、電量等。透過 `/fmu{N}_agent/state` topic 以 10Hz 發布。不同於 PX4 內部 odometry——這是 Agent 層級的抽象，隔離飛控實作細節。
_Avoid_：telemetry、PX4 state

**同儕發現 (Peer Discovery)**：
Agent 會定期掃描 ROS2 網路中所有 `/fmu*_agent/state` topic，動態建立或移除對其他 Agent 的訂閱，無需靜態設定檔。Agent 消失後不保留在活動同儕集合，再次出現時可重新加入。
_Avoid_：手動配置、靜態拓撲

**編隊 (Formation)**：
由 `FormationSlotPlanner` 將隊形幾何轉為每台 Agent 的虛擬編隊槽位。支援 CIRCLE、HORI_LINEAR 與 VERT_LINEAR；槽位依 Agent ID 決定，不要求 ID 連續。EGO-Swarm 只需追蹤自身槽位，避障與同儕避碰由本機局部規劃器處理。

**安全距離 (Collision Distance)**：
機體中心到機體中心的三維距離門檻，分為 `formation_min_distance_m` 與 `peer_collision_min_distance_m`。不設固定 3 公尺下限，可依機型與任務設定；不因軌跡資料年齡擴張。
_Avoid_：固定常數安全邊界、機體半徑加緩衝的混合語意

**沉默失效 (Silent Failure)**：
uXRCE-DDS 橋接 crash 或飛控斷線時，PX4 topic 停止更新但無 explicit 錯誤訊號。Agent 透過里程計 timeout（預設 1 秒）偵測此狀態，觸發降級程序（Position Hold → RTL）。
_Avoid_：連線中斷、通訊異常
