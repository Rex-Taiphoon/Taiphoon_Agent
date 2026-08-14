# AGENTS.md — src/nodes（ROS2 節點層）

## 目的

`src/nodes/` 是 ROS2 節點組裝層：把控制器、命令處理、狀態發布與安全監控組合成可執行的 `rclcpp::Node`，由 `src/main.cpp` 啟動。節點負責 topic 收發、參數載入、控制迴圈與生命週期，不包含業務演算法。

## 內容

| 檔案 | 角色 |
|------|------|
| `basic_controller_node.hpp/.cpp` | `BasicControllerNode`：整合 `BasicController`、`AgentCommandHandler`、`PositionSafetyMonitor`、`StatePublisher`，以固定頻率驅動（控制迴圈 10Hz、狀態發布 10Hz） |
| `ego_swarm_planner_node.hpp/.cpp` | `EgoSwarmPlannerNode`：組裝 EGO-Swarm 規劃、編隊、同儕軌跡與 trajectory safety，輸出 planner result |
| `obstacle_cloud_simulator_node.hpp/.cpp` | `ObstacleCloudSimulatorNode`：依 YAML 障礙物與選定 source mode 發布 PointCloud2 模擬資料 |

## 核心原則

- 節點只做「組裝與驅動」：trajectory / swarm / formation 演算法放 `src/ego_swarm/`，obstacle-map 演算法放 `src/perception/`；不要在節點內新增業務邏輯。
- Node 只做 ROS2 message、parameter、timer、QoS、資料轉換與模組組裝。
- 所有參數必須用 `declare_parameter` 宣告並指定預設值（L2-19）；初始化失敗必須記錄錯誤並中斷啟動流程。
- Publisher / Subscriber 必須以 `shared_ptr` 成員持有（L1-12），回呼命名以 `_cb` 結尾（L2-17）。
- 訂閱 PX4 資料（odometry / GPS / battery）必須檢查「已收到第一筆資料」旗標後才使用，避免以未初始化資料驅動控制。
- 節點命名空間與 topic 命名遵守根目錄 AGENTS.md 的 `/fmu{N}_agent/...` 規則；新增 topic 需同步 `docs/taiphoon-agent-interface.md`。

## 變更與驗證

- 修改後執行 `colcon build --packages-select taiphoon_agent`，並以 PX4 SITL 驗證節點啟動、topic 發布與命令流程。
