# ADR-0001：Agent-to-Agent 狀態發布架構

**狀態**：accepted

## 背景

Taiphoon_Agent 需支援多機編隊與協作，每個 Agent 必須感知其他無人機的位置與狀態才能計算編隊目標。目前實作中 `FormationNode` 直接訂閱所有飛控的 PX4 原生 topic（`/fmu{N}/fmu/out/vehicle_odometry`），此做法使 Agent 跨越「一 Agent 一機」邊界，也將 Agent 耦合至 PX4 內部 topic 命名與資料格式。

## 決策

每個 Agent 自行發布 `AgentState` 和 `AgentAttitude` 至 `/fmu{N}_agent/state` 與 `/fmu{N}_agent/attitude` topic，其他 Agent 透過同儕發現機制動態訂閱這些 topic，不再直接存取任何 `/fmu{N}/fmu/out/*` 原生 topic。

## 替代方案

| 方案 | 理由 |
|------|------|
| 直接訂閱 PX4 odometry（現狀） | 耦合 PX4 內部 topic 名稱與格式；PX4 升級時所有 Agent 都需修改；破壞一 Agent 一機邊界 |
| 集中式狀態伺服器 | 增加單點故障；不符 ROS2 去中心化設計精神；多一跳增加延遲 |

## 後果

- 需定義新自訂訊息（`AgentState.msg`、`AgentAttitude.msg`）
- `BasicControllerNode` 需增加狀態發布責任
- `FormationNode`（未來重構為任務規劃層）需實作同儕發現機制
- 編隊命令 topic `/fmu_all_agent/command` 維持不變
