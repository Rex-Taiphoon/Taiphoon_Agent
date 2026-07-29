---
name: ros2-tools
description: ROS2 指令工具與問題排除，所有指令透過 Docker 容器執行。用於：(1) 檢視與監控 ROS2 topics、services、actions，(2) 管理 ROS2 daemon 生命週期，(3) 檢查 ROS2 節點狀態與訊息內容，(4) 診斷 ROS2 通訊異常（如 topic 無法列出或訊息遺失），(5) 記錄與回放 ROS2 bag 資料。不適用於 uXRCE-DDS Agent 管理，請改用 uxrce_dds_agent-tools 技能；不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能。
---

# 功能名稱
ROS2 指令工具與問題排除（Docker 環境）

本技能提供在 Docker 環境下執行 ROS2 Humble 標準操作指令與問題排除流程。
所有指令透過 `docker compose exec taiphoon-agt` 在 Taiphoon_Agent 容器內執行，
容器內已預先設定 `/etc/profile.d/ros2.sh`，任何 shell（含非互動式）皆自動載入 ROS2 環境，
無需手動 source 或指定路徑變數。

主要功能涵蓋：topic 的列表、監聽與資訊查詢、node 的狀態檢查、service 與 action 的呼叫與列表、
ROS2 daemon 的啟動/停止/重啟管理、以及常見通訊異常的標準化排除流程。

# 前置條件
- Docker 已安裝並可執行
- Taiphoon_Agent 容器已啟動（`docker compose up -d`）
- ROS2 Humble 已於容器內安裝（由 Dockerfile 確保）

# 基本使用方式

所有指令在專案根目錄執行，格式統一為：
```
docker compose exec taiphoon-agt -- ros2 <子指令>
```

## 查詢主題列表
`docker compose exec taiphoon-agt -- ros2 topic list`

## 監聽主題訊息
`docker compose exec taiphoon-agt -- ros2 topic echo <topic名稱>`

## 查詢主題詳細資訊
`docker compose exec taiphoon-agt -- ros2 topic info <topic名稱>`

## 查詢節點列表
`docker compose exec taiphoon-agt -- ros2 node list`

## 查詢服務列表
`docker compose exec taiphoon-agt -- ros2 service list`

## 管理 ROS2 Daemon
`docker compose exec taiphoon-agt -- ros2 daemon stop`
`docker compose exec taiphoon-agt -- ros2 daemon start`

# 進階使用方式

## 確認特定節點的發布與訂閱關係
`docker compose exec taiphoon-agt -- ros2 node info <節點名稱>`

## 檢查 topic 發布頻率
`docker compose exec taiphoon-agt -- ros2 topic hz <topic名稱>`

## 進入容器互動式操作
`docker compose exec -it taiphoon-agt -- bash`

## ROS2 指令無回應時的標準處理流程
`docker compose exec taiphoon-agt -- bash -c "ros2 daemon stop && ros2 daemon start && ros2 topic list"`

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 本技能僅處理 ROS2 通訊層問題，不涉及特定中介軟體或硬體
- 所有 ROS2 指令必須透過 `docker compose exec taiphoon-agt` 在容器內執行
- 容器已預先設定 shell 環境，無需手動 source workspace
- 詳細指令參考請參閱 `references/ros2-commands.md`
- 問題排除指引請參閱 `references/ros2-troubleshooting.md`
