---
name: uxrce_dds_agent-tools
description: uXRCE-DDS Agent Docker 容器管理工具，用於 PX4 與 ROS2 之間的 DDS 通訊橋接。用於：(1) 啟動 MicroXRCEAgent 容器以橋接 PX4 與 ROS2，(2) 重新啟動特定實例的 Agent 容器，(3) 檢查 Agent 容器執行狀態，(4) 切換 RMW 實作（FastRTPS / CycloneDDS），(5) 診斷 uXRCE-DDS 通訊異常。不適用於 ROS2 指令操作，請改用 ros2-tools 技能；不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能。
---

# 功能名稱
uXRCE-DDS Agent Docker 容器管理

本技能負責管理 PX4 飛控與 ROS2 之間的 uXRCE-DDS 通訊橋接容器。MicroXRCEAgent 以獨立 Docker 容器運行（使用 `taiphoon/uxrce-agent` 映像），每個 PX4 實例對應一個 Agent 容器，使用對應的 UDP 通訊埠（8888 起遞增）。Agent 容器與 PX4 容器生命週期完全獨立，可單獨重啟、切換 RMW 或更換通訊埠。

主要功能涵蓋：啟動 Agent 容器、重啟異常的 Agent、檢查容器執行狀態、切換 RMW 實作，以及診斷 uXRCE-DDS 連線問題。

# 前置條件
- Docker 已安裝並可執行
- `taiphoon/uxrce-agent:latest` 映像已建構
- 目標 Docker 網路已存在（通常為 `Taiphoon_Net`）
- PX4 SITL 模擬器可選：Agent 可獨立啟動，PX4 後續連線即可

# 基本使用方式

## 啟動 Agent 容器
啟動一個 MicroXRCEAgent 容器，監聽指定埠號。
`bash scripts/start_agent.sh --name drone1-agent --rmw fastrtps --port 8888 --net Taiphoon_Net`

## 重新啟動 Agent 容器
當某個 Agent 連線異常時，重新建立該容器（停止舊容器後以相同參數啟動）。
`bash scripts/restart_agent.sh --name drone1-agent --rmw fastrtps --port 8888 --net Taiphoon_Net`

## 使用 CycloneDDS 啟動 Agent
切換至 CycloneDDS 中介軟體，適用於需要與特定 ROS2 節點相容的情境。
`bash scripts/start_agent.sh --name drone1-agent --rmw cyclonedds --port 8888`

## 手動啟動 Agent（不使用腳本）
直接使用 docker run 啟動，適合快速測試。
`docker run -d --rm --net=Taiphoon_Net --name=drone1-agent -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp taiphoon/uxrce-agent:latest udp4 -p 8888`

# 進階使用方式

## 檢查所有 Agent 容器狀態
確認所有 Agent 容器是否正常執行。
```bash
docker ps --filter "name=drone" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
```

## 強制關閉並重啟所有 Agent
當多個 Agent 同時異常時，批次清理並重新啟動。
```bash
docker ps --filter "name=drone" --format '{{.Names}}' | grep '\-agent$' | xargs -r docker stop
# 再透過 px4-sitl-launcher 重新啟動所有實例
```

## 查看 Agent 容器日誌
檢查特定 Agent 的執行日誌以診斷連線問題。
```bash
docker logs drone1-agent
docker logs -f drone1-agent  # 即時追蹤
```

## 切換執行中的 Agent RMW
若要變更執行中 Agent 的 RMW，需停止後以新參數重啟。
```bash
bash scripts/restart_agent.sh --name drone1-agent --rmw cyclonedds --port 8888
```

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 每個 PX4 實例對應一個獨立的 Agent 容器，通訊埠規則為 8888+N
- Agent 通訊埠必須與 PX4 實例的 `PX4_UXRCE_DDS_PORT` 設定一致
- Agent 容器與 PX4 容器生命週期獨立：重啟 Agent 不影響 PX4 運行
- RMW 切換僅影響 Agent 端，PX4 端不受影響
