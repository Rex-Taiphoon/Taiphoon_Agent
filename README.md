# Taiphoon_Agent 🚁

**無人機任務與感測器執行中樞**

Taiphoon_Agent 是一套裝載於無人載具上的自主飛行控制中樞，基於 **ROS2 (Humble)** 與 **PX4 / Ardupilot** 飛控系統構建，專注於多機編隊協作、邊緣運算與即時影像處理。

---

## 核心特色

- **多機編隊控制** — 支援一對多或鏈式編隊架構，失效自動重組隊形
- **ROS2 原生通訊** — 透過 MicroROS / uXRCE-DDS 與飛行控制器通訊
- **邊緣運算整合** — 即時影像識別、感測器融合、避障與追蹤演算法
- **影像串流轉發** — 向地面控制站傳遞即時影像
- **Docker 容器化** — 跨平台可攜，可掛載至任何 Linux 核心主機

## 系統架構

\mermaid
flowchart RL
    GCS[地面控制站] <-->|ROS2 / QGC| Agent1[Taiphoon_Agent #1]
    Agent1 <-->|ROS2| FC1[PX4 / Ardupilot]
    Agent1 <-->|ROS2| Agent2[Taiphoon_Agent #2]
    Agent2 <-->|ROS2| FC2[PX4 / Ardupilot]
    Agent1 <-->|ROS2| Agent3[Taiphoon_Agent #N]
    Agent3 <-->|ROS2| FC3[PX4 / Ardupilot]
- 一個 Taiphoon_Agent 直接控制一臺飛控
- 編隊時由一個 Agent 向另一個 Agent 下達高階指令
- 對地面站的 ROS2 Topic 使用 \/fmu1_agent/xxx\ 格式

## 快速開始

### 需求

- Linux 核心主機（或 Docker 相容環境）
- Docker / Docker Compose
- PX4 SITL 模擬器（開發測試用）
- ROS2 Humble

### 啟動模擬

\ash
# 啟動 PX4 SITL 模擬（預設 3 臺無人機）
./skills/px4-sitl-launcher/scripts/launch_all.sh
## 核心任務

| 任務 | 說明 |
|------|------|
| MicroROS Agent | 負責 PX4 / Ardupilot 的 uXRCE-DDS 雙向橋接 |
| 自主飛行 | 編隊飛行、避障、導航等任務排程 |
| 邊緣運算 | 視覺辨識、感測器融合、即時定位 |
| 影像串流 | 向地面控制站轉發即時影像 |

## 開發規範

- **語言**：C++ 為主，必要時輔以 Python / C
- **編碼風格**：Allman 括號、Doxygen 註解、遵循 SOLID 原則
- **命名慣例**：請參閱 [CODE_CONSTRAINT.md](CODE_CONSTRAINT.md)
- **提交格式**：\模組: 更動項目\（簡要描述，100 字內）

## 授權條款

本專案為 Rex-Taiphoon 所屬，保留所有權利。
