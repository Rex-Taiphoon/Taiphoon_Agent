# AGENTS.md — src（原始碼總覽）

## 目的

`src/` 是 Taiphoon_Agent 的核心原始碼目錄，依模組資料夾組織。每個子資料夾都有自己的 AGENTS.md，本檔只說明整體結構與跨模組慣例。

## 模組地圖

| 資料夾 | 職責 |
|--------|------|
| `core/` | 介面契約層（純虛擬基底類別） |
| `controllers/` | 實作 core 介面，轉譯 PX4 飛行控制 |
| `nodes/` | ROS2 節點組裝層（topic、參數、控制迴圈） |
| `safety/` | 安全監控與狀態轉譯（純邏輯） |
| `formation/` | 任務規劃：同儕發現、編隊計算 |
| `trajectory/` | 軌跡規劃與 waypoint 發布 |
| `utils/` | 命令解析與數值驗證工具 |

## 跨模組慣例

- include 一律以 `src/` 為根：`#include "core/IBasicFlightController.hpp"`、`#include "utils/CommandTypes.hpp"`。
- 命名空間對應：節點類別統一放在 `taiphoon::nodes`（即使檔案位於 `formation/`、`trajectory/`），純邏輯類別使用模組命名空間（`taiphoon::formation`、`taiphoon::trajectory`、`taiphoon::safety`、`taiphoon::utils`）。
- 依賴方向：`core` ← `controllers` ← `nodes`；`utils` 與純邏輯類別不依賴 ROS2；節點類別不包含演算法。
- `src/main.cpp` 是 `BasicControllerNode` 進入點；`formation/`、`trajectory/` 各有獨立進入點（`main_mission_planner.cpp`、`main_trajectory_controller.cpp`）。
- 單元測試統一放在專案根目錄 `test/` 並使用 gtest（`ament_add_gtest`）；`src/` 內禁止放置測試原始碼。
- 修改任何模組前，先閱讀該模組的 AGENTS.md 與 `CODE_CONSTRAINT.md`。
