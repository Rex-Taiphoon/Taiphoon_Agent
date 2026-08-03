# AGENTS.md — src/controllers（飛行控制實作層）

## 目的

`src/controllers/` 實作 `src/core/` 定義的介面，將高階飛行命令轉譯為 PX4 可執行的控制行為，是「core 契約 → PX4 動作」的實作層。

## 內容

| 檔案 | 角色 |
|------|------|
| `BasicController.hpp/.cpp` | 實作 `core::IBasicFlightController`，透過 PX4 Offboard 控制無人機；內含飛行狀態機 `FlightState` |
| `AgentCommandHandler.hpp/.cpp` | 訂閱 `/{agent_namespace}/command` 與 `/fmu_all_agent/command`，解析命令字串後以 callback 交回節點執行 |

## 核心原則

- 所有控制器必須實作 `src/core/` 介面，不自行定義平行的控制介面；core 介面變更時本模組必須同步（見 `src/core/AGENTS.md`）。
- 命令解析與執行職責分離：`AgentCommandHandler` 只負責解析與派發（`set_command_callback`），實際飛行動作由節點呼叫 `BasicController` 執行；不要在 handler 內直接呼叫控制器。
- 控制命令參數必須先驗證（NaN / Inf / 負值）再傳入控制器（L1-08）；`BasicController` 方法回傳 `bool` 表示命令是否設定成功。
- `FlightState` 是控制器核心狀態，新增狀態時需同步檢查所有狀態轉移與 `safety/StatePublisher` 的對應。
- 與 PX4 的資料交換統一透過 `px4_msgs` topic，禁止在此層直接撰寫 MAVLink 協定。

## 變更與驗證

- 修改命令字串格式時，必須同步更新 `src/utils/CommandTypes.hpp` 的解析器與 `docs/taiphoon-agent-interface.md` 的命令協定章節。
- 修改後至少執行 `colcon build --packages-select taiphoon_agent`，視影響範圍執行單元測試（位於 `test/`，gtest）與 PX4 SITL 驗證。
