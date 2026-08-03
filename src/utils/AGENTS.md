# AGENTS.md — src/utils（共用工具）

## 目的

`src/utils/` 存放跨模組共用的無狀態工具：高階命令字串解析與數學 / 數值驗證輔助。此層不依賴 ROS2，也不得依賴其他模組。

## 內容

| 檔案 | 角色 |
|------|------|
| `CommandTypes.hpp` | `AgentCommand` 列舉、`ParsedCommand` 結構、`parse_command_string()`（inline）、`agent_command_to_string()` |
| `math_utils.hpp` | `almost_equal()`（epsilon 比較）、`is_valid_float()` / `is_valid_double()`（NaN / Inf 檢查） |

## 核心原則

- 保持無狀態與無依賴：所有函式為 inline 或靜態，不得 include `rclcpp` 或專案其他模組標頭。
- 命令協定是「地面站 ↔ Agent」的對外契約：新增 / 修改命令時必須同步 `docs/taiphoon-agent-interface.md` 的命令協定章節，並保持簡寫向後相容（如 `FORWARD` ↔ `MOVE_FORWARD`）。
- `is_valid_*` 是 L1-08 的實作基礎，控制指令送達飛控前必須使用。
- 解析失敗必須回傳明確結果（`UNKNOWN` + 參數重設），禁止拋例外或回傳未初始化結構。

## 變更與驗證

- 修改後執行 `colcon build --packages-select taiphoon_agent`，並確認所有依賴本模組的模組（`controllers/`、`formation/`、`nodes/`）仍可編譯。
