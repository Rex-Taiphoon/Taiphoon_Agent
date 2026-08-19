# AGENTS.md — src/core（核心介面層）

## 目的

`src/core/` 是 Taiphoon_Agent 的介面契約層，只定義抽象介面（純虛擬基底類別），不包含任何實作。所有飛行控制器（如 `BasicController`）都必須透過實作這些介面與外界互動。

目前包含兩組介面：
- `IController`：控制器生命週期（初始化、釋放、狀態查詢）。
- `IBasicFlightController`：基礎飛行命令（解鎖、起飛、降落、返航、移動、NED 目標等）。

## 核心原則

- **core 只放介面**：禁止在 `src/core/` 加入類別實作、演算法或節點；實作放在 `src/controllers/` 等對應資料夾。
- **不依賴 ROS2 / PX4 型別**：core 介面必須保持純 C++ 抽象（基本型別與標準函式庫），不得 include `rclcpp`、`px4_msgs`；飛控差異由實作層隔離，介面因此可獨立單元測試。
- **介面最小化（ISP）**：只定義實作確實需要的操作；新功能優先嘗試用既有介面組合達成，避免為單一實作膨脹介面。
- **向後相容優先**：擴充時優先新增方法並保留既有語意；變更或刪除既有方法視為破壞性變更。

## 變更紀律

修改 `src/core/` 的任何介面時，必須一次完成以下同步：
1. 更新所有實作類別（至少 `src/controllers/BasicController.cpp/.hpp`）。
2. 更新或新增對應單元測試（`test/`）。
3. 若影響對外行為，同步更新 `docs/taiphoon-agent-interface.md`
4. 以 `colcon build --packages-select taiphoon_agent` 與 `colcon test --packages-select taiphoon_agent` 驗證。

## 介面設計規範

- 介面類別命名以 `I` 開頭（L2-16），命名空間 `taiphoon::core`。
- 純虛擬函式宣告為 `virtual ... = 0;`，實作端覆寫加 `override`。
- 虛擬解構子、刪除複製語義（`= delete`）、允許移動語義（`= default`）。
- 公開函式必須有 Doxygen 注釋（`@brief` / `@param` / `@return`），說明「為什麼」而非「是什麼」。
- 命令類方法回傳 `bool` 表示「命令是否設定成功」；參數合法性（NaN、Inf、負值）由實作端檢查（L1-08）。
- 不修改狀態的方法宣告 `const`（L3-08）。
- 其餘命名、格式與安全規範一律遵守 `CODE_CONSTRAINT.md`，本檔不重複列出。

## 禁止事項

- 不要在介面中引入實作細節（具體傳輸協定、topic 名稱、硬體型號）。
- 不要為單一實作的需求新增只會被部分實作使用的方法。
- 不要修改 core 介面後只同步一個實作，未同步其他實作與測試。
