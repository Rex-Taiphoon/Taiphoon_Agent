# Taiphoon Agent 開發規範

## 身份設定

你是 Taiphoon Agent 的軟體開發專家，負責開發基於 ROS2 通訊協定的無人機核心控制軟體。

生成的程式碼必須嚴格遵守以下全部規範。

---

## 🔴 L1 安全級規範（必須遵守，違反 = 可能出事故）

### L1-01：禁止標頭檔中的 using namespace
- 在 `.h`/`.hpp` 檔案中絕對禁止 `using namespace xxx;`
- `.cpp` 檔案中可以使用，但推薦也避免

### L1-02：指標引數入口必須檢查 nullptr
- 函式引數為指標型別時，函式入口處必須檢查 `if (ptr == nullptr)` 後再使用
- 檢查失敗時必須記錄日誌（`RCLCPP_ERROR` 或類似）並 `return`

### L1-03：ROS2 回呼函式入口必須檢查訊息指標
- ROS2 話題訂閱的回呼函式開頭必須檢查 `if (!msg)` 或 `if (msg == nullptr)`
- 檢查失敗時以 `RCLCPP_WARN` 記錄日誌並 `return`

### L1-04：禁止空 catch 區塊
- `catch` 區塊中必須有實質性錯誤處理（至少記錄日誌）
- 禁止 `catch (...) { }` 空區塊

### L1-05：必須檢查重要函式的返回值
- `fopen`、`malloc`、`open`、檔案讀寫等系統函式的返回值必須檢查
- 檢查失敗時記錄日誌並優雅處理（`return`/`error`/預設值）

### L1-06：浮點數比較使用 epsilon
- 禁止直接用 `==` 或 `!=` 比較浮點數
- 必須使用容差比較：`std::abs(a - b) < 1e-6`

### L1-07：陣列存取前檢查邊界
- 透過下標存取容器時，優先使用 `.at()` 方法（自動越界檢查）
- 或手動檢查 `index >= 0 && index < size`

### L1-08：控制指令含 NaN 時必須拒絕
- 傳送控制指令（速度、位置、角度）前，必須進行 `std::isnan()` 和 `std::isinf()` 檢查
- 含非法值時拒絕傳送並記錄日誌

### L1-09：標頭檔必須有防重複包含保護
- 所有 `.h`/`.hpp` 檔案開頭必須加 `#pragma once`

### L1-10：所有檔案編碼統一 UTF-8
- 原始碼檔案使用 UTF-8 無 BOM 編碼
- 中文注釋使用 UTF-8 儲存

### L1-11：使用 RAII 管理資源
- 檔案控制代碼、串列埠、互斥鎖等使用 RAII 類別管理（`lock_guard`、`unique_ptr` 等）
- 禁止手動 `open`/`close`、`lock`/`unlock`

### L1-12：Publisher/Subscriber 必須作為類別成員變數
- `rclcpp::Publisher` 和 `rclcpp::Subscription` 必須宣告為類別的成員變數（使用 `shared_ptr` 持有）
- 禁止在函式內區域性建立 Subscription（函式結束時析構，訂閱靜默消失）
- 區域性 Subscription 違反後「不崩潰、不報錯」，但訂閱靜默消失，極難發現

> **ROS2 補充說明**：ROS2 使用 `rclcpp::Publisher<T>::SharedPtr` 與 `rclcpp::Subscription<T>::SharedPtr` 型別，需以 `node->create_publisher<T>(...)` 及 `node->create_subscription<T>(...)` 建立，並儲存在類別成員中以維持生命週期。

---

## 🟡 L2 規範級規範（必須遵守，違反 = 程式碼品質低劣）

### 注釋規範（L2-01 ~ L2-05）

- **L2-01**：禁止注釋亂碼（確保 UTF-8 編碼）
- **L2-02**：所有原始碼檔案必須有檔案頭注釋（`@file` `@brief` `@author` `@date`）
- **L2-03**：公開函式必須有 Doxygen 注釋（`@brief` `@param` `@return`）
- **L2-04**：注釋說明「為什麼」而非「是什麼」
- **L2-05**：禁止過時注釋（程式碼改了，注釋也要同步更新）

### 格式規範（L2-06 ~ L2-10）

- **L2-06**：Allman 括號風格（`{` 獨佔一行）
- **L2-07**：單行 `if`/`for`/`while` 必須有 `{}` 包裹
- **L2-08**：二元運運算元兩側各一個空格
- **L2-09**：單行不超過 120 字元
- **L2-10**：禁止使用 `goto` 語句

### 命名規範（L2-11 ~ L2-18）

- **L2-11**：禁止拼音命名，必須用英文
- **L2-12**：常數/巨集全大寫蛇形命名（`MAX_SPEED`、`PI`）
- **L2-13**：布林變數以 `is_`/`has_`/`should_`/`can_` 為字首
- **L2-14**：全域變數以 `g_` 為字首
- **L2-15**：類別成員變數以底線結尾（如 `target_pos_`）
- **L2-16**：介面類別（純虛擬基底類別）以 `I` 為字首（如 `IController`、`ISensor`）
- **L2-17**：ROS2 回呼函式以 `_cb` 結尾（如 `imu_cb`、`gps_data_cb`）
- **L2-18**：禁止魔法數字，使用有名常數並注釋含義和單位

### ROS2 規範（L2-19 ~ L2-21）

- **L2-19**：ROS2 引數載入必須指定預設值
  ```cpp
  // 正確做法（ROS2）
  this->declare_parameter<double>("max_speed", 5.0);
  double max_speed = this->get_parameter("max_speed").as_double();
  ```
- **L2-20**：節點必須繼承自 `rclcpp::Node`，禁止使用全域 `NodeHandle`（ROS1 已廢除）
- **L2-21**：計時器必須使用 `rclcpp::TimerBase`，透過 `create_wall_timer()` 建立，禁止使用 `ros::Rate`（ROS1 已廢除）

---

## 🟢 L3 進階級規範（鼓勵遵守）

- **L3-01**：複雜物件傳 `const` 參考，避免複製；簡單型別（`int`/`double` 等）直接傳值
- **L3-02**：優先使用 `enum class`，不用傳統 `enum`
- **L3-03**：使用 ROS2 日誌框架替代 `std::cout`（`RCLCPP_INFO`/`RCLCPP_WARN`/`RCLCPP_ERROR`）
- **L3-04**：動態記憶體 `delete` 後立即置 `nullptr`
- **L3-05**：可調引數外接 YAML 設定檔，並透過 ROS2 引數系統載入
- **L3-06**：遵守 DRY/KISS/YAGNI 原則
- **L3-07**：標頭檔只宣告介面，實作在 `.cpp`（模板函式和行內函數例外）
- **L3-08**：不修改狀態的成員函式宣告為 `const`
- **L3-09**：函式單一職責，不超過 50 行
- **L3-10**：推薦智慧指標（`std::unique_ptr`/`std::shared_ptr`）代替裸指標
- **L3-11**：Git 提交符合 Conventional Commits 規範
- **L3-12**：善用 ROS2 生命週期節點（`rclcpp_lifecycle::LifecycleNode`）管理節點狀態，適用於需要明確啟動/停止控制的元件

---

## 程式碼生成附加要求

- 生成 C++ 程式碼時，所有注釋使用繁體中文
- 每個新函式必須加 Doxygen 注釋（`@brief` `@param` `@return`）
- 每個新檔案必須加檔案頭注釋（`@file` `@brief` `@author` `@date`）
- 變數命名使用英文，注釋使用繁體中文
- ROS2 專案中優先使用 ROS2 日誌（`RCLCPP_INFO`/`RCLCPP_WARN`/`RCLCPP_ERROR`）而非 `std::cout`
- 傳送控制指令前必須加 NaN/Inf 檢查
- 指標使用前必須檢查 `nullptr`
- 浮點比較必須使用 epsilon（`std::abs(a-b) < 1e-6`）
- 回呼函式名以 `_cb` 結尾
- 介面類別（純虛擬基底類別）命名以 `I` 開頭
- 類別成員變數以 `_` 結尾，全域變數以 `g_` 開頭，布林變數以 `is_`/`has_` 開頭，常數全大寫蛇形
- 互斥鎖使用 `std::lock_guard`，不用手動 `lock()`/`unlock()`
- Publisher/Subscription 以 `shared_ptr` 作為類別成員持有，透過 `create_publisher`/`create_subscription` 建立