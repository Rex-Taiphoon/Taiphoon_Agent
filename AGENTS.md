# AGENTS.md

## Project Overview

Taiphoon_Agent 是以 Docker 容器為部署單元的無人載具任務與感測器中樞。一個 Agent 容器對應一台飛控（PX4 或 ArduPilot），多機協作透過 Agent 對 Agent 指令進行，Agent 之間不直接控制彼此的飛控。

主要職責：
- 飛控橋接：與飛控通訊並執行飛行指令。
- 任務控制：執行避障、追蹤、編隊、定位等自主飛行任務。
- 軌跡規劃與安全監控：規劃飛行軌跡，監控飛行狀態並在異常時介入。
- 感測整合：整合機載感測器資料，對外發布一致的 Agent 狀態。
- 影像串流：處理即時影像並轉發給地面站。

這個專案的核心是「飛控通訊 + 任務控制 + 感測融合 + 影像資料流」的整合執行平台，所有資料鏈皆基於 ROS2。

## Repository structure

```TAIPHOON_AGENT/
├── src/ # 核心原始碼
├── msg/ # 自定義 ROS2 訊息
├── launch/ # ROS2 launch 啟動文件
├── test/ # 單元測試原始碼（.cpp）
│
├── docker/ # Docker 相關腳本
│ └── entrypoint.sh # 容器啟動進入點
│
├── docs/ # 開發文件
│ ├── taiphoon-agent-interface.md # Node / Topic 對外介面清單（需保持最新）
│ └── codex/ # EGO-Swarm / 機群規劃開發文件
│
├── build/ # 建置中間檔案
├── install/ # 安裝輸出目錄
├── log/ # 建置與測試紀錄
│
├── docker-compose.yml # 多容器服務定義
├── Dockerfile # 映像建置定義
├── CMakeLists.txt # CMake 建置設定
├── package.xml # ROS2 套件元資料
├── CODE_CONSTRAINT.md # 命名、排版與程式風格規範
├── CONTEXT.md # 專案背景與決策脈絡
├── CHARACTERS.md # 工程協作角色與跨模組決策規約
├── AGENTS.md # AI Agent 操作指引
└── README.md # 快速上手說明
```

## Engineering characters

- `CHARACTERS.md` 定義 Taiphoon_Agent 的工程協作角色、任務啟用方式與跨模組變更 Gate。
- 每個任務開始時先指定一個主角色；跨模組或安全關鍵工作再指定協作角色。
- `CHARACTERS.md` 不取代根目錄或子目錄的 `AGENTS.md`：角色文件負責協作責任，`AGENTS.md` 負責專案與模組技術規範。
- `CONTEXT.md` 僅負責領域詞彙與架構語言，不作為實作規格或角色規約使用。
- 任何 EGO-Swarm、機群編隊、trajectory planning、obstacle avoidance 相關工作開始前，先閱讀 `CONTEXT.md`、`docs/taiphoon-agent-interface.md`、對應子目錄的 `AGENTS.md` 及現有程式碼。
- 開發範圍以目前程式碼、介面文件與人類開發者明確指定的需求為準，不引用已不存在的歷史規劃文件。
- 規則衝突時，安全與人類確認要求優先，其次是本文件與對應子目錄 `AGENTS.md`，再依 `CHARACTERS.md` 補充跨模組協作行為。

## Topic & interface conventions

- 外部可見的 ROS2 topic 應盡量精簡，優先降低網路與頻寬負擔；非必要資訊請留在內部命名空間。
- 若飛控命名空間為 `/uav_{N}`：
  - 對外 Agent topic 一律使用 `/uav_{N}_agent/...`。
  - 所有 Agent 共用控制命令使用 `/uav_all_agent/...`。
- Taiphoon_Agent 產生的 Node 與 Topic 必須同步記錄在 `docs/taiphoon-agent-interface.md`，
  並保持最新狀態（新增／刪除任何 Node/Topic 時都要更新）。

## Boundaries(邊界條件)
**Always:**
- 所有變更僅限於 Taiphoon_Agent 專案本身，除非任務明確要求，不要修改外部依賴或替換既有基礎架構。
- 主要實作語言以C++為主、必要時才引入 Python/C
- 所有中文註解、文件與回覆都使用繁體中文。
- 命名、排版與其他程式風格規範請遵循 `CODE_CONSTRAINT.md`，不要在此檔重複描述。
- 與無人機、地面站及其他 Agent 的資料鏈一律基於 ROS2（Humble），不繞過 DDS 通訊層。
- 多機協作一律透過 ROS2/DDS 的 Agent-level topic/message 交換狀態、任務意圖、編隊資訊與未來軌跡；任何 Agent 不得直接發布控制命令至其他 Agent 所屬的飛控。
- 功能開發先規劃再動手：在修改程式之前，先寫下介面、資料流、失效情境與驗證方式。
- 維持模組低耦合：避免把飛控橋接、任務邏輯、感測融合、影像串流寫成互相纏繞的單體模組。
- 開發過程中任何單元測試應該在`test/`中實作
- `README.md`應該保持最新狀態
- 當`msg/`資料更新時，必須同步更新一份`msg/`裡面的介面檔案(.msg/.srv/.action)到與專案目錄同級的`Taiphoon_Agent_Interfaces`資料夾，包含`taiphoon-agent-interface.md`
- 當需要參閱舊架構時，可以參考`docs/codex/step1-data-flow-analysis.md`，未經人類開發者同意禁止修改

**Ask first:**
- 使用Git commit與push時，應確認提交的內容，使用 `模組: 更改項目` 格式，內容簡短明確。
- 修改一個功能時，如果會影響到其他功能的運行邏輯，請告知風險並由開發者決定是否繼續。
- 修改現有模組邊界時
- 當有出現檔案或資料夾刪除行為時

**Never:**
- 自行提交Git
- 在任何指令中使用 `--force`等強制命令
- 嘗試繞過平台的安全機制或人工審核流程；遇到需要確認的操作時，一律交給人類開發者決定。
- 單元測試禁止出現在`src/`原始碼中

## Build & Verification(測試與驗證)

### 環境需求
所有測試必須在 Docker 開發容器內執行。
禁止在 host 主機上直接執行 ROS2 指令。

開發映像不存在或需要更新時，AI Agent 可自行建置：
```bash
docker build --target taiphoon-dev -t taiphoon-agent:dev .
```

請先啟動環境：
```bash
docker compose up -d uav_1_agent
```

### 建置驗證
在執行任何測試之前，必須先完成 workspace 建置：
```bash
docker compose exec uav_1_agent bash -c "source /opt/ros/humble/setup.bash && colcon build"
```

### 單元測試（Unit Tests）
修改單一套件後，請執行該套件的測試：
```bash
docker compose exec uav_1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test --packages-select taiphoon_agent"
docker compose exec uav_1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"
```

僅在修改共用介面或核心模組時，才執行完整測試：
```bash
docker compose exec uav_1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"
docker compose exec uav_1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --all"
```

### 多機模擬驗證的快速啟動方法
以下指令僅限於多機模擬。啟動多機模擬驗證前，先啟動Agent，再啟動PX4模擬器，全部啟動完畢後後等待10秒後才能執行快速起飛

一次啟動多Taiphoon_Agent（目前預設為4台）
```bash
docker compose up -d
```
一次啟動多台PX4模擬器（目前預設為4台）
```bash
bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 4 --rmw cyclonedds
```
快速起飛
```bash
bash AutoTest/scripts/quick_takeoff.sh  
```

一次關閉多Taiphoon_Agent
```bash
docker compose down
```

一次關閉全部PX4模擬器
```bash
bash AutoTest/scripts/px4-sitl-launcher/stop_all.sh  
```
