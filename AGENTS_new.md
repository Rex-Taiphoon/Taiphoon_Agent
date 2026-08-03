# AGENTS.md

## Project Overview

Taiphoon_Agent 是部署在 Linux 主機上的無人載具任務與感測器中樞，負責：
- 與 PX4 或 ArduPilot 飛控橋接通訊。
- 執行自主飛行任務，例如避障、追蹤、編隊與定位。
- 整合視覺與其他感測器的邊緣運算。
- 處理即時影像串流與對地面站的資料轉發。

這個專案的核心不是單一演算法，而是「飛控通訊 + 任務控制 + 感測融合 + 影像資料流」的整合執行平台。

## Non-obvious constraints

- 預設控制目標為四旋翼機。
- 所有服務必須可部署於 Docker，避免實作直接綁定單一宿主環境。
- 與無人機、地面站及其他 Agent 的資料鏈必須基於 ROS2。
- 一個 Taiphoon_Agent 只直接控制一台 PX4 或 ArduPilot；多機協作應透過 Agent 對 Agent 指令，不應跨越此邊界直接控多台飛控。
- 外部可見 topic 應盡量精簡，優先降低網路與頻寬負擔。
- 若飛控命名空間為 `/fmu1`，則對外 Agent topic 使用 `/fmu1_agent/...`；所有 Agent 共用控制命令使用 `/fmu_all_agent`。
- Taiphoon_Agent產生的Node與Topic需要記錄在`docs/taiphoon-agent-interface.md`，並且保持最新狀態

## Engineering defaults

- 主要實作語言為 C++。
- 只有在 C++ 不適合，或需要大量依賴外部生態時，才引入 Python、C。
- 所有中文註解、文件與回覆都使用繁體中文。
- 功能開發先規劃再動手；先定義介面、資料流、失效情境與驗證方式，再修改程式。
- 模組之間應維持低耦合，避免把飛控橋接、任務邏輯、感測融合、串流流程寫成互相纏繞的單體。
- 命名、排版與其他程式風格規範請遵循 `CODE_CONSTRAINT.md`，不要在此檔重複描述。

## Boundaries(邊界條件、禁止事項)

- 所有變更僅限於 Taiphoon_Agent 專案本身，除非任務明確要求，不要修改外部依賴或替換既有基礎架構。
- Git commit message 使用 `模組: 更改項目` 格式，內容簡短明確，不得超過 100 字。
- 若是大型或高風險單一功能修改，優先使用獨立分支；分支命名格式為 `xxx-dev`，且名稱長度不超過 20 字元。
- 禁止自行建構Docker，如果必須構建，必須停止任務並且提供指令讓使用者自行構建。

## Build & Verification(測試與驗證)

### 環境需求
所有測試必須在 Docker 開發容器內執行。
禁止在 host 主機上直接執行 ROS2 指令。

請先啟動環境：
```bash
docker compose up -d fmu1_agent
```

### 建置驗證
在執行任何測試之前，必須先完成 workspace 建置：
```bash
docker compose exec fmu1_agent colcon build
```

如果只修改單一套件，請使用局部建置：
```bash
docker compose exec fmu1_agent colcon build --packages-select taiphoon_agent
```

### 單元測試（Unit Tests）
修改單一套件後，請執行該套件的測試：
```bash
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test --packages-select taiphoon_agent"
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"
```

僅在修改共用介面或核心模組時，才執行完整測試：
```bash
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --verbose"
docker compose exec fmu1_agent bash -c "source /opt/ros/humble/setup.bash && colcon test-result --all"
```

## Working style for agents

- 優先先理解目前任務影響哪一層：飛控橋接、任務控制、感測融合、影像串流、或多機協作。
- 避免一次同時改動多個子系統，除非任務本身就是跨模組重構。
- 完成修改後，先說明變更內容、影響模組、驗證方式與未處理風險，再進入下一步。
