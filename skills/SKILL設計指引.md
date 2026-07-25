
# Taiphoon_Agent的Skill開發原則
- 每個Skill都專責一個子任務
- Skill是為了確保的開發、測試、驗證必須在同一個標準上，因此將以指令與使用邏輯為根本，讓Agent可以在需要時調用該指令
- 所有Skill是建立在Codex上的，必須能夠被Codex顯式或隱式調用
- 全部的Skill的描述應該以繁體中文且非中國用語撰寫，**Skill名稱**應該用英文撰寫，並且以`-`分隔與區分功能，用`_`分隔功能名稱間隔，例如:
    - `px4-sitl-launcher`：px4啟動sitl的Skill
    - `px4-env-builder`：建構px4環境的Skill
    - `ros2-env-builder`：建構ros2環境的Skill
    - `ros2-tools`：ros2相關指令的Skill
    - `uxrce_dds_agent-env-builder`：建構uxrce-dds-agent環境的Skill
    - `uxrce_dds_agent-tools`：uxrce-dds-agent相關指令的Skill


# Taiphoon_Agent的Skill框架
**Taiphoon_Agent的Skill包含以下結構**
### SKILL.md:：指令 + 元資料 (YAML frontmatter)
```
---
name: Skill名稱，應該以英文撰寫，命名規範參考上方Taiphoon_Agent的Skill開發原則
description: 描述Skill的功能。主要是給Agent看何時觸發這個功能，必須要涵蓋目前這個Skill的全部功能，功能描述必須要專一且明確，總長度不得超過500字
---
# 功能名稱
以800字內功能說明這個Skill的功能

# 前置條件
寫出執行這個Skill的先決條件，包含需要的環境以及套件，以及在使用這個Skill前需要先執行的指令

# 基本使用方式
應該在這邊寫出所基本指令或腳本的使用方式，目的在於定義基本的指令集與腳本檔案

## 功能A
簡單描述此功能的內容
`指令或腳本名稱`

## 功能B
簡單描述此功能的內容
`指令或腳本名稱`

## 功能C
簡單描述此功能的內容
`指令或腳本名稱`

# 進階使用方式
應該提供在特定使用情境下如何組合指令的範例，目的在於減少編寫大量的複雜腳本檔案，應該讓Agent藉由使用場景的指引來自行組合指令

## 使用場景B
簡單描述此使用場景，並簡單描述為甚麼會需要使用到基本指令
`包含功能A、功能B組合的腳本`

# 開發原則限制
- 應遵照AGENTS.md的指引，避免偏離核心開發原則
```

### scripts/：可執行腳本（Python、Bash或Shell等腳本檔案）
這邊理論上只能擺放基本的執行腳本，除非必要才需要建置，可以由指令行運行就不需要建立腳本。複雜的使用場景應該由Agent自行組合Skill提供的基本指令與腳本，例如
- stop_all.sh: 關閉所有SITL模擬器

### references/：補充文件、說明
這邊應該擺放跟這個Skill有關的相關設定文件或網頁
- port_mapping.md: 紀錄開啟SITL時的port與IP相關設定原則

### assets/：模板、JSON schema、設定檔
這邊應該要放預設設定檔等資源，非必要可以保持空白即可

### agents/openai.yaml:
這裡應該要放顯示給Codex的介面
```
interface:
  display_name: "目前這個Skill的名稱"
  short_description: "請用50個字以內描述這個Skill的功能"
```



# 目前Taiphoon設計的Skill列表

## ROS2工具
**使用ros2相關命令列時常用的工具，可以在使用過程中自行擴充**
- 功能：基本指令，查詢`ros2 topic list`等
- 功能：除錯指令，重新啟動，`ros2 daemon status`與`ros2 daemon start`
- 使用場景：如果ros2指令無回應應該自動重新啟動daemon

## uXRCE-DDS-Agent工具
**使用uXRCE-DDS-Agent相關命令列時常用的工具，可以在使用過程中自行擴充**
- 功能：基本啟動指令

## PX4環境建立工具
**在沒有PX4的環境下自動建立PX4環境的工具**
- 功能：使用SubAgent自動建立，並且把PX4原始碼建立在家目錄（~）底下
參考資源1：https://docs.px4.io/main/en/dev_setup/building_px4


## PX4模擬工具(px4-sitl-launcher)
**PX4的SITL模擬工具**
- 功能：基礎單機模擬腳本，啟動時不啟用Gazebo或其他視覺化模擬工具
- 功能：啟用多機模擬器腳本
- 功能：關閉多機模擬器腳本
