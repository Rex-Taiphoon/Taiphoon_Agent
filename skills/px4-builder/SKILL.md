---
name: px4-builder
description: 建構 PX4 原始碼與執行環境。用於：(1) 從原始碼編譯 PX4-Autopilot 韌體，(2) 安裝 PX4 建構所需之相依套件與工具鏈，(3) 設定 WSL2 Ubuntu 22.04 之開發環境，(4) 安裝 MicroXRCEAgent 以支援 ROS2 通訊，(5) 驗證建構結果與執行環境是否正確。不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能；不適用於 ROS2 問題排除，請改用 ros2-tools 技能。
---

# 功能名稱
PX4 環境建構工具

本技能負責在 WSL2 Ubuntu 22.04 環境中，從原始碼建置完整的 PX4-Autopilot 開發與模擬環境。原始碼將克隆至 `$HOME/PX4-Autopilot` 目錄（與 `Taiphoon_Agent` 專案目錄平級），包含相依套件安裝、工具鏈設定、SITL 韌體編譯、MicroXRCEAgent 安裝，以及建構結果驗證。

# 前置條件
- WSL2 Ubuntu 22.04 環境
- 至少 10GB 可用磁碟空間（存放 PX4 原始碼與建構產物）
- 穩定的網路連線（需從 GitHub 克隆原始碼與子模組）
- 建議設定 WSL2 記憶體上限至少 8GB（於 `%USERPROFILE%\.wslconfig` 設定）

# 基本使用方式
以下為本技能提供的基本腳本，依序執行即可完成完整環境建置。

## 安裝相依套件與工具鏈
安裝 PX4 建構所需的所有系統套件、Python 相依套件與 ROS2 Humble。
`bash scripts/setup_env.sh`

## 建構 PX4 韌體
從 GitHub 克隆 PX4-Autopilot 原始碼並編譯 `px4_sitl` target。
`bash scripts/build_px4.sh [分支名稱]`

## 安裝 MicroXRCEAgent
安裝 MicroXRCEAgent 以支援 PX4 與 ROS2 之間的 uXRCE-DDS 通訊橋接。
`bash scripts/setup_env.sh --install-agent`

## 驗證建構結果
檢查 PX4 二進位檔、MicroXRCEAgent、ROS2、tmux 與 git 是否正確安裝。
`bash scripts/validate_build.sh`

# 進階使用方式
以下為常見使用場景的指令組合方式，由 Agent 根據需求自行組合基本腳本。

## 完整從零建置流程
在全新 WSL2 環境中執行完整建置流程。
`bash scripts/setup_env.sh --install-agent && bash scripts/build_px4.sh && bash scripts/validate_build.sh`

## 僅重新編譯（不重新克隆）
當 PX4 原始碼有更新或需清除舊有建構快取時使用。
`cd ~/PX4-Autopilot && make clean && make px4_sitl -j$(nproc)`

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 原始碼目錄固定於 `$HOME/PX4-Autopilot`，不可變更
- 建構目標為 `px4_sitl_default`，不建構 Gazebo 相關 target
- 相依套件安裝腳本應具備冪等性（idempotent），可重複執行
- 詳細相依套件清單請參閱 `references/prerequisites.md`
- 建構參考資訊請參閱 `references/build-reference.md`