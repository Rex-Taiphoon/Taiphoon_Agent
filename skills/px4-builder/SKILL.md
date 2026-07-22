---
name: px4-builder
description: "建構 PX4 原始碼與執行環境。用於：(1) 從原始碼編譯 PX4-Autopilot 韌體，(2) 安裝 PX4 建構所需之相依套件與工具鏈，(3) 設定 WSL2 Ubuntu 22.04 之開發環境，(4) 安裝 MicroXRCEAgent 以支援 ROS2 通訊，(5) 驗證建構結果與執行環境是否正確。不適用於啟動 PX4 SITL 模擬器，請改用 px4-sitl-launcher 技能；不適用於 ROS2 問題排除，請改用 ros2-tools 技能。"
---

# PX4 建構工具

本技能協助您從原始碼建構 PX4-Autopilot 韌體並設定執行環境。PX4 原始碼將複製到 `$HOME/PX4-Autopilot` 目錄下，與 `Taiphoon_Agent` 目錄平行。

## 前置需求

- WSL2 Ubuntu 22.04 環境
- 足夠的磁碟空間（建議至少 10GB）
- 穩定的網路連線（用於下載相依套件）

## 技能目錄結構

```
skills/px4-builder/
├── SKILL.md
├── agents/openai.yaml
├── references/
│   ├── prerequisites.md       # 前置需求說明
│   └── build-reference.md     # 建構參考指南
├── assets/
└── scripts/
    ├── setup_env.sh           # 安裝系統相依套件
    ├── build_px4.sh           # 複製原始碼 + 編譯 SITL
    └── validate_build.sh      # 驗證建構結果
```

## 建構步驟

### 步驟 1：安裝系統相依套件

```bash
bash scripts/setup_env.sh
```

此腳本會安裝 PX4 建構所需的編譯工具、Python 套件與其他相依程式庫。

### 步驟 2：建構 PX4 SITL

```bash
bash scripts/build_px4.sh
```

此腳本會：
1. 若 `$HOME/PX4-Autopilot` 不存在，自動從 GitHub 複製原始碼
2. 執行 `make px4_sitl` 編譯 SITL 版本
3. 將編譯完成的執行檔放置於 `$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4`

### 步驟 3：安裝 MicroXRCEAgent

```bash
bash scripts/setup_env.sh --install-agent
```

此參數會一併安裝 MicroXRCEAgent，用於 PX4 與 ROS2 之間的通訊橋接。

### 步驟 4：驗證建構結果

```bash
bash scripts/validate_build.sh
```

此腳本會檢查：
- PX4 執行檔是否存在且可執行
- MicroXRCEAgent 是否已安裝
- ROS2 Humble 環境是否可用
- tmux 與 git 等工具是否已安裝

## 參考資料

- 建構環境詳細說明請參閱 `references/prerequisites.md`
- 建構參考指南請參閱 `references/build-reference.md`
- 啟動 PX4 模擬器請使用 px4-sitl-launcher 技能
