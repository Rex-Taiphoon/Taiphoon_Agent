---
name: wsl-tools
description: WSL2 環境操作工具，用於在 Windows 終端機（CMD/PowerShell）中存取 WSL2 內部的檔案與指令。用於：(1) 從 Windows 端執行 WSL2 內的 Linux 指令，(2) 查詢 WSL2 執行狀態與已安裝的發行版，(3) 重新啟動 WSL2 執行環境，(4) 在 Windows 與 WSL2 之間轉換檔案路徑。不適用於 ROS2 或 PX4 操作，請改用對應的技能。
---

# 功能名稱
WSL2 環境存取工具

本技能提供從 Windows 終端機（CMD 或 PowerShell）存取 WSL2 Ubuntu 22.04 環境的標準化指令介面。當 Agent 的當前終端為 Windows 環境（非 WSL 內部 bash）時，所有需要在 Linux 環境中執行的操作都應透過本技能提供的指令封裝來執行。

主要功能涵蓋：從 Windows 端執行單一 Linux 指令、查詢 WSL 發行版狀態與執行中行程、重新啟動 WSL2 虛擬機器、以及路徑格式轉換。

# 前置條件
- Windows 10/11 已啟用 WSL2 功能
- 已安裝 Ubuntu-22.04 發行版
- 目前終端環境為 CMD 或 PowerShell（非 WSL 內 bash）
- 具備 `wsl` 命令列工具的執行權限

# 基本使用方式
以下為本技能提供的基本指令操作方式。

## 從 Windows 終端執行 Linux 指令
在 WSL2 Ubuntu-22.04 中執行單一 bash 指令並取得輸出結果。
`wsl -d Ubuntu-22.04 -- bash -c "linux_command_here"`

## 查詢 WSL 執行狀態
列出所有已安裝的 WSL 發行版及其執行狀態。
`wsl --list --verbose`

## 重新啟動 WSL2
當 WSL2 環境出現異常（如網路中斷、記憶體不足）時，完全關閉並重新啟動。
`wsl --shutdown`

## 查詢 WSL 內部行程
從 Windows 端查看 WSL2 內部正在執行的行程。
`wsl -d Ubuntu-22.04 -- bash -c "ps aux"`

# 進階使用方式
以下為常見使用場景的指令組合方式，由 Agent 根據需求自行組合基本指令。

## 檢查 ROS2 狀態（從 Windows 端）
當無法直接進入 WSL bash 時，從 Windows 端查詢 ROS2 狀態。
`wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"`

## 批次執行多個 Linux 指令
使用分號串接多個指令在單一 WSL 呼叫中執行。
`wsl -d Ubuntu-22.04 -- bash -c "cmd1 && cmd2 && cmd3"`

## 強制重啟後驗證環境
完全重啟 WSL2 後確認環境恢復正常。
```bash
wsl --shutdown
Start-Sleep -Seconds 5
wsl -d Ubuntu-22.04 -- bash -c "echo 'WSL2 is ready'"
```

# 開發原則限制
- 應遵照 AGENTS.md 的指引，避免偏離核心開發原則
- 僅在目前終端為 Windows 環境（CMD/PowerShell）時使用本技能
- 若已在 WSL bash 環境內，應直接執行 Linux 指令而非透過 wsl 封裝
- 預設目標發行版為 `Ubuntu-22.04`，不應隨意變更
- 長時間執行的指令（如 PX4 編譯）不應透過本技能執行，應直接進入 WSL 操作