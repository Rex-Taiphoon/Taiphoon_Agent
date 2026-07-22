# PX4 建構環境前置需求

## 系統需求

- **作業系統**：Ubuntu 22.04（建議使用 WSL2）
- **處理器**：x86_64 相容
- **記憶體**：建議至少 8GB
- **磁碟空間**：至少 10GB 可用空間
- **網路**：需要穩定的網路連線以複製原始碼與下載相依套件

## 相依套件清單

### 編譯工具

| 套件 | 用途 | 安裝方式 |
|------|------|----------|
| `git` | 版本控制與原始碼複製 | `sudo apt install git` |
| `cmake` | 建構系統 | `sudo apt install cmake` |
| `ninja-build` | 加速建構 | `sudo apt install ninja-build` |
| `build-essential` | 基本編譯工具（gcc, g++） | `sudo apt install build-essential` |
| `ccache` | 編譯快取加速 | `sudo apt install ccache` |

### Python 套件

| 套件 | 用途 |
|------|------|
| `jinja2` | 範本引擎 |
| `toml` | TOML 設定檔解析 |
| `numpy` | 數值運算 |
| `empy` | Python 範本引擎 |
| `packaging` | 套件版本管理 |
| `kconfiglib` | Kconfig 設定解析 |

### 系統程式庫

| 套件 | 用途 |
|------|------|
| `libgstreamer1.0-dev` | 多媒體串流 |
| `libzmq3-dev` | ZeroMQ 訊息佇列 |
| `liblog4cxx-dev` | 日誌記錄 |
| `libncurses5-dev` | 終端機介面 |

## WSL2 建議設定

在 WSL2 環境中，建議在 `%USERPROFILE%\.wslconfig` 中設定以下參數：

```ini
[wsl2]
memory=8GB
processors=4
```

然後在 PowerShell 中執行 `wsl --shutdown` 重啟 WSL2 以套用設定。
