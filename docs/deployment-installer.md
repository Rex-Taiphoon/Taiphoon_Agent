# 安裝與替換設計

主角色：Docker／部署維護人員，負責介面、失效情境、測試與文件。
範圍：Linux／macOS 平台包內的 `install.sh`；Windows `install.ps1` 不在本輪範圍。
不修改原生啟動工具、ROS2 Node／Topic、飛控參數、網路或 Docker 設定。

## 介面

```text
bash install.sh [--yes] [--agent-id N] [--autostart | --no-autostart] [--no-start]
bash install.sh --rollback [--yes]
```

- 在解壓縮後的平台包目錄，以要執行 Agent 的使用者執行；不接受 `sudo bash install.sh`。
  設定 systemd 時腳本自行呼叫 `sudo`。
- `--yes` 不詢問：沿用既有設定、開機自動啟動狀態，並停止會阻擋啟動的舊入口容器。
- `--agent-id N` 只用於首次安裝：經由 `taiphoon_agent init` 建立無人機 N 的 deploy 設定。
  既有設定的編號不同時停止，請改用 `taiphoon_agent config`。
- 結束碼：0 成功；1 失敗（未變更、或已切回上一版）；2 參數錯誤。

## 安裝配置

```text
~/.local/share/taiphoon_agent/
├── releases/<版本>/        平台包內容：啟動工具、release.manifest、映像 archive
├── current  -> releases/<版本>
├── previous -> 上一版       可指向舊版 install.sh 安裝或手動部署的套件目錄
└── backup/                 被改寫前的 systemd 服務檔
~/.local/bin/taiphoon_agent  wrapper，執行 current/taiphoon_agent
~/.config/taiphoon_agent/    設定（TAIPHOON_AGENT_HOME 可覆寫）
/etc/systemd/system/taiphoon-agent.service  選用；ExecStart／ExecStop 指向 current
```

啟動工具以 `/proc/self/exe` 的實際路徑尋找同目錄的 manifest 與映像，`current` 為 symlink 不影響校驗。
服務檔只指向 `current`，之後升級或回復不需要再修改服務檔。

## 資料流

1. 檢查（不改系統）：必要檔案、manifest 版本、啟動工具 `--version` 與 manifest 一致、
   Docker 架構與套件相同、映像 archive SHA256、安裝位置磁碟空間；
   Linux 實機設定 `time_sync=auto／required` 時確認主機有 `chronyc`（啟動工具會拒絕未對時的實機）。
2. 偵測既有安裝：`current`、舊版 `install.sh` 的平鋪安裝、`taiphoon-agent.service`
   （User、ExecStart 所在套件、TAIPHOON_AGENT_HOME、enabled／active）、
   Compose 專案 `taiphoon_agent` 的執行中容器與其設定目錄、會阻擋啟動的 `uav_N_agent`／`fmuN_agent`。
3. 列出計畫並確認：版本、設定來源、開機自動啟動、要停止的容器、要清除的舊安裝版本。
4. 舊版仍在執行時：複製到 `releases/.staging-*`、預先 `docker load` 映像，縮短停機時間。
5. 停止舊版：`systemctl stop` → 該設定目錄的 `docker compose down` → 舊入口容器
   `docker update --restart=no` 後 `docker stop`（不刪除）。
6. 切換：舊平鋪安裝搬入 `releases/<舊版>`，設定 `previous`、`current`，更新 wrapper，
   把使用中的 `settings.ini` 複製到設定目錄（目的地已有不同內容時先備份），改寫並 reload 服務檔。
7. 安裝前有 Agent 在執行、或首次安裝時啟動新版；`start` 等待容器健康。
   由 systemd 管理時改用 `systemctl start`，避免服務狀態與實際容器不同步。
8. 成功後只保留 `current` 與 `previous` 兩個安裝版本；Docker 映像不刪除。

`--rollback` 停止目前版本、交換 `current` 與 `previous`，原本在執行時再以相同方式啟動。

## 失效情境

| 情境 | 行為 |
|---|---|
| 套件缺檔、版本不符、SHA256 不符、CPU 或 Docker 架構不符 | 停止；不變更檔案、服務或容器 |
| 找不到 docker CLI | 警告；只安裝檔案，不載入映像也不啟動 |
| docker CLI 存在但無法連線 Engine | 停止；無法確認舊版是否執行，不切換 |
| `docker load` 或空間不足 | 舊版仍在執行，停止安裝並清除 staging |
| 非互動執行且未加 `--yes` | 停止；不變更 |
| Linux 實機沒有 chrony 且 time_sync 不是 off／warn | 替換執行中的 Agent 時停止、不變更；首次安裝只警告 |
| 服務的 User 不是目前使用者、以 sudo 執行、同時執行兩個安裝 | 停止；不變更 |
| 停止舊版失敗或仍有容器執行 | 停止；不切換版本 |
| 新版啟動失敗（含對時未就緒、映像內容不符） | 停止新版、切回 `previous` 並依原狀態啟動，結束碼 1 |
| 首次安裝啟動失敗 | 保留安裝與設定，依啟動工具訊息處理後再 `start` |
| 首次安裝沒有設定且非互動 | 只安裝，不啟動；提示執行 `taiphoon_agent init` |

## 不做的事

不刪除設定、volume、Docker 映像或安裝目錄以外的套件；舊入口容器只停止不刪除。
不 Arm／起飛，不修改飛控、chrony、網路或 Docker daemon 設定。

## 驗證

`test/agent_installer_test.py` 以 docker、systemctl、sudo 與啟動工具替身在 Docker 開發容器內執行：
首次安裝與開機自啟、重複安裝、連續升級與保留兩版、`--rollback`、新版啟動失敗自動回復、
SHA256／架構不符、缺少 chrony 與非互動未確認時不變更、從手動 systemd 部署與舊版平鋪安裝替換、
舊入口容器處理。另以刪除關鍵步驟的變異確認測試會失敗（預載映像、停止舊版、切回、搬移設定等 13 項）。
替身不涵蓋真正的 systemd、Docker 映像載入與樹莓派效能，需在目標機實測。
