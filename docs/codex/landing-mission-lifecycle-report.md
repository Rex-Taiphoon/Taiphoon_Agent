# 降落後編隊與規劃修正驗證

日期：2026-10-06。範圍只限 Taiphoon_Agent；WGS84／GCS 精度由 GCS 專案處理。

## 行為

- PX4 有效觀測確認落地或解除解鎖時，清除任務／保留位置目標、編隊 Epoch、assignment、center、候選、active plan、煞車／等待與 terminal hold。
- 地面不執行 optimizer，不發布有效本機控制軌跡，也不廣播同儕佔位；既有 local／global trajectory 各發布撤銷訊息 `valid=false`。
- 其他 Agent 依新鮮、有效的 landed 狀態移除該機舊軌跡，並排除其編隊成員、即時距離與必要軌跡 freshness 名單。地面狀態過期時不擅自延用此排除結論。
- 地面 MOVE／FORMATION 不排隊；重新起飛後不會恢復舊槽位或任務。舊 Epoch 重播無法啟動編隊，必須收到新的空中編隊命令。
- BasicController 依落地／解除解鎖觀測清除定時軌跡；ROS2 節點拒絕地面及無效觀測期間抵達的有效軌跡，避免延遲 callback 重新填入快取。
- 維持原有 PX4 LAND／TAKEOFF／RTL 模式與命令時序，沒有新增訊息、Node、Topic 或編隊 FSM。

## 結果

| 項目 | 結果 |
|---|---|
| 獨立 Docker workspace Release 建置 | PASS |
| 日常 Docker workspace Release 最終建置 | PASS |
| 現行單元測試（24 個 suite、254 個案例） | PASS，0 errors／failures／skipped |
| 真實 planner 節點 ROS2 回歸（隔離 DDS domain 77） | PASS |
| `git diff --check` | PASS |
| 本次修正後實際 PX4 SITL 降落再起飛 | 未執行；本次節點回歸使用合成 AgentState，不代表物理飛行驗收 |

新增 3 個任務生命週期測試，涵蓋地面命令、重飛／Epoch 回復限制與無效觀測；新增 2 個 BasicController 回歸，直接經 ROS2 發布落地／解除解鎖觀測並確認快取清除。

節點回歸啟動實際 `ego_swarm_planner` 執行檔，提供有效空障礙物地圖與一台仍在線的同儕。同儕先在空中發布與本機重疊的佔位，再落地並重播舊佔位；本機仍可產生真正移動的軌跡，編隊名單只包含本機。接著驗證空中 MOVE／編隊 → 落地 → 地面命令／舊 Epoch 重播 → 再起飛 → 新編隊命令：降落後無有效 local／peer trajectory，重飛不恢復舊任務，新命令可以重新啟動。

單元測試使用 DDS domain 78。所有 ROS2、編譯與測試均在 Docker 內執行，沒有使用 WSL。沒有向人工操作機群發布飛行命令或重啟 GCS。觀察到既有三機 SITL／Agent 已停止後，才建置日常 workspace；一次性驗證與建置容器皆以 `--rm` 清理，保留建置卷與證據。

初次查核日常 workspace 時，`colcon test-result` 遞迴讀入舊 `test_results/` 內的 4 筆歷史失敗。本次 CMake 實際結果目錄是 `build/taiphoon_agent/small_field_test_results/`；改用此路徑查核，254 個案例全部通過。舊 XML 未刪除。日常 workspace 最終已統一為 Release 編譯。

## 證據與重現

證據保存在 `AutoTest/Log/landing-lifecycle-20261006/`：

- `runtime-final-build.txt`：日常 workspace 最終建置。
- `runtime-unit.txt`：254 個案例的最終結果。
- `runtime-integration.txt`：加強後節點回歸結果。
- `landing-node.txt`：實際執行檔的編隊建立與重新建立紀錄。
- `build.txt`、`unit.txt`、`integration.txt`、`planner-node.txt`：獨立 workspace 的驗證證據。

使用先前三機人工驗證相同的 Windows Compose override：`AutoTest/Log/small-field-20261006/formation-failed-2-143318/compose.windows.yaml`，以及 `taiphoon_agent_validation_{build,install,log}_clean` volumes。新編譯執行檔已安裝到這組日常驗證 workspace，下一次以此設定啟動 Agent 即使用修正版；既有 Release 套件／映像沒有重新打包。

下列命令在載入上述 workspace 的 Docker 開發容器中執行：

```bash
source install/setup.bash
colcon build --packages-select taiphoon_agent --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  colcon test --packages-select taiphoon_agent --executor sequential
colcon test-result --test-result-base build/taiphoon_agent/small_field_test_results --verbose
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  TAIPHOON_PLANNER_EXECUTABLE="$PWD/install/taiphoon_agent/lib/taiphoon_agent/ego_swarm_planner" \
  TAIPHOON_TEST_LOG_DIR="$PWD/AutoTest/Log/landing-lifecycle-20261006" \
  python3 test/landing_planner_integration_test.py
```
