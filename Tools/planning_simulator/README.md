# Taiphoon Agent 離線路徑規劃模擬器

此工具位於 `Tools/`，不屬於 `taiphoon_agent` ROS2 package，也不會被根目錄
`CMakeLists.txt` 自動編譯。它直接引用目前專案的純 C++ EGO-Swarm planner
與 safety 元件，因此不需要 ROS2、Docker、PX4 或 SITL。

## 需求

- macOS Apple Clang 或其他 C++17 編譯器
- CMake 3.16 以上
- Python 3.9 以上（只使用標準函式庫）

## 建置

在專案根目錄執行：

```bash
cmake -S Tools/planning_simulator \
      -B /tmp/taiphoon_planning_build \
      -DTAIPHOON_AGENT_SOURCE_DIR="$PWD"
cmake --build /tmp/taiphoon_planning_build --parallel
```

這個 build tree 在 `/tmp`，不會寫入 ROS2 的 `build/`、`install/` 或 `log/`。

模擬器會直接編譯目前 runtime 使用的 `OptimizedEgoPlanner`、
`SpatiotemporalRouteInitializer` 與 `ProjectedGradientSolver`；FIXED_Z 情境會
啟用與 runtime 相同的時空 route seed 流程。這是工具層的組裝設定，不會改變
planner 的核心預設值或演算法。

## 啟動瀏覽器工具

最方便的方式是在專案根目錄執行：

```bash
sh Tools/planning_simulator/run.sh
```

它會把這個工具編譯到 `/tmp/taiphoon_planning_build`，再啟動本機服務；也可以直接
把 `--port 9000` 等參數傳給 Python service。

若希望分開控制建置與服務，也可以使用以下指令：

```bash
python3 Tools/planning_simulator/server/server.py \
  --engine /tmp/taiphoon_planning_build/planning_simulator
```

接著用瀏覽器開啟：

```text
http://127.0.0.1:8765
```

若只想確認 engine 與情境檔串接，不啟動瀏覽器服務，可執行：

```bash
python3 Tools/planning_simulator/tests/smoke_test.py \
  --engine /tmp/taiphoon_planning_build/planning_simulator
```

頁面可以調整：

- 自身飛機位置與目標
- FREE_3D、FIXED_Z、LINEAR_Z
- B-spline 控制點數量與 knot interval
- obstacle clearance、peer separation、速度／加速度／jerk
- box、cylinder、wall 固定障礙物
- 懸停或移動中的 peer trajectory

結果會顯示規劃 B-spline、控制點、peer trajectory、固定障礙物、側視高度、
最小障礙距離、最小 peer 距離、最大動力學量、規劃耗時與 safety reason。

網頁輸入的目標是 Final Target。模擬器會重用既有 `LocalGoalSelector`，以預設
10 m horizon 產生目前這一段的 Local Goal，再交給 `OptimizedEgoPlanner`；圖上以
實心標記顯示 Local Goal，以空心標記顯示尚未到達的 Final Target。這個離線工具
目前只模擬單一 planning segment，不會模擬正式 ROS2 Node 的進度觸發與下一段
replan。

固定障礙物會先用 `ObstacleCloudGenerator` 產生點雲，再交給目前的
`RawPointObstacleMap`、planner 與 `ContinuousSafetyValidator`；網頁顯示的障礙物
幾何則來自同一份 JSON 情境。因此這裡不是只把障礙物畫在圖上，規劃與安全檢查
也會實際使用它們。

## 情境檔

目前提供：

- `scenarios/open_space.json`
- `scenarios/fixed_obstacle_peer.json`

`fixed_obstacle_peer.json` 預設把障礙物放在直線路徑旁，方便先確認成功結果；把 Box、Cylinder
或 Peer 移到路徑上即可觀察目前 planner 的避障、逾時或 safety 診斷。

瀏覽器中的「匯出 JSON」可儲存目前設定，「載入 JSON」可重新載入情境。

## 設計邊界

- C++ engine 直接編譯 `src/ego_swarm/OptimizedEgoPlanner.cpp`、
  `ProjectedGradientSolver.cpp`、`src/perception/RawPointObstacleMap.cpp` 與
  `ObstacleCloudGenerator.cpp`；不複製 planner 實作。
- 因此既有這些純 C++ 元件的演算法更新，重新執行 `run.sh` 就會帶入；若未來新增
  planner translation unit，才需要同步補上一行 `Tools/planning_simulator/CMakeLists.txt`。
- 所有離線情境使用單一公尺制 NED frame；不依賴 ROS2 的 WGS84 message adapter。
- peer 以時間化 B-spline 表示，不會被錯誤地當成靜態 PointCloud 障礙物。
- 目前工具驗證單次 optimized planning 與共同 ContinuousSafetyValidator。
  ROS2 Node 內的完整 callback lifecycle 仍由正式 runtime 負責。
- 時空 route initializer 目前只適用於 `FIXED_Z`；`FREE_3D` 與 `LINEAR_Z` 仍使用
  既有 optimizer 與共同 safety validator。若搜尋超過現有 planner time budget，
  網頁會如實顯示 timeout／安全失敗，不會改用不安全的直線結果。
- 這是離線模擬器，不會訂閱正在執行中的 ROS2 topic；它使用相同的 pure C++ planner
  與 safety 元件重建一次規劃結果。正式 runtime 的 callback、reservation lifecycle
  與 PX4 tracking 仍需用 Docker／SITL 驗證。

## 直接使用 engine

Python server 使用 stdin 協定呼叫 engine。除非要整合其他工具，不需要直接操作
這個協定；瀏覽器介面與 JSON 情境是建議入口。
