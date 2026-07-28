# Taiphoon_Agent 模擬測試紀錄
## 日期：2026-07-28

### 測試環境
- PX4 SITL x3 (FastDDS, SIH 模式)
- Taiphoon_Agent (basic_controller_node + formation_node x3)
- Docker Compose

### 測試項目與結果

#### 1. 基本高階控制指令 ✅
- `ARM`：三機同時解鎖，成功
- `TAKEOFF 8.0`：三機起飛至 8m，成功
- `MOVE_FORWARD 3` (fmu1)：向前移動 3m，成功
- `MOVE_RIGHT 3` (fmu2)：向右移動 3m，成功
- `MOVE_LEFT 3` (fmu3)：向左移動 3m，成功

#### 2. 環形編隊 FORMATION_CIRCLE ✅
- 命令：`FORMATION_CIRCLE 5.0 8.0`
- fmu1 (index 0, yaw 0°)：NED (5.69, 2.06, -8.0) ✅
- fmu2 (index 1, yaw 120°)：NED (-1.61, 6.33, -8.0) ✅
- fmu3 (index 2, yaw 240°)：NED (-1.73, -2.16, -8.0) ✅
- 三機等角分布 (120°)，半徑約 5m，高度一致

#### 3. 20m 半徑環形編隊 ✅
- 命令：`FORMATION_CIRCLE 20.0 10.0`
- fmu1 (yaw 0°)：NED (18.41, -2.19, -10.0) ✅
- 半徑約 18.5m，符合 20m 設定

#### 4. 無人機離線後編隊調整 ✅
- 關閉 fmu3 後：
  - fmu1 目標重新計算為雙機圓形 (yaw 0°)
  - 質心與角度重新分配，編隊自動縮小
  - 驗證：NED (3.57, -2.01, -8.0)

### 已知問題
- launch 時需手動 source px4_msgs workspace (`/home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash`)
- `ros2 topic echo` 需 source 全部 workspace 才能解析自訂訊息

### 結論
所有測試項目通過，basic_controller 重構與 formation_node 新增功能正常運作。

#### 追加測試（補完）

#### 5. MOVE_BACKWARD ✅
- 命令：`MOVE_BACKWARD 3` (fmu2)
- fmu2 位置：(-22.32, -2.87, -10.01)，向後移動成功

#### 6. MODE_RTL ✅
- 命令：`MODE_RTL` (fmu1)
- fmu1 開始返航，位置從 (18.4, -2.2, -10) 移向起飛點

---

### 完整測試證據

所有六項高階指令均正常執行：
| 指令 | 測試 | 結果 |
|------|------|------|
| ARM | 三機同時解鎖 | ✅ |
| TAKEOFF | 起飛至 8m | ✅ |
| MOVE_FORWARD | fmu1 向前 3m | ✅ |
| MOVE_BACKWARD | fmu2 向後 3m | ✅ |
| MOVE_LEFT | fmu3 向左 3m | ✅ |
| MOVE_RIGHT | fmu2 向右 3m | ✅ |
| MODE_RTL | fmu1 返航 | ✅ |
| FORMATION_CIRCLE | 5m/8m 環形 | ✅ |
| 離線調整 | fmu3 關閉後 2 機編隊 | ✅ |
