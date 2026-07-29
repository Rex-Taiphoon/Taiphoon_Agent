# 01 — 自訂訊息定義與 CMake 更新

**What to build:** 定義 `AgentState.msg` 與 `AgentAttitude.msg`，更新 `CMakeLists.txt` 讓 colcon 能編譯這些訊息，其他模組可透過 `rosidl_target_interfaces` 引用。

**Blocked by:** None — can start immediately

**Status:** ready-for-agent

- [ ] `AgentState.msg` 定義於 `msg/`，含 `position_ned`（float32[3]）、`flight_mode`（uint8）、`battery_remaining`（float32）、`online`（bool）、`stamp`（builtin_interfaces/Time）
- [ ] `AgentAttitude.msg` 定義於 `msg/`，含 `q`（float32[4] 四元數 w,x,y,z）、`stamp`
- [ ] `CMakeLists.txt` 的 `rosidl_generate_interfaces` 加入兩個新訊息
- [ ] `colcon build --symlink-install` 成功，無編譯錯誤
- [ ] `ros2 interface show taiphoon_agent/msg/AgentState` 顯示正確欄位與型別
- [ ] `ros2 interface show taiphoon_agent/msg/AgentAttitude` 顯示正確欄位與型別

## Comments
