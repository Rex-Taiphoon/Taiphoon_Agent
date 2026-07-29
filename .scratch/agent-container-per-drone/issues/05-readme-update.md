# 05 — README / 使用手冊更新

**What to build:** 更新 `README.md` 與相關操作手冊，僅收錄經過 #1-#4 驗證的指令與流程。涵蓋：建構映像、啟動全棧、單機控制、多機編隊、除錯、關閉清理。

**Blocked by:** #1, #2, #3, #4

**Status:** ready-for-agent

## Acceptance Criteria

- [ ] 「快速開始」章節改寫為新的 docker compose 流程（`docker compose up -d` + `launch_all.sh`）
- [ ] 移除所有引用舊架構的指令：`ros2 launch ... drone_count:=`、`docker compose exec taiphoon-agt`、`--profile hw`
- [ ] 新增「單機操作」章節：如何對特定 `fmu{N}_agent` 重啟/停止/查看 log
- [ ] 新增「多機編隊」章節：如何發送編隊命令、如何驗證隊形
- [ ] 新增「除錯」章節：`docker compose logs -f`、`docker compose ps`、`ros2 topic list` 等常用指令
- [ ] 模擬器管理章節更新：`launch_all.sh` 參數說明、`stop_all.sh` 用法
- [ ] 系統需求更新：移除 uxrce-agent 相關依賴
- [ ] 所有指令必須在 #4 的端對端測試中實際執行過且驗證正確
- [ ] `docs/basic_controller_manual.md` 同步更新（如有衝突內容）

## Design Notes

- 只收錄驗證過的內容，不寫「預計」、「未來」等未實現功能
- 指令範例使用實際的 container name（`fmu1_agent`）、topic name（`/fmu1_agent/state`）
- 保留繁體中文撰寫風格，與現有 README 一致
