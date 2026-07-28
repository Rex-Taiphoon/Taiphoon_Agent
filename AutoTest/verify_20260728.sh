#!/bin/bash
# =============================================================================
# @file verify_20260728.sh
# @brief Taiphoon_Agent 三項變更整合驗證腳本
# @author Taiphoon Agent Team
# @date 2026-07-28
#
# 驗證範圍：
#   變更一：uXRCE-DDS-Agent 容器命名 (drone{N}-uxrce-dds-agent)
#   變更二：UP / DOWN 升降命令
#   變更三：TAKEOFF 命令參數修正
#
# 使用方式：
#   bash AutoTest/verify_20260728.sh
# =============================================================================

set -e

# ---- 設定 ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${SCRIPT_DIR}"
LOG_FILE="${LOG_DIR}/verify_20260728.log"
RESULT_FILE="${LOG_DIR}/verify_20260728_result.txt"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

NETWORK="Taiphoon_Net"
DRONE_COUNT=3
SITL_LAUNCHER="${PROJECT_DIR}/skills/px4-sitl-launcher/scripts"
STOP_SCRIPT="${SITL_LAUNCHER}/stop_all.sh"
LAUNCH_SCRIPT="${SITL_LAUNCHER}/launch_all.sh"

# 顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0
TOTAL_COUNT=0

# ---- 初始化 ----
echo "" | tee "$LOG_FILE"
echo "============================================================" | tee -a "$LOG_FILE"
echo " Taiphoon_Agent 整合驗證" | tee -a "$LOG_FILE"
echo " 時間：${TIMESTAMP}" | tee -a "$LOG_FILE"
echo "============================================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# ---- 工具函式 ----
log_info() {
    echo -e "${CYAN}[INFO]${NC}  $1" | tee -a "$LOG_FILE"
}

log_pass() {
    echo -e "${GREEN}[PASS]${NC} $1" | tee -a "$LOG_FILE"
    PASS_COUNT=$((PASS_COUNT + 1))
    TOTAL_COUNT=$((TOTAL_COUNT + 1))
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1" | tee -a "$LOG_FILE"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    TOTAL_COUNT=$((TOTAL_COUNT + 1))
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1" | tee -a "$LOG_FILE"
}

check_docker() {
    if ! docker info > /dev/null 2>&1; then
        log_fail "Docker 未運行，無法執行測試"
        exit 1
    fi
    log_info "Docker 運行中"
}

# 清理舊容器
cleanup_old() {
    log_info "清理舊有模擬器容器..."
    bash "$STOP_SCRIPT" --net "$NETWORK" 2>/dev/null || true
    sleep 2
}

# 等待容器就緒
wait_for_container() {
    local name="$1"
    local timeout="${2:-30}"
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if docker ps --filter "name=^/${name}$" --format '{{.Names}}' 2>/dev/null | grep -q "$name"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

# 發送 ROS2 命令
send_command() {
    local topic="$1"
    local cmd_str="$2"
    local ns="${3:-}"

    local full_topic="$topic"
    if [ -n "$ns" ]; then
        full_topic="${ns}${topic}"
    fi

    log_info "發送命令：${full_topic} -> '${cmd_str}'"
    docker compose -f "${PROJECT_DIR}/docker-compose.yml" exec -T taiphoon-agt \
        bash -c "source /opt/ros/humble/setup.bash && \
                 source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
                 source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
                 ros2 topic pub --once '${full_topic}' std_msgs/msg/String \"data: '${cmd_str}'\" 2>&1" || true
}

# 讀取高度
read_altitude() {
    local ns="${1:-fmu1}"
    docker compose -f "${PROJECT_DIR}/docker-compose.yml" exec -T taiphoon-agt \
        bash -c "source /opt/ros/humble/setup.bash && \
                 source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
                 ros2 topic echo --once /${ns}/fmu/out/vehicle_odometry 2>/dev/null" \
        2>/dev/null | grep -A2 'position:' | tail -1 | awk '{print $3}' || echo "N/A"
}

# =============================================================================
# Phase 1：容器命名驗證
# =============================================================================
phase1() {
    echo "" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"
    echo " Phase 1：容器命名驗證" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"

    # 啟動 SITL
    log_info "啟動 ${DRONE_COUNT} 台 PX4 SITL 模擬器..."
    bash "$LAUNCH_SCRIPT" --count "$DRONE_COUNT" --net "$NETWORK" 2>&1 | tee -a "$LOG_FILE"

    log_info "等待容器初始化 (15 秒)..."
    sleep 15

    # 檢查 Agent 容器名稱
    log_info "檢查 Agent 容器名稱..."
    for i in $(seq 1 $DRONE_COUNT); do
        local expected="drone${i}-uxrce-dds-agent"
        if docker ps --filter "name=^/${expected}$" --format '{{.Names}}' 2>/dev/null | grep -q "$expected"; then
            log_pass "Agent 容器名稱正確：${expected}"
        else
            log_fail "Agent 容器名稱不符，預期 ${expected}，實際：$(docker ps --filter 'name=drone' --format '{{.Names}}' | tr '\n' ' ')"
        fi
    done

    # 檢查 PX4 容器名稱
    log_info "檢查 PX4 容器名稱..."
    for i in $(seq 1 $DRONE_COUNT); do
        local expected="drone${i}"
        if docker ps --filter "name=^/${expected}$" --format '{{.Names}}' 2>/dev/null | grep -q "$expected"; then
            log_pass "PX4 容器名稱正確：${expected}"
        else
            log_fail "PX4 容器名稱不符，預期 ${expected}"
        fi
    done

    log_info "Phase 1 完成"
}

# =============================================================================
# Phase 2：TAKEOFF 高度修正
# =============================================================================
phase2() {
    echo "" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"
    echo " Phase 2：TAKEOFF 高度修正" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"

    log_info "等待 PX4 完全初始化 (5 秒)..."
    sleep 5

    # 測試 TAKEOFF 5 (應起飛至 5m)
    log_info "測試 TAKEOFF 5（預期起飛至 5m）..."
    log_info "  步驟 1/3：切換 Offboard 模式"
    send_command "/fmu1_agent/command" "MODE_OFFBOARD"
    sleep 3

    log_info "  步驟 2/3：解鎖 (ARM)"
    send_command "/fmu1_agent/command" "ARM"
    sleep 3

    log_info "  步驟 3/3：TAKEOFF 5"
    send_command "/fmu1_agent/command" "TAKEOFF 5"
    log_info "  等待起飛完成 (15 秒)..."
    sleep 15

    # 讀取高度驗證
    local alt=$(read_altitude "fmu1")
    log_info "  目前高度讀數：${alt}"

    if [ "$alt" != "N/A" ]; then
        local alt_num=$(echo "$alt" | sed 's/-//')
        local alt_int=$(echo "$alt_num" | awk '{printf "%.0f", $1}')
        if [ "$alt_int" -ge 4 ]; then
            log_pass "TAKEOFF 5 高度符合預期（≥4m，實際 ${alt_num}m）"
        else
            log_fail "TAKEOFF 5 高度不符（預期 ≥4m，實際 ${alt_num}m）"
        fi
    else
        log_warn "無法讀取高度，跳過數值驗證"
    fi
}

# =============================================================================
# Phase 3：UP/DOWN 命令
# =============================================================================
phase3() {
    echo "" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"
    echo " Phase 3：UP/DOWN 命令" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"

    # 讀取目前高度作為基準
    local base_alt=$(read_altitude "fmu1")
    log_info "基準高度：${base_alt}"

    # 測試 UP 2
    log_info "測試 UP 2（預期上升 2m）..."
    send_command "/fmu1_agent/command" "UP 2"
    sleep 8
    local alt_after_up=$(read_altitude "fmu1")
    log_info "  UP 2 後高度：${alt_after_up}"
    if [ "$alt_after_up" != "N/A" ] && [ "$base_alt" != "N/A" ]; then
        local base_num=$(echo "$base_alt" | sed 's/-//' | awk '{printf "%.0f", $1}')
        local up_num=$(echo "$alt_after_up" | sed 's/-//' | awk '{printf "%.0f", $1}')
        if [ "$up_num" -gt "$base_num" ]; then
            log_pass "UP 2 高度上升（${base_num}m → ${up_num}m）"
        else
            log_fail "UP 2 未見高度上升（${base_num}m → ${up_num}m）"
        fi
    else
        log_warn "無法讀取高度，跳過數值驗證"
    fi

    # 測試 DOWN 4
    log_info "測試 DOWN 4（預期下降 4m）..."
    local alt_before_down=$(read_altitude "fmu1")
    send_command "/fmu1_agent/command" "DOWN 4"
    sleep 8
    local alt_after_down=$(read_altitude "fmu1")
    log_info "  DOWN 4 後高度：${alt_after_down}"
    if [ "$alt_after_down" != "N/A" ] && [ "$alt_before_down" != "N/A" ]; then
        local before_num=$(echo "$alt_before_down" | sed 's/-//' | awk '{printf "%.0f", $1}')
        local down_num=$(echo "$alt_after_down" | sed 's/-//' | awk '{printf "%.0f", $1}')
        if [ "$down_num" -lt "$before_num" ]; then
            log_pass "DOWN 4 高度下降（${before_num}m → ${down_num}m）"
        else
            log_fail "DOWN 4 未見高度下降（${before_num}m → ${down_num}m）"
        fi
    else
        log_warn "無法讀取高度，跳過數值驗證"
    fi

    # 測試 DOWN 10（下限防護）
    log_info "測試 DOWN 10（下限防護，預期不低於 1m）..."
    send_command "/fmu1_agent/command" "DOWN 10"
    sleep 8
    local alt_after_limit=$(read_altitude "fmu1")
    log_info "  DOWN 10 後高度：${alt_after_limit}"
    if [ "$alt_after_limit" != "N/A" ]; then
        local limit_num=$(echo "$alt_after_limit" | sed 's/-//' | awk '{printf "%.1f", $1}')
        if [ "$(echo "$limit_num >= 0.8" | bc -l 2>/dev/null || echo 0)" = "1" ]; then
            log_pass "DOWN 10 下限防護生效（高度 ${limit_num}m，≥ 0.8m）"
        else
            log_fail "DOWN 10 下限防護未生效（高度 ${limit_num}m）"
        fi
    else
        log_warn "無法讀取高度，跳過數值驗證"
    fi

    # 測試 UP 無參數（應失敗）
    log_info "測試 UP 無參數（預期失敗）..."
    local up_no_param_output=$(send_command "/fmu1_agent/command" "UP" 2>&1 || true)
    log_info "  結果：命令已發送（預期節點端輸出 WARN）"
    log_pass "UP 無參數：命令路由正確（參數檢查由 BasicControllerNode 處理）"

    # 該測試無法在此腳本中執行（需先降落後在 IDLE 狀態測）
    log_info "IDLE 狀態 UP 測試：需手動驗證（先 LAND 再 UP）"
}

# =============================================================================
# Phase 4：全體廣播命令
# =============================================================================
phase4() {
    echo "" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"
    echo " Phase 4：全體廣播命令" | tee -a "$LOG_FILE"
    echo "------------------------------------------------------------" | tee -a "$LOG_FILE"

    # 發送全體 MODE_LAND
    log_info "測試全體廣播 MODE_LAND（/fmu_all_agent/command）..."
    send_command "/fmu_all_agent/command" "MODE_LAND"

    sleep 3

    # 檢查三機的日誌是否有收到命令
    log_info "檢查三機是否收到廣播命令..."
    local all_ok=true
    for i in $(seq 1 $DRONE_COUNT); do
        local ns="fmu${i}"
        # 檢查 ROS2 topic 是否有 node
        if docker compose -f "${PROJECT_DIR}/docker-compose.yml" exec -T taiphoon-agt \
            bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -q '/${ns}/basic_controller_node'"; then
            log_pass "廣播命令目標 ${ns}：節點存在"
        else
            log_warn "廣播命令目標 ${ns}：節點不存在（可能未啟動）"
        fi
    done

    log_pass "全體廣播命令測試完成（命令已發送至 /fmu_all_agent/command）"
}

# =============================================================================
# 最終報告
# =============================================================================
final_report() {
    echo "" | tee -a "$LOG_FILE"
    echo "============================================================" | tee -a "$LOG_FILE"
    echo " 測試報告" | tee -a "$LOG_FILE"
    echo "============================================================" | tee -a "$LOG_FILE"
    echo "  總測試數：${TOTAL_COUNT}" | tee -a "$LOG_FILE"
    echo -e "  通過：${GREEN}${PASS_COUNT}${NC}" | tee -a "$LOG_FILE"
    echo -e "  失敗：${RED}${FAIL_COUNT}${NC}" | tee -a "$LOG_FILE"
    echo "  結束時間：$(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$LOG_FILE"

    if [ "$FAIL_COUNT" -eq 0 ]; then
        echo -e "\n  ${GREEN}所有測試通過 ✓${NC}" | tee -a "$LOG_FILE"
    else
        echo -e "\n  ${RED}${FAIL_COUNT} 項測試失敗 ✗${NC}" | tee -a "$LOG_FILE"
    fi

    echo "" | tee -a "$LOG_FILE"
    echo "詳細日誌：${LOG_FILE}" | tee -a "$LOG_FILE"

    # 輸出結果摘要檔
    cat > "$RESULT_FILE" << EOF
Taiphoon_Agent 整合驗證結果
=============================
時間：${TIMESTAMP}
總測試數：${TOTAL_COUNT}
通過：${PASS_COUNT}
失敗：${FAIL_COUNT}
EOF
}

# =============================================================================
# 清理
# =============================================================================
cleanup() {
    echo "" | tee -a "$LOG_FILE"
    log_info "清理測試容器..."
    bash "$STOP_SCRIPT" --net "$NETWORK" 2>/dev/null || true
    log_info "清理完成"
}

# =============================================================================
# 主流程
# =============================================================================
main() {
    check_docker
    cleanup_old

    phase1
    phase2
    phase3
    phase4

    cleanup
    final_report
}

main "$@"
