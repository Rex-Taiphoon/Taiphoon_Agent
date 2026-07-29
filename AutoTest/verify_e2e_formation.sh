#!/bin/bash
# =============================================================================
# @file verify_e2e_formation.sh
# @brief 端對端整合驗證：三機環形編隊
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${SCRIPT_DIR}/20260729_e2e_$(date +%H%M)"
RESULT_FILE="${LOG_DIR}/result.txt"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

NETWORK="Taiphoon_Net"
SITL_LAUNCHER="${PROJECT_DIR}/skills/px4-sitl-launcher/scripts"

PASS_COUNT=0; FAIL_COUNT=0
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'; NC='\033[0m'
mkdir -p "$LOG_DIR"

log_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS_COUNT=$((PASS_COUNT+1)); }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL_COUNT=$((FAIL_COUNT+1)); }
log_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

ros2_exec() {
    local agent="$1"; shift
    docker exec "$agent" bash -c "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash 2>/dev/null
        source /home/taiphoon/Taiphoon_Agent/install/setup.bash 2>/dev/null
        $*
    " 2>&1
}

broadcast_cmd() {
    local cmd_str="$1"; local wait="${2:-2}"
    log_info "廣播: ${cmd_str}"
    ros2_exec fmu1_agent \
        "ros2 topic pub --wait-matching-subscriptions 3 --times 3 --rate 1 /fmu_all_agent/command std_msgs/msg/String 'data: ${cmd_str}'" \
        || log_warn "廣播 timeout"
    sleep "$wait"
}

read_agent_ned() {
    local agent_ns="$1"
    ros2_exec fmu1_agent "ros2 topic echo --once /${agent_ns}/state 2>/dev/null" \
        2>/dev/null | grep -A3 "position_ned:" | grep "^\s*-" | sed 's/^\s*- //' | tr '\n' ' '
}

# =============================================================================
phase1_start() {
    echo ""; echo "======== Phase 1：啟動環境 ========"
    log_info "清理舊容器..."
    docker compose -f "${PROJECT_DIR}/docker-compose.yml" down 2>/dev/null || true
    docker rm -f drone1 drone2 drone3 2>/dev/null || true
    log_info "啟動 Agent..."
    docker compose -f "${PROJECT_DIR}/docker-compose.yml" up -d 2>&1
    sleep 5
    log_info "啟動 PX4 SITL..."
    for i in 0 1 2; do
        bash "${SITL_LAUNCHER}/start_single_instance.sh" --index $i --agent "fmu$((i+1))_agent" --net "$NETWORK"
        sleep 2
    done
    log_info "等待初始化..."
    sleep 15
    for name in fmu1_agent fmu2_agent fmu3_agent drone1 drone2 drone3; do
        docker ps --filter "name=^/${name}$" --format "{{.Names}}" | grep -q "$name" \
            && log_pass "容器 ${name}" || log_fail "容器 ${name}"
    done
}

phase2_ready() {
    echo ""; echo "======== Phase 2：節點就緒 ========"
    sleep 10
    for ns in fmu1_agent fmu2_agent fmu3_agent; do
        ros2_exec fmu1_agent "ros2 topic list 2>/dev/null | grep /${ns}/state" \
            && log_pass "${ns} state topic" || log_fail "${ns} state topic"
    done
    for agent in fmu1_agent fmu2_agent fmu3_agent; do
        PEER=$(docker logs "$agent" 2>&1 | grep -c "Discovered peer" || true)
        [ "$PEER" -ge 2 ] && log_pass "${agent} 同儕 ${PEER}" || log_warn "${agent} 同儕 ${PEER}"
    done
}

phase3_offboard() {
    echo ""; echo "======== Phase 3：MODE_OFFBOARD ========"
    broadcast_cmd "MODE_OFFBOARD" 5
    for agent in fmu1_agent fmu2_agent fmu3_agent; do
        docker logs "$agent" 2>&1 | grep -q "offboard.*mode\|OFFBOARD" \
            && log_pass "${agent} Offboard" || log_warn "${agent} Offboard 未確認"
    done
}

phase4_arm() {
    echo ""; echo "======== Phase 4：ARM ========"
    broadcast_cmd "ARM" 3
    for agent in fmu1_agent fmu2_agent fmu3_agent; do
        docker logs "$agent" 2>&1 | grep -q "ARM command sent" \
            && log_pass "${agent} ARM" || log_fail "${agent} ARM"
    done
}

phase5_takeoff() {
    echo ""; echo "======== Phase 5：TAKEOFF 8m ========"
    broadcast_cmd "TAKEOFF 8.0" 20
    for ns in fmu1_agent fmu2_agent fmu3_agent; do
        NED=$(read_agent_ned "$ns")
        Z=$(echo "$NED" | awk '{print $3}')
        Z_ABS=$(echo "$Z" | sed 's/-//')
        log_info "${ns}: z=${Z_ABS}"
        [ -n "$Z_ABS" ] && [ "$(echo "$Z_ABS > 2.0" | bc -l 2>/dev/null || echo 0)" = "1" ] \
            && log_pass "${ns} 高度 ${Z_ABS}m" || log_warn "${ns} 高度 ${Z_ABS}m"
    done
}

phase6_formation() {
    echo ""; echo "======== Phase 6：FORMATION_CIRCLE 5.0 8.0 ========"
    broadcast_cmd "FORMATION_CIRCLE 5.0 8.0" 15
    for ns in fmu1_agent fmu2_agent fmu3_agent; do
        read_agent_ned "$ns" > "${LOG_DIR}/ned_${ns}.txt" 2>/dev/null || true
    done
    X1=$(awk '{print $1}' "${LOG_DIR}/ned_fmu1_agent.txt" 2>/dev/null || echo 0)
    Y1=$(awk '{print $2}' "${LOG_DIR}/ned_fmu1_agent.txt" 2>/dev/null || echo 0)
    Z1=$(awk '{print $3}' "${LOG_DIR}/ned_fmu1_agent.txt" 2>/dev/null || echo 0)
    X2=$(awk '{print $1}' "${LOG_DIR}/ned_fmu2_agent.txt" 2>/dev/null || echo 0)
    Y2=$(awk '{print $2}' "${LOG_DIR}/ned_fmu2_agent.txt" 2>/dev/null || echo 0)
    Z2=$(awk '{print $3}' "${LOG_DIR}/ned_fmu2_agent.txt" 2>/dev/null || echo 0)
    X3=$(awk '{print $1}' "${LOG_DIR}/ned_fmu3_agent.txt" 2>/dev/null || echo 0)
    Y3=$(awk '{print $2}' "${LOG_DIR}/ned_fmu3_agent.txt" 2>/dev/null || echo 0)
    Z3=$(awk '{print $3}' "${LOG_DIR}/ned_fmu3_agent.txt" 2>/dev/null || echo 0)
    log_info "fmu1:(${X1},${Y1},${Z1}) fmu2:(${X2},${Y2},${Z2}) fmu3:(${X3},${Y3},${Z3})"
    d12=$(python3 -c "import math; print(math.sqrt((${X1}-${X2})**2+(${Y1}-${Y2})**2+(${Z1}-${Z2})**2))" 2>/dev/null||echo N/A)
    d13=$(python3 -c "import math; print(math.sqrt((${X1}-${X3})**2+(${Y1}-${Y3})**2+(${Z1}-${Z3})**2))" 2>/dev/null||echo N/A)
    d23=$(python3 -c "import math; print(math.sqrt((${X2}-${X3})**2+(${Y2}-${Y3})**2+(${Z2}-${Z3})**2))" 2>/dev/null||echo N/A)
    log_info "d12=${d12} d13=${d13} d23=${d23}"
    ALL_OK=true
    for d in "$d12" "$d13" "$d23"; do
        [ "$d" = "N/A" ] && ALL_OK=false
        [ "$(echo "$d < 3.0" | bc -l 2>/dev/null || echo 0)" = "1" ] && ALL_OK=false
    done
    $ALL_OK && log_pass "間距 ≥3m" || log_fail "間距不足"
}

phase7_offline() {
    echo ""; echo "======== Phase 7：離線測試 ========"
    log_info "停止 drone3..."
    docker stop drone3 2>/dev/null || true
    sleep 15
    for agent in fmu1_agent fmu2_agent; do
        docker logs "$agent" 2>&1 | tail -10 | grep -qi "peer.*offline\|recalc\|lost" \
            && log_pass "${agent} 偵測離線" || log_warn "${agent} 未偵測"
    done
}

phase8_cleanup() {
    echo ""; echo "======== Phase 8：清理 ========"
    ros2_exec fmu1_agent "ros2 topic list" > "${LOG_DIR}/topics.txt" 2>&1 || true
    docker compose -f "${PROJECT_DIR}/docker-compose.yml" logs > "${LOG_DIR}/agent_logs.txt" 2>&1 || true
    docker rm -f drone1 drone2 drone3 2>/dev/null || true
    echo ""; echo "通過:${PASS_COUNT} 失敗:${FAIL_COUNT} 日誌:${LOG_DIR}"
    [ "$FAIL_COUNT" -eq 0 ] && echo -e "${GREEN}全部通過 ✓${NC}" || echo -e "${RED}${FAIL_COUNT} 項失敗 ✗${NC}"
    return $FAIL_COUNT
}

main() { phase1_start; phase2_ready; phase3_offboard; phase4_arm; phase5_takeoff; phase6_formation; phase7_offline; phase8_cleanup; }
main "$@"
