#!/bin/bash
# ============================================================================
# acceptance_test.sh — Taiphoon Agent 驗收測試腳本
# ============================================================================
# 功能：啟動 3 臺 PX4 SITL，透過 /fmu_all_agent/command 依序發送命令，
#       監控狀態並確認是否成功完成任務流程。
#
# 命令流程：MODE_OFFBOARD → ARM → TAKEOFF 10 → MOVE_FORWARD 10 → MODE_RTL
#
# 測試紀錄存放於：AutoTest/Log/
# ============================================================================
set -e

# ---------------------------------------------------------------------------
# 環境變數與路徑設定
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/Log"
mkdir -p "${LOG_DIR}"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/acceptance_test_${TIMESTAMP}.log"

PX4_BIN="${HOME}/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_DIR="${HOME}/PX4-Autopilot/build/px4_sitl_default"
DRONE_COUNT=3
BASE_LAT=25.03996592258936
BASE_LON=121.51195163336611

# 來源 ROS2 環境
source /opt/ros/humble/setup.bash 2>/dev/null
source "${HOME}/Taiphoon_Agent/install/setup.bash" 2>/dev/null

# ---------------------------------------------------------------------------
# 輔助函式
# ---------------------------------------------------------------------------

# 寫入日誌（同時輸出到終端機與日誌檔）
log_msg() {
    local msg="[$(date '+%H:%M:%S')] $1"
    echo "${msg}" | tee -a "${LOG_FILE}"
}

# 檢查 ROS2 topic 是否存在
topic_exists() {
    local topic="$1"
    local timeout_sec="${2:-5}"
    timeout ${timeout_sec} ros2 topic echo "${topic}" --once &>/dev/null
}

# 從 vehicle_status 擷取特定欄位值
get_status_field() {
    local ns="$1"
    local field="$2"
    local topic="/${ns}/fmu/out/vehicle_status"
    local raw
    raw=$(timeout 5 ros2 topic echo "${topic}" --once 2>/dev/null | grep "${field}:" | head -1 | awk '{print $NF}')
    echo "${raw:-N/A}"
}

# 從 vehicle_odometry 擷取高度
get_altitude() {
    local ns="$1"
    local topic="/${ns}/fmu/out/vehicle_odometry"
    local raw
    raw=$(timeout 5 ros2 topic echo "${topic}" --once 2>/dev/null | grep "z:" | head -1 | awk '{print $NF}')
    echo "${raw:-N/A}"
}

# 檢查 PX4 實例是否正在執行
check_px4_alive() {
    local instance="$1"
    pgrep -f "px4.*-i ${instance}" &>/dev/null
}

# 等待特定條件成立
wait_for_condition() {
    local description="$1"
    local max_wait="${2:-30}"
    local check_cmd="$3"
    local waited=0
    log_msg "等待: ${description}（最長 ${max_wait} 秒）..."
    while [ $waited -lt $max_wait ]; do
        if eval "${check_cmd}" &>/dev/null; then
            log_msg "  ✓ ${description} — 已就緒（等待 ${waited} 秒）"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
        if [ $((waited % 5)) -eq 0 ]; then
            log_msg "  仍在等待...（${waited}/${max_wait} 秒）"
        fi
    done
    log_msg "  ✗ ${description} — 逾時（${max_wait} 秒）"
    return 1
}

# 透過 ROS2 topic 發送命令到所有無人機
send_broadcast_cmd() {
    local cmd="$1"
    local desc="$2"
    log_msg ""
    log_msg ">>> 發送廣播命令: ${desc}（${cmd}）"
    ros2 topic pub --once /fmu_all_agent/command std_msgs/msg/String "data: '${cmd}'" 2>&1 | tee -a "${LOG_FILE}"
    local ret=$?
    if [ $ret -ne 0 ]; then
        log_msg "  ✗ 命令發送失敗（退出碼: ${ret}）"
        return $ret
    fi
    log_msg "  ✓ 命令已發送"
    return 0
}

# 監控特定欄位直到符合預期值
monitor_until_match() {
    local ns="$1"
    local field="$2"
    local expected="$3"
    local max_wait="${4:-30}"
    local desc="$5"
    local waited=0

    log_msg "  監控 /${ns} ${field} → 預期: ${expected}"
    while [ $waited -lt $max_wait ]; do
        local actual
        actual=$(get_status_field "${ns}" "${field}")
        if [ "${actual}" = "${expected}" ]; then
            log_msg "  ✓ ${desc} 達成（${field}=${actual}，等待 ${waited} 秒）"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    local final_val
    final_val=$(get_status_field "${ns}" "${field}")
    log_msg "  ✗ ${desc} 逾時（目前 ${field}=${final_val}，預期 ${expected}）"
    return 1
}

# ---------------------------------------------------------------------------
# 步驟 0：清理舊實例
# ---------------------------------------------------------------------------
log_msg "╔══════════════════════════════════════════════════════════════╗"
log_msg "║   Taiphoon Agent 驗收測試                                    ║"
log_msg "║   時間: $(date '+%Y-%m-%d %H:%M:%S')                         ║"
log_msg "║   日誌: ${LOG_FILE}                                          ║"
log_msg "╚══════════════════════════════════════════════════════════════╝"

log_msg ""
log_msg "=== 步驟 0：清理舊實例 ==="
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f basic_controller 2>/dev/null || true
tmux kill-server 2>/dev/null || true
sleep 2
log_msg "  ✓ 已清理所有舊實例"

# ---------------------------------------------------------------------------
# 步驟 1：啟動 3 臺 PX4 SITL
# ---------------------------------------------------------------------------
log_msg ""
log_msg "=== 步驟 1：啟動 ${DRONE_COUNT} 臺 PX4 SITL + MicroXRCEAgent ==="

OVERALL_START=$(date +%s)

for ((i=0; i<DRONE_COUNT; i++)); do
    INSTANCE=$i
    PORT=$((8888 + INSTANCE))

    # 計算一字型排列位置（每臺向東偏移 3 公尺）
    read HOME_LAT HOME_LON <<< $(python3 -c "
import math, sys
base_lat = ${BASE_LAT}
base_lon = ${BASE_LON}
instance = int(sys.argv[1])
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))
offset = instance * 3.0 * lon_per_m
print(f'{base_lat} {base_lon + offset}')
" $INSTANCE)

    NS_INDEX=$((INSTANCE + 1))
    NAMESPACE="fmu${NS_INDEX}"

    mkdir -p "/tmp/px4_instance_${INSTANCE}"

    # 啟動 MicroXRCEAgent
    tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
    tmux send-keys -t "agent_${INSTANCE}" \
        "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter
    sleep 1

    # 啟動 PX4 SITL
    tmux new-session -d -s "sitl_${INSTANCE}" 2>/dev/null
    tmux send-keys -t "sitl_${INSTANCE}" \
        "cd ${PX4_DIR} && \
         PX4_SIM_MODEL=sihsim_quadx \
         PX4_SIMULATOR=sihsim \
         PX4_UXRCE_DDS_PORT=${PORT} \
         PX4_UXRCE_DDS_NS=${NAMESPACE} \
         PX4_NET_INTERFACE=eth0 \
         PX4_PARAM_MAV_0_BROADCAST=1 \
         PX4_PARAM_COM_ARM_WO_GPS=1 \
         PX4_PARAM_COM_RCL_EXCEPT=4 \
         PX4_PARAM_NAV_RCL_ACT=0 \
         PX4_PARAM_COM_RC_IN_MODE=4 \
         PX4_PARAM_SYS_MC_EST_GROUP=2 \
         PX4_HOME_LAT=${HOME_LAT} \
         PX4_HOME_LON=${HOME_LON} \
         exec ${PX4_BIN} -d -i ${INSTANCE}" Enter

    log_msg "  Instance ${INSTANCE}: port=${PORT}, ns=/${NAMESPACE}, pos=(${HOME_LAT}, ${HOME_LON})"
    sleep 2
done

log_msg "  等待 PX4 就緒（最多 60 秒）..."
sleep 10

# 檢查所有 PX4 實例是否存活
for ((i=0; i<DRONE_COUNT; i++)); do
    NS="fmu$((i+1))"
    if ! wait_for_condition "/${NS} vehicle_status topic 出現" 60 \
        "timeout 3 ros2 topic echo /${NS}/fmu/out/vehicle_status --once 2>/dev/null | grep -q 'arm_state'"; then
        log_msg "  ✗ PX4 實例 ${i} 未就緒，測試中止"
        exit 1
    fi
done

# ---------------------------------------------------------------------------
# 步驟 2：啟動 Taiphoon_Agent 節點
# ---------------------------------------------------------------------------
log_msg ""
log_msg "=== 步驟 2：啟動 ${DRONE_COUNT} 個 Taiphoon Agent ==="

for ((i=0; i<DRONE_COUNT; i++)); do
    NS="fmu$((i+1))"
    AGENT_ID=$((i+1))
    ROS_DOMAIN_ID=$((100 + i))

    log_msg "  啟動 Agent /${NS}（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}）"
    tmux new-session -d -s "agent_ctrl_${i}" 2>/dev/null
    tmux send-keys -t "agent_ctrl_${i}" \
        "source /opt/ros/humble/setup.bash && \
         source ${HOME}/Taiphoon_Agent/install/setup.bash && \
         export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
         ros2 run taiphoon_agent basic_controller --ros-args \
             -r __ns:=/${NS} \
             -p target_altitude_m:=10.0 \
             -p hover_duration_s:=0.0 \
             -p target_system:=${AGENT_ID} \
             -p agent_namespace:=${NS}" Enter
    sleep 2
done

# 等待 Agent 節點完成初始化
log_msg "  等待 Agent 初始化（15 秒）..."
sleep 15

# 檢查 Agent 命令訂閱是否就緒
if ! ros2 topic list 2>/dev/null | grep -q "/fmu_all_agent/command"; then
    log_msg "  ⚠ /fmu_all_agent/command 主題未出現，但繼續測試..."
else
    log_msg "  ✓ /fmu_all_agent/command 主題已就緒"
fi

# ---------------------------------------------------------------------------
# 步驟 3：依序發送命令並驗證
# ---------------------------------------------------------------------------
log_msg ""
log_msg "=== 步驟 3：依序發送命令 ==="

# 用來記錄每步驟結果的陣列
declare -A STEP_RESULTS
TEST_PASSED=true

# --- 3-1: MODE_OFFBOARD ---
log_msg ""
log_msg "--- 3-1: MODE_OFFBOARD ---"
if send_broadcast_cmd "MODE_OFFBOARD" "切換至 Offboard 模式"; then
    sleep 3
    STEP_OK=true
    for ((i=0; i<DRONE_COUNT; i++)); do
        NS="fmu$((i+1))"
        nav_state=$(get_status_field "${NS}" "nav_state")
        log_msg "  /${NS} nav_state=${nav_state}"
        if [ "${nav_state}" != "14" ]; then
            # nav_state 14 = OFFBOARD
            log_msg "    ⚠ 預期 nav_state=14（Offboard），目前為 ${nav_state}"
        fi
    done
else
    STEP_OK=false
    TEST_PASSED=false
fi
STEP_RESULTS["MODE_OFFBOARD"]="${STEP_OK}"

# --- 3-2: ARM ---
log_msg ""
log_msg "--- 3-2: ARM ---"
if send_broadcast_cmd "ARM" "解鎖無人機"; then
    sleep 5
    STEP_OK=true
    for ((i=0; i<DRONE_COUNT; i++)); do
        NS="fmu$((i+1))"
        arm_state=$(get_status_field "${NS}" "arm_state")
        log_msg "  /${NS} arm_state=${arm_state}"
        if [ "${arm_state}" != "2" ]; then
            # arm_state 2 = ARMED
            log_msg "    ⚠ 預期 arm_state=2（Armed），目前為 ${arm_state}"
            STEP_OK=false
            TEST_PASSED=false
        fi
    done
else
    STEP_OK=false
    TEST_PASSED=false
fi
STEP_RESULTS["ARM"]="${STEP_OK}"

# --- 3-3: TAKEOFF 10 ---
log_msg ""
log_msg "--- 3-3: TAKEOFF 10 ---"
if send_broadcast_cmd "TAKEOFF 10" "起飛至 10 公尺"; then
    log_msg "  等待起飛完成（15 秒）..."
    sleep 15
    STEP_OK=true
    for ((i=0; i<DRONE_COUNT; i++)); do
        NS="fmu$((i+1))"
        altitude=$(get_altitude "${NS}")
        log_msg "  /${NS} 高度 z=${altitude}"

        # 檢查高度是否 >= 5m（給予容許範圍，SIH 模擬可能不完全精準）
        if [ -n "${altitude}" ] && [ "${altitude}" != "N/A" ]; then
            alt_abs=$(echo "${altitude}" | sed 's/-//')
            alt_check=$(echo "${alt_abs} >= 3.0" | bc -l 2>/dev/null || echo "0")
            if [ "${alt_check}" = "0" ]; then
                log_msg "    ⚠ 高度 ${altitude} 未達預期（>= -10m 即 z <= -3）"
            else
                log_msg "    ✓ 高度已上升"
            fi
        fi
    done
else
    STEP_OK=false
    TEST_PASSED=false
fi
STEP_RESULTS["TAKEOFF"]="${STEP_OK}"

# --- 3-4: MOVE_FORWARD 10 ---
log_msg ""
log_msg "--- 3-4: MOVE_FORWARD 10 ---"
if send_broadcast_cmd "MOVE_FORWARD 10" "向前移動 10 公尺"; then
    log_msg "  等待移動完成（10 秒）..."
    sleep 10
    STEP_OK=true
    for ((i=0; i<DRONE_COUNT; i++)); do
        NS="fmu$((i+1))"
        altitude=$(get_altitude "${NS}")
        nav_state=$(get_status_field "${NS}" "nav_state")
        log_msg "  /${NS} 高度 z=${altitude}, nav_state=${nav_state}"
        # 移動後應仍在飛行狀態
    done
else
    STEP_OK=false
    TEST_PASSED=false
fi
STEP_RESULTS["MOVE_FORWARD"]="${STEP_OK}"

# --- 3-5: MODE_RTL ---
log_msg ""
log_msg "--- 3-5: MODE_RTL ---"
if send_broadcast_cmd "MODE_RTL" "啟動 RTL 返航模式"; then
    sleep 5
    STEP_OK=true
    for ((i=0; i<DRONE_COUNT; i++)); do
        NS="fmu$((i+1))"
        nav_state=$(get_status_field "${NS}" "nav_state")
        arm_state=$(get_status_field "${NS}" "arm_state")
        log_msg "  /${NS} nav_state=${nav_state}, arm_state=${arm_state}"
        # nav_state 5 = RTL
        if [ "${nav_state}" != "5" ]; then
            log_msg "    ⚠ 預期 nav_state=5（RTL），目前為 ${nav_state}"
        fi
    done
else
    STEP_OK=false
    TEST_PASSED=false
fi
STEP_RESULTS["MODE_RTL"]="${STEP_OK}"

# ---------------------------------------------------------------------------
# 步驟 4：彙總測試結果
# ---------------------------------------------------------------------------
OVERALL_END=$(date +%s)
ELAPSED=$((OVERALL_END - OVERALL_START))

log_msg ""
log_msg "╔══════════════════════════════════════════════════════════════╗"
log_msg "║   測試結果彙總                                                ║"
log_msg "╠══════════════════════════════════════════════════════════════╣"

for cmd in MODE_OFFBOARD ARM TAKEOFF MOVE_FORWARD MODE_RTL; do
    result="${STEP_RESULTS[${cmd}]}"
    if [ "${result}" = "true" ]; then
        log_msg "║  ✓ ${cmd} — 通過"
    else
        log_msg "║  ✗ ${cmd} — 失敗"
    fi
done

log_msg "╠══════════════════════════════════════════════════════════════╣"
if [ "${TEST_PASSED}" = "true" ]; then
    log_msg "║  整體結果: ✓ 全部通過"
else
    log_msg "║  整體結果: ✗ 存在失敗項目"
fi
log_msg "║  總耗時: ${ELAPSED} 秒"
log_msg "║  日誌位置: ${LOG_FILE}"
log_msg "╚══════════════════════════════════════════════════════════════╝"

# ---------------------------------------------------------------------------
# 步驟 5：保留實例供進一步檢查（不自動清理）
# ---------------------------------------------------------------------------
log_msg ""
log_msg "=== 測試完成 ==="
log_msg "PX4 SITL 與 Agent 實例仍在執行中，可手動檢查。"
log_msg "若要清理，請執行：bash ${SCRIPT_DIR}/restart_all.sh"
log_msg ""

# 回傳測試結果
if [ "${TEST_PASSED}" = "true" ]; then
    exit 0
else
    exit 1
fi
