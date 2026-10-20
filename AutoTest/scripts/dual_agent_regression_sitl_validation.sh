#!/usr/bin/env bash
# @file dual_agent_regression_sitl_validation.sh
# @brief 多機（2～4 台）SITL 回歸：起飛懸停、單機 MOVE、YAW、一字編隊、全體 MOVE、
#        燕行隊形切換、取消編隊、時鐘偏差診斷與降落
# @author Taiphoon Agent Team
# @date 2026-10-20
#
# 用法（於專案根目錄）：
#   bash AutoTest/scripts/dual_agent_regression_sitl_validation.sh [--count 2|3|4]
#   檔名沿用雙機版本；預設 2 台，最終展示以 --count 3 驗收。
# Windows Git Bash 需先設定：
#   export MSYS_NO_PATHCONV=1 MSYS2_ENV_CONV_EXCL=SSH_AUTH_SOCK
#   export SSH_AUTH_SOCK=/run/host-services/ssh-auth.sock
#
# 依 AGENTS.md 順序先啟動 Agent，再啟動 PX4 SITL，等待 10 秒後才起飛。
# 飛行步驟由 dual_agent_regression_monitor.py 在 uav_1_agent 容器內執行；
# 本腳本另外從 Agent 日誌確認各同儕的時鐘差 INFO 與偏差注入 WARN。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NETWORK_NAME="Taiphoon_Net"
LOG_DIR="${PROJECT_DIR}/AutoTest/Log"
AGENT_STARTED=0
COUNT=2

while [[ $# -gt 0 ]]; do
    case "$1" in
        --count) COUNT="$2"; shift 2 ;;
        *) echo "錯誤：不支援的參數：$1" >&2; exit 2 ;;
    esac
done
case "${COUNT}" in
    2|3|4) ;;
    *) echo "錯誤：--count 必須是 2、3 或 4" >&2; exit 2 ;;
esac
AGENT_IDS="$(seq 1 "${COUNT}")"
AGENT_SERVICES=()
for agent_id in ${AGENT_IDS}; do
    AGENT_SERVICES+=("uav_${agent_id}_agent")
done

cleanup()
{
    local exit_code=$?
    echo "=== 保存 Agent 日誌並清理 PX4 SITL 與 Agent ==="
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        for agent_id in ${AGENT_IDS}; do
            docker logs "uav_${agent_id}_agent" \
                > "${RUN_DIR}/uav_${agent_id}_agent.log" 2>&1 || true
        done
    fi
    bash "${SCRIPT_DIR}/px4-sitl-launcher/stop_all.sh" \
        --net "${NETWORK_NAME}" >/dev/null 2>&1 || true
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        docker compose stop "${AGENT_SERVICES[@]}" >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}

cd "${PROJECT_DIR}"
RUN_DIR="${LOG_DIR}/agent_regression_${COUNT}x_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RUN_DIR}"
trap cleanup EXIT INT TERM

if ! docker image inspect taiphoon-agent:dev >/dev/null 2>&1; then
    echo "錯誤：找不到 taiphoon-agent:dev，請先建置 image：" >&2
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&2
    exit 2
fi
if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
    docker network create "${NETWORK_NAME}" >/dev/null
fi

echo "=== 啟動 ${COUNT} 台 Agent ==="
docker compose stop "${AGENT_SERVICES[@]}" >/dev/null 2>&1 || true
docker compose up -d "${AGENT_SERVICES[@]}"
AGENT_STARTED=1

echo "=== 等待 ${COUNT} 台 Agent planner node ready ==="
deadline=$((SECONDS + 300))
until docker exec -i uav_1_agent bash -lc \
    "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | \
    grep -c "/ego_swarm_planner" | grep -q "^${COUNT}$"; do
    if (( SECONDS >= deadline )); then
        echo "錯誤：300 秒內 ${COUNT} 台 Agent 未就緒" >&2
        exit 1
    fi
    sleep 2
done

echo "=== 啟動 ${COUNT} 台 PX4 SITL ==="
bash "${SCRIPT_DIR}/px4-sitl-launcher/launch_all.sh" \
    --count "${COUNT}" --net "${NETWORK_NAME}" --rmw cyclonedds

px4_ready()
{
    local agent_id
    for agent_id in ${AGENT_IDS}; do
        docker logs "drone${agent_id}" 2>&1 | grep -q "Ready for takeoff" || return 1
    done
}

echo "=== 等待 PX4 uXRCE-DDS 連線與 Ready for takeoff ==="
deadline=$((SECONDS + 120))
until px4_ready; do
    if (( SECONDS >= deadline )); then
        echo "錯誤：120 秒內 PX4 未就緒" >&2
        exit 1
    fi
    sleep 2
done
sleep 10

echo "=== 執行 ${COUNT} 機回歸步驟 ==="
set +e
docker exec -i uav_1_agent bash -lc \
    "source /opt/ros/humble/setup.bash && \
     source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
     source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
     python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/dual_agent_regression_monitor.py \
     --agents ${COUNT}" \
    | tee "${RUN_DIR}/monitor.log"
monitor_status=${PIPESTATUS[0]}
set -e

echo "=== 檢查 Agent 時鐘差診斷日誌 ==="
log_status=0
for agent_id in ${AGENT_IDS}; do
    agent_log="$(docker logs "uav_${agent_id}_agent" 2>&1)"
    for peer_id in ${AGENT_IDS}; do
        if [[ "${peer_id}" -eq "${agent_id}" ]]; then
            continue
        fi
        offset_line="$(grep "Peer clock offset agent=${peer_id}:" <<<"${agent_log}" | tail -1 || true)"
        if [[ -z "${offset_line}" ]]; then
            echo "[FAIL] uav_${agent_id} 未輸出 agent=${peer_id} 的時鐘差估計"
            log_status=1
        else
            echo "[PASS] uav_${agent_id}：${offset_line##*]: }"
        fi
    done
    skew_line="$(grep -m1 "Peer clock skew agent=7" <<<"${agent_log}" || true)"
    if [[ -z "${skew_line}" ]]; then
        echo "[FAIL] uav_${agent_id} 未對偏差注入的 agent=7 輸出 WARN"
        log_status=1
    else
        echo "[PASS] uav_${agent_id}：${skew_line##*]: }"
    fi
done

if [[ "${monitor_status}" -ne 0 || "${log_status}" -ne 0 ]]; then
    echo "[FAIL] ${COUNT} 機 SITL 回歸未通過；日誌：${RUN_DIR}"
    exit 1
fi
echo "[PASS] ${COUNT} 機 SITL 回歸完成；日誌：${RUN_DIR}"
