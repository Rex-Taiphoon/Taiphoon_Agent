#!/usr/bin/env bash
# @file dual_agent_regression_sitl_validation.sh
# @brief 雙機 SITL 回歸：起飛懸停、單機 MOVE、YAW、編隊、全體 MOVE、取消編隊、時鐘偏差診斷與降落
# @author Taiphoon Agent Team
# @date 2026-10-20
#
# 用法（於專案根目錄）：
#   bash AutoTest/scripts/dual_agent_regression_sitl_validation.sh
# Windows Git Bash 需先設定：
#   export MSYS_NO_PATHCONV=1 MSYS2_ENV_CONV_EXCL=SSH_AUTH_SOCK
#   export SSH_AUTH_SOCK=/run/host-services/ssh-auth.sock
#
# 依 AGENTS.md 順序先啟動 Agent，再啟動 PX4 SITL，等待 10 秒後才起飛。
# 飛行步驟由 dual_agent_regression_monitor.py 在 uav_1_agent 容器內執行；
# 本腳本另外從 Agent 日誌確認時鐘差 INFO 與偏差注入 WARN。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NETWORK_NAME="Taiphoon_Net"
LOG_DIR="${PROJECT_DIR}/AutoTest/Log"
AGENT_STARTED=0

cleanup()
{
    local exit_code=$?
    echo "=== 保存 Agent 日誌並清理 PX4 SITL 與 Agent ==="
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        docker logs uav_1_agent > "${RUN_DIR}/uav_1_agent.log" 2>&1 || true
        docker logs uav_2_agent > "${RUN_DIR}/uav_2_agent.log" 2>&1 || true
    fi
    bash "${SCRIPT_DIR}/px4-sitl-launcher/stop_all.sh" \
        --net "${NETWORK_NAME}" >/dev/null 2>&1 || true
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        docker compose stop uav_1_agent uav_2_agent >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}

cd "${PROJECT_DIR}"
RUN_DIR="${LOG_DIR}/dual_agent_regression_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RUN_DIR}"
trap cleanup EXIT INT TERM

if ! docker image inspect taiphoon-agent:dev >/dev/null 2>&1; then
    echo "錯誤：找不到 taiphoon-agent:dev，請由開發者自行建置 image。" >&2
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&2
    exit 2
fi
if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
    docker network create "${NETWORK_NAME}" >/dev/null
fi

echo "=== 啟動 2 台 Agent ==="
docker compose stop uav_1_agent uav_2_agent >/dev/null 2>&1 || true
docker compose up -d uav_1_agent uav_2_agent
AGENT_STARTED=1

echo "=== 等待兩台 Agent planner node ready ==="
deadline=$((SECONDS + 300))
until docker exec -i uav_1_agent bash -lc \
    "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | \
    grep -c "/ego_swarm_planner" | grep -q "^2$"; do
    if (( SECONDS >= deadline )); then
        echo "錯誤：300 秒內兩台 Agent 未就緒" >&2
        exit 1
    fi
    sleep 2
done

echo "=== 啟動 2 台 PX4 SITL ==="
bash "${SCRIPT_DIR}/px4-sitl-launcher/launch_all.sh" \
    --count 2 --net "${NETWORK_NAME}" --rmw cyclonedds

echo "=== 等待 PX4 uXRCE-DDS 連線與 Ready for takeoff ==="
deadline=$((SECONDS + 120))
until docker logs drone1 2>&1 | grep -q "Ready for takeoff" && \
      docker logs drone2 2>&1 | grep -q "Ready for takeoff"; do
    if (( SECONDS >= deadline )); then
        echo "錯誤：120 秒內 PX4 未就緒" >&2
        exit 1
    fi
    sleep 2
done
sleep 10

echo "=== 執行雙機回歸步驟 ==="
set +e
docker exec -i uav_1_agent bash -lc \
    "source /opt/ros/humble/setup.bash && \
     source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
     source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
     python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/dual_agent_regression_monitor.py" \
    | tee "${RUN_DIR}/monitor.log"
monitor_status=${PIPESTATUS[0]}
set -e

echo "=== 檢查 Agent 時鐘差診斷日誌 ==="
log_status=0
for agent_id in 1 2; do
    peer_id=$((3 - agent_id))
    agent_log="$(docker logs "uav_${agent_id}_agent" 2>&1)"
    offset_line="$(grep -m1 "Peer clock offset agent=${peer_id}:" <<<"${agent_log}" || true)"
    if [[ -z "${offset_line}" ]]; then
        echo "[FAIL] uav_${agent_id} 未輸出 agent=${peer_id} 的時鐘差估計"
        log_status=1
    else
        echo "[PASS] uav_${agent_id}：${offset_line##*]: }"
    fi
    skew_line="$(grep -m1 "Peer clock skew agent=7" <<<"${agent_log}" || true)"
    if [[ -z "${skew_line}" ]]; then
        echo "[FAIL] uav_${agent_id} 未對偏差注入的 agent=7 輸出 WARN"
        log_status=1
    else
        echo "[PASS] uav_${agent_id}：${skew_line##*]: }"
    fi
done

if [[ "${monitor_status}" -ne 0 || "${log_status}" -ne 0 ]]; then
    echo "[FAIL] 雙機 SITL 回歸未通過；日誌：${RUN_DIR}"
    exit 1
fi
echo "[PASS] 雙機 SITL 回歸完成；日誌：${RUN_DIR}"
