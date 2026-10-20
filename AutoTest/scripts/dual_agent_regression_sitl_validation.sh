#!/usr/bin/env bash
# @file dual_agent_regression_sitl_validation.sh
# @brief 多機 SITL 回歸：起飛懸停、單機 MOVE、YAW、一字編隊、全體 MOVE、
#        燕行隊形切換、取消編隊、時鐘偏差診斷與降落
# @author Taiphoon Agent Team
# @date 2026-10-20
#
# 用法（於專案根目錄）：
#   bash AutoTest/scripts/dual_agent_regression_sitl_validation.sh [--count 2|3|4]
#   bash AutoTest/scripts/dual_agent_regression_sitl_validation.sh --agent-ids 2,3
#   bash AutoTest/scripts/dual_agent_regression_sitl_validation.sh --count 3 --deploy-runtime --obstacle-source none
#   檔名沿用雙機版本；預設 uav_1～uav_2，最終展示以 --count 3 驗收。
#   --agent-ids：只啟動指定機號（例如缺少 uav_1 的 2,3 或單機 2）。
#   --deploy-runtime：改用樹莓派實機相同的 runtime-entrypoint.sh 與規劃參數。
#   --obstacle-source：障礙物地圖來源，與實機 settings.ini 的 obstacle_source 對應：
#     explicit_empty（預設）：啟動模擬器發布空地圖，執行完整回歸。
#     none：不提供地圖（實機預設），驗證起飛／YAW／降落正常且 MOVE 被擋下。
#     manual：以 none 啟動後，用 rc.8 的 docker exec 暫時做法手動啟動空地圖節點，執行完整回歸。
# Windows Git Bash 需先設定：
#   export MSYS_NO_PATHCONV=1 MSYS2_ENV_CONV_EXCL=SSH_AUTH_SOCK
#   export SSH_AUTH_SOCK=/run/host-services/ssh-auth.sock
#
# 依 AGENTS.md 順序先啟動 Agent，再啟動 PX4 SITL，等待 10 秒後才起飛。
# 飛行步驟由 dual_agent_regression_monitor.py 在第一台 Agent 容器內執行；
# 本腳本另外從 Agent 日誌確認各同儕的時鐘差 INFO 與偏差注入 WARN。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NETWORK_NAME="Taiphoon_Net"
LOG_DIR="${PROJECT_DIR}/AutoTest/Log"
AGENT_STARTED=0
COUNT=2
AGENT_ID_LIST=""
OBSTACLE_SOURCE="explicit_empty"
COMPOSE_FILES=(-f docker-compose.yml)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --count) COUNT="$2"; shift 2 ;;
        --agent-ids) AGENT_ID_LIST="$2"; shift 2 ;;
        --obstacle-source) OBSTACLE_SOURCE="$2"; shift 2 ;;
        --deploy-runtime)
            COMPOSE_FILES+=(-f AutoTest/scripts/compose.deploy-runtime.yml)
            shift ;;
        *) echo "錯誤：不支援的參數：$1" >&2; exit 2 ;;
    esac
done
case "${OBSTACLE_SOURCE}" in
    explicit_empty) ENABLE_OBSTACLE_CLOUD=true ;;
    none|manual) ENABLE_OBSTACLE_CLOUD=false ;;
    *) echo "錯誤：--obstacle-source 必須是 explicit_empty、none 或 manual" >&2; exit 2 ;;
esac
MONITOR_EXTRA_ARGS=""
if [[ "${OBSTACLE_SOURCE}" == "none" ]]; then
    MONITOR_EXTRA_ARGS="--expect-move-blocked"
fi

if [[ -n "${AGENT_ID_LIST}" ]]; then
    if [[ ! "${AGENT_ID_LIST}" =~ ^[1-4](,[1-4])*$ ]]; then
        echo "錯誤：--agent-ids 必須是 1～4 的機號，以逗號分隔，例如 2,3" >&2
        exit 2
    fi
    AGENT_IDS="${AGENT_ID_LIST//,/ }"
else
    case "${COUNT}" in
        2|3|4) ;;
        *) echo "錯誤：--count 必須是 2、3 或 4" >&2; exit 2 ;;
    esac
    AGENT_IDS="$(seq 1 "${COUNT}" | tr '\n' ' ')"
    AGENT_ID_LIST="$(seq -s, 1 "${COUNT}")"
fi
AGENT_SERVICES=()
for agent_id in ${AGENT_IDS}; do
    AGENT_SERVICES+=("uav_${agent_id}_agent")
    # docker-compose.yml 以 UAV_<N>_ENABLE_OBSTACLE_CLOUD_SIMULATOR 控制障礙物模擬器。
    export "UAV_${agent_id}_ENABLE_OBSTACLE_CLOUD_SIMULATOR=${ENABLE_OBSTACLE_CLOUD}"
done
AGENT_COUNT=${#AGENT_SERVICES[@]}
FIRST_AGENT="${AGENT_SERVICES[0]}"

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
        docker compose "${COMPOSE_FILES[@]}" stop "${AGENT_SERVICES[@]}" >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}

cd "${PROJECT_DIR}"
RUN_DIR="${LOG_DIR}/agent_regression_${AGENT_ID_LIST//,/-}_$(date +%Y%m%d_%H%M%S)"
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

echo "=== 啟動 Agent：${AGENT_SERVICES[*]}（obstacle_source=${OBSTACLE_SOURCE}，compose：${COMPOSE_FILES[*]}）==="
# 清掉不在本次組合內、但仍在執行的 Agent，避免其他機號干擾。
docker compose "${COMPOSE_FILES[@]}" stop >/dev/null 2>&1 || true
docker compose "${COMPOSE_FILES[@]}" up -d "${AGENT_SERVICES[@]}"
AGENT_STARTED=1

echo "=== 等待 ${AGENT_COUNT} 台 Agent planner node ready ==="
deadline=$((SECONDS + 300))
until docker exec -i "${FIRST_AGENT}" bash -lc \
    "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | \
    grep -c "/ego_swarm_planner" | grep -q "^${AGENT_COUNT}$"; do
    if (( SECONDS >= deadline )); then
        echo "錯誤：300 秒內 ${AGENT_COUNT} 台 Agent 未就緒" >&2
        exit 1
    fi
    sleep 2
done

if [[ "${OBSTACLE_SOURCE}" == "manual" ]]; then
    echo "=== rc.8 暫時做法：在每台 Agent 容器內手動啟動 explicit_empty 空地圖節點 ==="
    for agent_id in ${AGENT_IDS}; do
        # 與 docs/obstacle-source.md 記載的實機指令相同，只把容器選擇改為 SITL 容器名稱。
        docker exec -d "uav_${agent_id}_agent" bash -c "source /opt/ros/humble/setup.bash && source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && source /home/taiphoon/Taiphoon_Agent/install/setup.bash && exec ros2 run taiphoon_agent obstacle_cloud_simulator --ros-args -r __ns:=/uav_${agent_id}_agent -p agent_namespace:=uav_${agent_id}_agent -p mode:=explicit_empty"
    done
    deadline=$((SECONDS + 60))
    until [[ "$(docker exec -i "${FIRST_AGENT}" bash -lc \
        "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | \
        grep -c "/obstacle_cloud_simulator")" -eq "${AGENT_COUNT}" ]]; do
        if (( SECONDS >= deadline )); then
            echo "錯誤：60 秒內手動空地圖節點未全部啟動" >&2
            exit 1
        fi
        sleep 2
    done
fi

echo "=== 啟動 PX4 SITL：${AGENT_IDS}==="
bash "${SCRIPT_DIR}/px4-sitl-launcher/stop_all.sh" --net "${NETWORK_NAME}" >/dev/null 2>&1 || true
for agent_id in ${AGENT_IDS}; do
    # PX4 index 從 0 起：uav_N 對應 index N-1，起始位置與 launch_all.sh 相同。
    bash "${SCRIPT_DIR}/px4-sitl-launcher/start_single_instance.sh" \
        --index "$((agent_id - 1))" --net "${NETWORK_NAME}" --rmw cyclonedds \
        --agent "uav_${agent_id}_agent"
    sleep 2
done

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

echo "=== 執行回歸步驟：uav ${AGENT_ID_LIST} ==="
set +e
docker exec -i "${FIRST_AGENT}" bash -lc \
    "source /opt/ros/humble/setup.bash && \
     source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
     source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
     python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/dual_agent_regression_monitor.py \
     --agent-ids ${AGENT_ID_LIST} ${MONITOR_EXTRA_ARGS}" \
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
    echo "[FAIL] uav ${AGENT_ID_LIST} SITL 回歸未通過；日誌：${RUN_DIR}"
    exit 1
fi
echo "[PASS] uav ${AGENT_ID_LIST} SITL 回歸完成；日誌：${RUN_DIR}"
