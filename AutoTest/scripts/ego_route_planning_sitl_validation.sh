#!/usr/bin/env bash
# =============================================================================
# ego_route_planning_sitl_validation.sh
# EGO 式 FIXED_Z 時空路徑規劃 SITL 驗證：單機、東向避障、雙機與四機。
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NETWORK_NAME="Taiphoon_Net"
TAKEOFF_ALTITUDE_M="8.0"
FORMATION_ALTITUDE_M="10.0"
SCENARIO="all"
SKIP_BASIC=0
AGENT_STARTED=0
LOG_DIR="${PROJECT_DIR}/AutoTest/Log"

usage()
{
    cat <<'EOF'
使用方式：
  bash AutoTest/scripts/ego_route_planning_sitl_validation.sh \
      --scenario single|east|dual|four_move|swarm|all [--skip-basic]

single：BasicController 回歸、無障礙直線、固定障礙繞行、封閉搜尋範圍。
east：uav_1 起飛至 8 m 後沿 +East 移動 30 m 的無障礙與 East wall 場景。
dual：兩台以交叉 MOVE_POINT 驗證 moving peer 的時間避碰。
four_move：四台起飛後維持西向東 8 m 間距，僅 uav_1 執行 +East MOVE_RIGHT 30 m。
swarm：四台 `FORMATION_CIRCLE 10 10` 固定高度、幾何半徑與 terminal hold。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario) SCENARIO="$2"; shift 2 ;;
        --skip-basic) SKIP_BASIC=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知參數：$1" >&2; usage >&2; exit 2 ;;
    esac
done

case "${SCENARIO}" in
    single|east|dual|four_move|swarm|all) ;;
    *) echo "錯誤：不支援的 scenario：${SCENARIO}" >&2; exit 2 ;;
esac

cleanup()
{
    local exit_code=$?
    echo "=== 清理 PX4 SITL 與 Agent ==="
    bash "${SCRIPT_DIR}/px4-sitl-launcher/stop_all.sh" \
        --net "${NETWORK_NAME}" >/dev/null 2>&1 || true
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        docker compose down >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

cd "${PROJECT_DIR}"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ego_route_planning_$(date +%Y%m%d_%H%M%S).log"
exec 3>&1
echo "=== EGO SITL validation log: ${LOG_FILE} ===" >&3
exec >"${LOG_FILE}" 2>&1
echo "=== EGO SITL validation log: ${LOG_FILE} ==="

if ! docker image inspect taiphoon-agent:dev >/dev/null 2>&1; then
    echo "SITL_RESULT status=NOT RUN reason=missing taiphoon-agent:dev" >&2
    echo "SITL_RESULT status=NOT RUN reason=missing taiphoon-agent:dev" >&3
    echo "錯誤：找不到 taiphoon-agent:dev，請由開發者自行建置 image。" >&2
    echo "錯誤：找不到 taiphoon-agent:dev，請由開發者自行建置 image。" >&3
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&2
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&3
    exit 2
fi
if ! docker image inspect px4io/px4-sitl:latest >/dev/null 2>&1; then
    echo "SITL_RESULT status=NOT RUN reason=missing px4io/px4-sitl:latest" >&2
    echo "SITL_RESULT status=NOT RUN reason=missing px4io/px4-sitl:latest" >&3
    echo "錯誤：找不到 px4io/px4-sitl:latest，請先 pull 或準備 SITL image。" >&2
    echo "錯誤：找不到 px4io/px4-sitl:latest，請先 pull 或準備 SITL image。" >&3
    exit 2
fi
if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
    docker network create "${NETWORK_NAME}" >/dev/null
fi

start_agents()
{
    local count="$1"
    local obstacle_mode="${2:-explicit_empty}"
    local obstacle_config="${3:-/home/taiphoon/Taiphoon_Agent/config/obstacles/default.yaml}"
    export UAV_1_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export UAV_2_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export UAV_3_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export UAV_4_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export UAV_1_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export UAV_2_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export UAV_3_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export UAV_4_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    docker compose down >/dev/null 2>&1 || true
    if [[ "${count}" -eq 1 ]]; then
        docker compose up -d uav_1_agent
    elif [[ "${count}" -eq 2 ]]; then
        docker compose up -d uav_1_agent uav_2_agent
    else
        docker compose up -d
    fi
    AGENT_STARTED=1
    wait_for_agent_nodes "${count}"
    bash "${SCRIPT_DIR}/px4-sitl-launcher/launch_all.sh" \
        --count "${count}" --net "${NETWORK_NAME}" --rmw cyclonedds
    sleep 10
}

wait_for_agent_nodes()
{
    local count="$1"
    local deadline=$((SECONDS + 180))
    echo "=== 等待 ${count} 個 Agent planner node ready ==="
    while (( SECONDS < deadline )); do
        local all_ready=1
        local agent_id
        for agent_id in $(seq 1 "${count}"); do
            if ! docker exec -i uav_1_agent bash -lc \
                "source /opt/ros/humble/setup.bash && \
                 ros2 node list" 2>/dev/null | \
                grep -F "/uav_${agent_id}/ego_swarm_planner" >/dev/null; then
                all_ready=0
                break
            fi
        done
        if [[ "${all_ready}" -eq 1 ]]; then
            echo "=== Agent planner node 全部 ready ==="
            return 0
        fi
        sleep 2
    done
    echo "錯誤：180 秒內未等到所有 Agent planner node ready" >&2
    return 1
}

publish_command()
{
    local topic="$1"
    local command="$2"
    # 全體命令需等待所有訂閱者配對；只等 1 個時可能只配對到驗收監測程式，
    # 單次發布的命令便無法送達部分 Agent。
    local subscriptions="${3:-1}"
    docker exec -i uav_1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && ros2 topic pub \
        --wait-matching-subscriptions ${subscriptions} --once ${topic} \
        std_msgs/msg/String \"data: '${command}'\"" >/dev/null
}

run_monitor()
{
    local count="$1"
    shift
    docker exec -i uav_1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && \
         source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
         source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
         python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/ego_route_planning_monitor.py \
         --agents ${count} $*"
}

wait_for_takeoff_altitude()
{
    local count="$1"
    local altitude_m="${2:-${TAKEOFF_ALTITUDE_M}}"
    docker exec -i uav_1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && \
         source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
         source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
         python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/wait_for_takeoff_altitude.py \
         --agents ${count} --target-altitude-m ${altitude_m} \
         --tolerance-m 0.5"
}

takeoff_and_wait()
{
    local count="$1"
    local altitude_m="${2:-${TAKEOFF_ALTITUDE_M}}"
    # 先讓監測器取得地面基準，再發既有 TAKEOFF；避免命令已開始執行後
    # 才收到第一筆狀態，將半途高度誤當成基準而低估實際爬升量。
    wait_for_takeoff_altitude "${count}" "${altitude_m}" &
    local waiter_pid=$!
    sleep 2
    bash "${SCRIPT_DIR}/quick_takeoff.sh" "${altitude_m}"
    wait "${waiter_pid}"
}

run_single()
{
    if [[ "${SKIP_BASIC}" -eq 0 ]]; then
        echo "=== 單機 BasicController 回歸 ==="
        bash "${SCRIPT_DIR}/basic_controller_offboard_validation.sh"
    fi

    echo "=== 單機無障礙 FIXED_Z 直線 ==="
    start_agents 1 explicit_empty
    takeoff_and_wait 1
    sleep 1
    run_monitor 1 --timeout-s 45 --minimum-displacement-m 8 \
        --max-lateral-m 1.0 &
    local monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_FORWARD 12"
    wait "${monitor_pid}"

    echo "=== 單機固定障礙繞行 ==="
    start_agents 1 ground_truth \
        /home/taiphoon/Taiphoon_Agent/config/obstacles/fixed_wall.yaml
    takeoff_and_wait 1
    sleep 1
    run_monitor 1 --timeout-s 90 --minimum-displacement-m 12 \
        --require-lateral-m 0.5 &
    monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_FORWARD 16"
    wait "${monitor_pid}"

    echo "=== 單機封閉搜尋範圍：必須 HOLD／Brake，不得穿牆 ==="
    start_agents 1 ground_truth \
        /home/taiphoon/Taiphoon_Agent/config/obstacles/closed_search_wall.yaml
    takeoff_and_wait 1
    sleep 1
    run_monitor 1 --timeout-s 20 --minimum-displacement-m 3 \
        --expect-failure &
    monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_FORWARD 20"
    wait "${monitor_pid}"
}

run_dual()
{
    echo "=== 雙機 moving peer 交叉路徑 ==="
    start_agents 2 explicit_empty
    takeoff_and_wait 2
    sleep 1
    run_monitor 2 --timeout-s 90 --minimum-displacement-m 8 \
        --minimum-distance-m 3.0 &
    local monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "YAW 34"
    publish_command /uav_2_agent/command "YAW -34"
    publish_command /uav_1_agent/command "MOVE_FORWARD 14.5"
    publish_command /uav_2_agent/command "MOVE_FORWARD 14.5"
    wait "${monitor_pid}"
}

run_east()
{
    echo "=== Scenario B：uav_1 無障礙 +East 30 m ==="
    start_agents 1 explicit_empty
    takeoff_and_wait 1 "${TAKEOFF_ALTITUDE_M}"
    run_monitor 1 --timeout-s 20 --metric-warmup-s 0 \
        --minimum-progress-m 29 --travel-axis east \
        --max-lateral-m 1.0 --max-path-length-m 32.0 \
        --max-state-z-deviation-m 0.5 --expected-control-points 10 \
        --require-initial-heading-north &
    local monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_RIGHT 30"
    wait "${monitor_pid}"

    echo "=== Scenario C：uav_1 East wall +East 30 m ==="
    start_agents 1 ground_truth \
        /home/taiphoon/Taiphoon_Agent/config/obstacles/east_wall.yaml
    takeoff_and_wait 1 "${TAKEOFF_ALTITUDE_M}"
    run_monitor 1 --timeout-s 20 --metric-warmup-s 0 \
        --minimum-progress-m 29 --travel-axis east \
        --minimum-lateral-m 3.0 --require-lateral-m 3.0 \
        --max-lateral-m 4.5 --max-path-length-m 35.0 \
        --minimum-obstacle-distance-m 1.0 \
        --max-state-z-deviation-m 0.5 --expected-control-points 10 \
        --require-initial-heading-north &
    monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_RIGHT 30"
    wait "${monitor_pid}"
}

run_four_move()
{
    echo "=== Scenario D：四機西向東排列，僅 uav_1 +East MOVE_RIGHT 30 m ==="
    start_agents 4 explicit_empty
    takeoff_and_wait 4 "${TAKEOFF_ALTITUDE_M}"
    run_monitor 4 --moving-agents 1 --timeout-s 120 \
        --metric-warmup-s 0 --minimum-progress-m 29 \
        --travel-axis east --minimum-lateral-m 2.5 \
        --minimum-distance-m 3.0 --max-state-z-deviation-m 0.5 \
        --expected-control-points 10 --require-initial-heading-north &
    local monitor_pid=$!
    sleep 1
    publish_command /uav_1_agent/command "MOVE_RIGHT 30"
    wait "${monitor_pid}"
}

wait_for_formation_converged()
{
    local count="$1"
    local deadline=$((SECONDS + 90))
    while (( SECONDS < deadline )); do
        local all_ready=1
        local agent_id
        for agent_id in $(seq 1 "${count}"); do
            local reason
            reason=$(docker exec -i uav_1_agent bash -lc \
                "source /opt/ros/humble/setup.bash && ros2 topic echo \
                 --once /uav_${agent_id}_agent/planning/status" 2>/dev/null || true)
            if ! grep -q "formation_converged; terminal hold active" <<<"${reason}"; then
                all_ready=0
                break
            fi
        done
        if [[ "${all_ready}" -eq 1 ]]; then
            # acceptance monitor 仍會驗證連續 2 秒；這裡只負責讓外層
            # command sequence 依實際 planner 狀態前進，不使用固定 20 秒。
            sleep 3
            return 0
        fi
        sleep 1
    done
    echo "錯誤：90 秒內未觀察到所有 formation terminal hold" >&2
    return 1
}

run_swarm()
{
    echo "=== Scenario A：四機完整環形編隊固定高度 ==="
    start_agents 4 explicit_empty
    takeoff_and_wait 4 "${FORMATION_ALTITUDE_M}"
    docker exec -i uav_1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && \
         source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
         source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
         python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/ego_swarm_acceptance_monitor.py \
         --mode 2 --agents 4 --timeout-s 90 --minimum-distance-m 3.0" &
    local monitor_pid=$!
    sleep 1
    # 4 台 Agent 各有 BasicController 與 planner 兩個訂閱者，另加驗收監測程式。
    publish_command /uav_all_agent/command "FORMATION_CIRCLE 10 10" 9
    wait_for_formation_converged 4
    publish_command /uav_all_agent/command "FORMATION_CANCEL" 9
    wait "${monitor_pid}"
}

case "${SCENARIO}" in
    single) run_single ;;
    east) run_east ;;
    dual) run_dual ;;
    four_move) run_four_move ;;
    swarm) run_swarm ;;
    all)
        run_single
        run_east
        run_dual
        run_four_move
        run_swarm
        ;;
esac

echo "[PASS] EGO 時空路徑規劃 SITL 驗證完成：${SCENARIO}"
echo "[PASS] EGO 時空路徑規劃 SITL 驗證完成：${SCENARIO}" >&3
