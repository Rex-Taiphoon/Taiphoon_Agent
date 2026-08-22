#!/usr/bin/env bash
# =============================================================================
# ego_route_planning_sitl_validation.sh
# EGO 式 FIXED_Z 時空路徑規劃 SITL 驗證：單機、雙機與四機。
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

usage()
{
    cat <<'EOF'
使用方式：
  bash AutoTest/scripts/ego_route_planning_sitl_validation.sh \
      --scenario single|dual|swarm|all [--skip-basic]

single：BasicController 回歸、無障礙直線、固定障礙繞行、封閉搜尋範圍。
dual：兩台以交叉 MOVE_POINT 驗證 moving peer 的時間避碰。
swarm：四台 `FORMATION_CIRCLE 10 10` 固定高度與既有 FORMATION_CIRCLE → CANCEL → MOVE 回歸。
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
    single|dual|swarm|all) ;;
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

if ! docker image inspect taiphoon-agent:dev >/dev/null 2>&1; then
    echo "錯誤：找不到 taiphoon-agent:dev，請由開發者自行建置 image。" >&2
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&2
    exit 2
fi
if ! docker image inspect px4io/px4-sitl:latest >/dev/null 2>&1; then
    echo "錯誤：找不到 px4io/px4-sitl:latest，請先 pull 或準備 SITL image。" >&2
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
    export FMU1_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export FMU2_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export FMU3_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export FMU4_OBSTACLE_CLOUD_MODE="${obstacle_mode}"
    export FMU1_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export FMU2_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export FMU3_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    export FMU4_OBSTACLE_CLOUD_CONFIG="${obstacle_config}"
    docker compose down >/dev/null 2>&1 || true
    if [[ "${count}" -eq 1 ]]; then
        docker compose up -d fmu1_agent
    elif [[ "${count}" -eq 2 ]]; then
        docker compose up -d fmu1_agent fmu2_agent
    else
        docker compose up -d
    fi
    AGENT_STARTED=1
    bash "${SCRIPT_DIR}/px4-sitl-launcher/launch_all.sh" \
        --count "${count}" --net "${NETWORK_NAME}" --rmw cyclonedds
    sleep 10
}

publish_command()
{
    local topic="$1"
    local command="$2"
    docker exec -i fmu1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && ros2 topic pub \
        --wait-matching-subscriptions 1 --once ${topic} \
        std_msgs/msg/String \"data: '${command}'\"" >/dev/null
}

run_monitor()
{
    local count="$1"
    shift
    docker exec -i fmu1_agent bash -lc \
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
    docker exec -i fmu1_agent bash -lc \
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
    publish_command /fmu1_agent/command "MOVE_FORWARD 12"
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
    publish_command /fmu1_agent/command "MOVE_FORWARD 16"
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
    publish_command /fmu1_agent/command "MOVE_FORWARD 20"
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
    publish_command /fmu1_agent/command "YAW 34"
    publish_command /fmu2_agent/command "YAW -34"
    publish_command /fmu1_agent/command "MOVE_FORWARD 14.5"
    publish_command /fmu2_agent/command "MOVE_FORWARD 14.5"
    wait "${monitor_pid}"
}

run_swarm()
{
    echo "=== 四機完整環形編隊固定高度與 Formation 回歸 ==="
    start_agents 4 explicit_empty
    # 先以 10 m 起飛，再建立完整的 10 m 半徑、10 m 高度環形編隊；
    # 編隊目標高度與起飛高度一致，避免 Formation 期間改變 Z。
    takeoff_and_wait 4 "${FORMATION_ALTITUDE_M}"
    publish_command /fmu_all_agent/command "FORMATION_CIRCLE 10 10"
    sleep 20
    publish_command /fmu_all_agent/command "FORMATION_CANCEL"
    sleep 3
    run_monitor 4 --timeout-s 90 --minimum-displacement-m 8 \
        --moving-agents 1 --minimum-distance-m 3.0 \
        --travel-axis east &
    local monitor_pid=$!
    sleep 1
    publish_command /fmu1_agent/command "MOVE_RIGHT 16"
    wait "${monitor_pid}"

    echo "=== FORMATION_CIRCLE → FORMATION_CANCEL → MOVE ==="
    start_agents 4 explicit_empty
    takeoff_and_wait 4 "${FORMATION_ALTITUDE_M}"
    sleep 1
    docker exec -i fmu1_agent bash -lc \
        "source /opt/ros/humble/setup.bash && \
         source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
         source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
         python3 /home/taiphoon/Taiphoon_Agent/AutoTest/scripts/ego_swarm_acceptance_monitor.py \
         --mode 2 --agents 4 \
         --timeout-s 150 --minimum-distance-m 3.0" &
    monitor_pid=$!
    sleep 1
    publish_command /fmu_all_agent/command "FORMATION_CIRCLE 10 10"
    sleep 20
    publish_command /fmu_all_agent/command "FORMATION_CANCEL"
    sleep 3
    # Formation_CANCEL 後已沒有 active FormationEpoch；既有架構只接受
    # local MOVE，因此沿用既有 local command topic，不改 runtime 行為。
    publish_command /fmu1_agent/command "MOVE_FORWARD 5"
    # 既有 acceptance monitor 需要先觀察第一個 MOVE 抵達，才接受下一個
    # MOVE_RIGHT；避免兩筆既有命令在同一個 callback window 互相覆蓋。
    sleep 10
    publish_command /fmu1_agent/command "MOVE_RIGHT 20"
    wait "${monitor_pid}"
}

case "${SCENARIO}" in
    single) run_single ;;
    dual) run_dual ;;
    swarm) run_swarm ;;
    all)
        run_single
        run_dual
        run_swarm
        ;;
esac

echo "[PASS] EGO 時空路徑規劃 SITL 驗證完成：${SCENARIO}"
