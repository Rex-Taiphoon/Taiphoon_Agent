#!/usr/bin/env bash
# @file runtime_entrypoint_test.sh
# @brief 在隔離開發容器驗證無飛控啟動、ROS2 狀態與必要程序失敗
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eo pipefail
test_dir="$(mktemp -d /tmp/taiphoon-runtime-test.XXXXXX)"
install_base="${TAIPHOON_TEST_INSTALL:-/tmp/taiphoon-workspace-install}"
source_base="${TAIPHOON_TEST_SOURCE:-/workspace}"
mkdir -p "${test_dir}/workspace"
ln -s "${install_base}" "${test_dir}/workspace/install"
ln -s "${source_base}/config" "${test_dir}/workspace/config"
export TAIPHOON_WS="${test_dir}/workspace"
export ROS_DOMAIN_ID=98
export UXRCE_PORT=9899
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash "${source_base}/docker/runtime-entrypoint.sh" > "${test_dir}/runtime.log" 2>&1 &
runtime_pid=$!
trap 'kill -TERM "${runtime_pid}" 2>/dev/null || true' EXIT
is_healthy=false
for attempt in $(seq 1 20); do
    if bash "${source_base}/docker/runtime-healthcheck.sh" 2>/dev/null; then
        is_healthy=true
        break
    fi
    if ! kill -0 "${runtime_pid}" 2>/dev/null; then
        cat "${test_dir}/runtime.log"
        exit 1
    fi
    sleep 1
done
if [ "${is_healthy}" != true ]; then
    cat "${test_dir}/runtime.log"
    exit 1
fi
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
source "${install_base}/setup.bash"
set +e
timeout 5 ros2 topic echo /uav_1_agent/state taiphoon_agent/msg/AgentState --once \
    --no-daemon --field px4_state_valid | tee "${test_dir}/px4-state.txt"
observation_result=${PIPESTATUS[0]}
set -e
if [ "${observation_result}" -eq 0 ]; then
    grep -Eq 'False|false' "${test_dir}/px4-state.txt"
elif [ "${observation_result}" -ne 124 ]; then
    exit 1
fi
# 既有節點可在無 odometry 時不發布 state；timeout 代表等待，不代表就緒。
# 只終止測試容器內由本測試建立的橋接，不觸及任何實機或其他服務。
kill -TERM "$(head -n 1 /tmp/taiphoon-agent.pids)"
set +e
wait "${runtime_pid}"
result=$?
set -e
if [ "${result}" -ne 1 ]; then
    cat "${test_dir}/runtime.log"
    exit 1
fi
trap - EXIT
echo "PASS: 無飛控程序健康、PX4 狀態無效、橋接失敗使服務退出"
