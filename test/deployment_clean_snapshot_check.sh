#!/usr/bin/env bash
# @file deployment_clean_snapshot_check.sh
# @brief 只驗證 HEAD 原始碼加部署變更，排除工作區既有未提交規劃器變更
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eo pipefail
snapshot="$(mktemp -d /tmp/taiphoon-release-review.XXXXXX)"
tar -xf /workspace/.scratch/deployment-base.tar -C "${snapshot}"
mkdir -p "${snapshot}/Tools/agent_launcher" "${snapshot}/.github/workflows"
cp -a /workspace/Tools/agent_launcher/. "${snapshot}/Tools/agent_launcher/"
cp -a /workspace/.github/workflows/. "${snapshot}/.github/workflows/"
cp /workspace/Dockerfile /workspace/README.md "${snapshot}/"
cp /workspace/docker/runtime-entrypoint.sh /workspace/docker/runtime-healthcheck.sh \
    /workspace/docker/sitl-entrypoint.sh "${snapshot}/docker/"
cp /workspace/test/agent_launcher_test.cpp /workspace/test/agent_launcher_integration_test.py \
    /workspace/test/deployment_package_test.py /workspace/test/runtime_entrypoint_test.sh "${snapshot}/test/"
cd "${snapshot}"
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
colcon --log-base "${snapshot}/log" build --executor sequential --packages-select taiphoon_agent \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${snapshot}/install/setup.bash"
colcon --log-base "${snapshot}/log" test --packages-select taiphoon_agent
colcon --log-base "${snapshot}/log" test-result --verbose
cmake -S Tools/agent_launcher -B "${snapshot}/native-build" -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Release -DTAIPHOON_RELEASE_VERSION=v0.0.0-review
cmake --build "${snapshot}/native-build" --parallel 2
ctest --test-dir "${snapshot}/native-build" --output-on-failure
TAIPHOON_TEST_SOURCE="${snapshot}" TAIPHOON_TEST_INSTALL="${snapshot}/install" \
    bash "${snapshot}/test/runtime_entrypoint_test.sh"
echo "PASS: 排除既有工作區修改的部署快照：${snapshot}"
