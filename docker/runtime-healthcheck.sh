#!/usr/bin/env bash
# @file runtime-healthcheck.sh
# @brief 程序健康檢查，不代表飛控／定位／感測器已就緒
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eo pipefail
test -r /tmp/taiphoon-agent.pids
while read -r child; do
    kill -0 "${child}"
done < /tmp/taiphoon-agent.pids
pgrep -f '/lib/taiphoon_agent/basic_controller' >/dev/null
pgrep -f '/lib/taiphoon_agent/ego_swarm_planner' >/dev/null
# 實機宣告 explicit_empty 時，空地圖來源必須存在；否則規劃器會因地圖過期而停住。
if [ "${ENABLE_OBSTACLE_CLOUD_SIMULATOR:-false}" = "true" ]; then
    pgrep -f '/lib/taiphoon_agent/obstacle_cloud_simulator' >/dev/null
fi
