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
