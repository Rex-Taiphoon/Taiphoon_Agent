#!/bin/sh
# 以 macOS host 的 C++17 與 Python 啟動離線規劃模擬器。
set -eu

TOOL_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "$TOOL_ROOT/../.." && pwd)
BUILD_ROOT=/tmp/taiphoon_planning_build

cmake -S "$TOOL_ROOT" -B "$BUILD_ROOT" \
  -DTAIPHOON_AGENT_SOURCE_DIR="$PROJECT_ROOT"
cmake --build "$BUILD_ROOT" --parallel

exec python3 "$TOOL_ROOT/server/server.py" \
  --engine "$BUILD_ROOT/planning_simulator" "$@"
