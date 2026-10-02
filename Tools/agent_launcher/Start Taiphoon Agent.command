#!/usr/bin/env bash
# @file Start Taiphoon Agent.command
# @brief macOS Finder 雙擊入口，交由原生工具完成設定與啟動
# @author Taiphoon Agent Team
# @date 2026-10-02
package_dir="$(cd "$(dirname "$0")" && pwd)"
"${package_dir}/taiphoon_agent" start
result=$?
if [ "${result}" -ne 0 ]; then
    echo "啟動未完成，請依上方訊息處理後重試。"
fi
read -r -p "按 Enter 關閉視窗。" _
exit "${result}"
