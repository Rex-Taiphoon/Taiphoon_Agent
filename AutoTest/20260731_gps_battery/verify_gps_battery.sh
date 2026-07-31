#!/bin/bash
# =============================================================================
# 驗證 AgentState 擴充欄位：GPS 座標 + 電池電壓/電流
# 前提：PX4 SITL + Taiphoon_Agent 容器已啟動
# =============================================================================
set -euo pipefail

AGENT_NS="${1:-fmu1_agent}"
TOPIC="/${AGENT_NS}/state"
TIMEOUT=10

echo "=== AgentState GPS/Battery 驗證 ==="
echo "Topic: ${TOPIC}"
echo "Timeout: ${TIMEOUT}s"
echo ""

echo "--- 等待 AgentState topic ---"
timeout ${TIMEOUT} ros2 topic echo --once "${TOPIC}" > /tmp/agent_state_verify.txt 2>&1 || {
    echo "FAIL: Cannot receive ${TOPIC}"
    exit 1
}

echo "--- 原始訊息 ---"
cat /tmp/agent_state_verify.txt

# 提取欄位值進行驗證
LAT=$(grep "position_global\[0\]" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')
LON=$(grep "position_global\[1\]" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')
ALT=$(grep "position_global\[2\]" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')
VOLT=$(grep "battery_voltage_v" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')
CURR=$(grep "battery_current_a" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')
REMAIN=$(grep "battery_remaining" /tmp/agent_state_verify.txt | head -1 | awk '{print $NF}')

echo ""
echo "--- 欄位驗證 ---"

# GPS 座標：應在合理範圍（台北 SITL home: 25.04, 121.51）
LAT_OK=$(python3 -c "print('PASS' if 22 < float('${LAT:-0}') < 28 else 'FAIL (out of range)')")
LON_OK=$(python3 -c "print('PASS' if 118 < float('${LON:-0}') < 124 else 'FAIL (out of range)')")
ALT_OK=$(python3 -c "print('PASS' if float('${ALT:-0}') >= 0 else 'FAIL')")

echo "  lat  = ${LAT}  ${LAT_OK}"
echo "  lon  = ${LON}  ${LON_OK}"
echo "  alt  = ${ALT}  ${ALT_OK}"

# 電池電壓：應 > 0
VOLT_OK=$(python3 -c "print('PASS' if float('${VOLT:-0}') > 0 else 'FAIL')")
echo "  volt = ${VOLT}  ${VOLT_OK}"

# 電池電流：合理範圍（SITL 通常 0~30A）
CURR_OK=$(python3 -c "print('PASS' if float('${CURR:-0}') >= 0 else 'FAIL')")
echo "  curr = ${CURR}  ${CURR_OK}"

# 電量比例：0.0 ~ 1.0
REMAIN_OK=$(python3 -c "print('PASS' if 0 <= float('${REMAIN:-0}') <= 1 else 'FAIL')")
echo "  rem  = ${REMAIN}  ${REMAIN_OK}"

echo ""
echo "=== 驗證完成 ==="
