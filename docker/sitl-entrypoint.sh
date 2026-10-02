#!/bin/sh
# @file sitl-entrypoint.sh
# @brief 可攜 PX4 SIH 啟動；遵循既有 namespace 與 8m 起始間距
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eu
export PX4_SIM_MODEL=sihsim_quadx
export PX4_SIMULATOR=sihsim
export PX4_NET_INTERFACE=eth0
export PX4_PARAM_COM_RC_IN_MODE=4
export PX4_PARAM_COM_ARM_MIS_REQ=0
export PX4_PARAM_MAV_0_BROADCAST=1
export PX4_PARAM_UXRCE_DDS_NS_IDX="${PX4_SYS_ID}"
export PX4_UXRCE_DDS_NS="uav_${PX4_SYS_ID}"
export PX4_UXRCE_DDS_PORT="${PX4_AGENT_PORT}"
export PX4_HOME_LAT=25.03996592258936
PX4_HOME_LON=$(awk -v instance="${PX4_INSTANCE}" \
    'BEGIN { printf "%.12f", 121.51195163336611 + instance * 8 / (111320 * cos(25.03996592258936 * 3.141592653589793 / 180)) }')
export PX4_HOME_LON
agent_ip=$(getent ahostsv4 "${PX4_AGENT_HOST}" | awk '/STREAM/ {print $1; exit}')
if [ -z "${agent_ip}" ]; then
    echo "[SITL] 無法解析 Agent 位址" >&2
    exit 1
fi
sed -i "s|uxrce_dds_client start -t udp|uxrce_dds_client start -t udp -h ${agent_ip} -p ${PX4_AGENT_PORT}|" \
    /opt/px4/etc/init.d-posix/rcS
gcs_ip=$(getent ahostsv4 host.docker.internal | awk '/STREAM/ {print $1; exit}')
if [ -n "${gcs_ip}" ]; then
    sed -i "s/mavlink start -x -u/mavlink start -x -t ${gcs_ip} -u/g" \
        /opt/px4/etc/init.d-posix/px4-rc.mavlink
fi
exec /opt/px4/bin/px4 -d -i "${PX4_INSTANCE}"
