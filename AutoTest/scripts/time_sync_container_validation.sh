#!/usr/bin/env bash
# @file time_sync_container_validation.sh
# @brief 以兩個一次性容器驗證 chrony 主機／客戶端設定與 taiphoon_agent timesync 判定
# @author Taiphoon Agent Team
# @date 2026-10-20
#
# 用法（於專案根目錄）：
#   bash AutoTest/scripts/time_sync_container_validation.sh <Linux 版 taiphoon_agent 路徑>
#
# 驗證項目：
#   1. render master 設定可通過 chronyd 語法檢查，時間主機以本機基準通過 timesync。
#   2. client 套用流程（以 systemctl 替身）會備份原設定、只備份一次、拒絕非法 IPv4。
#   3. 客戶端向主機對時後 timesync 通過，來源為主機 IP。
#   4. 以 libfaketime 讓主機時間快 5 秒，客戶端 timesync 判定未就緒。
#
# chronyd 一律以 -x 執行，不修改容器或 Docker 主機的系統時鐘；因此第 4 項會保留
# 5 秒誤差（實機 chronyd 會在開機前三次更新內直接跳時校正）。容器以 --rm 啟動，
# 結束時自動移除，不刪除任何專案檔案。需要網路安裝 chrony 與 faketime。
set -euo pipefail

# Git Bash 下避免把容器內路徑改寫成 Windows 路徑。
export MSYS_NO_PATHCONV=1

readonly LAUNCHER="${1:?需要 Linux 版 taiphoon_agent 路徑}"
readonly IMAGE="${TIME_SYNC_TEST_IMAGE:-ubuntu:24.04}"
readonly NETWORK="${TIME_SYNC_TEST_NETWORK:-Taiphoon_Net}"
readonly MASTER="taiphoon_time_sync_master"
readonly CLIENT="taiphoon_time_sync_client"
readonly FAKETIME_LIBRARY="/usr/lib/x86_64-linux-gnu/faketime/libfaketime.so.1"

cd "$(dirname "$0")/../.."

cleanup() {
    docker stop "${MASTER}" "${CLIENT}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

fail() {
    echo "[FAIL] $*" >&2
    exit 1
}

pass() {
    echo "[PASS] $*"
}

start_container() {
    local name="$1"
    docker run -d --rm --name "${name}" --network "${NETWORK}" "${IMAGE}" sleep infinity >/dev/null
    docker exec "${name}" bash -c "apt-get update -qq >/dev/null && \
        DEBIAN_FRONTEND=noninteractive apt-get install -y -qq chrony faketime >/dev/null 2>&1"
    docker exec "${name}" mkdir -p /opt/time_sync
    docker cp Tools/time_sync/setup_time_sync.sh "${name}:/opt/time_sync/"
    docker cp "${LAUNCHER}" "${name}:/opt/time_sync/taiphoon_agent"
}

timesync() {
    docker exec "$1" bash -c "TAIPHOON_AGENT_HOME=/tmp/taiphoon-config /opt/time_sync/taiphoon_agent timesync"
}

start_container "${MASTER}"
start_container "${CLIENT}"
master_ip="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${MASTER}")"
echo "chrony：$(docker exec "${MASTER}" chronyd --version)"
echo "時間主機 IPv4：${master_ip}"

# 1. 時間主機
docker exec "${MASTER}" bash -c "bash /opt/time_sync/setup_time_sync.sh render master > /etc/chrony/chrony.conf \
    && chronyd -p -f /etc/chrony/chrony.conf >/dev/null && chronyd -x -f /etc/chrony/chrony.conf"
sleep 2
output="$(timesync "${MASTER}")" || fail "時間主機 timesync 未通過：${output}"
[[ "${output}" == *"本機基準"* ]] || fail "時間主機未使用本機基準：${output}"
pass "時間主機：${output}"

# 2. 客戶端套用流程；systemctl 替身只記錄呼叫，不需要 systemd。
docker exec "${CLIENT}" bash -c "printf '#!/bin/sh\necho \"\$*\" >> /tmp/systemctl.log\n' > /usr/local/bin/systemctl \
    && chmod +x /usr/local/bin/systemctl && cp /etc/chrony/chrony.conf /tmp/original.conf"
docker exec "${CLIENT}" bash /opt/time_sync/setup_time_sync.sh client "${master_ip}" >/dev/null \
    || fail "客戶端套用失敗"
docker exec "${CLIENT}" bash /opt/time_sync/setup_time_sync.sh client "${master_ip}" >/dev/null \
    || fail "客戶端重複套用失敗"
docker exec "${CLIENT}" cmp -s /tmp/original.conf /etc/chrony/chrony.conf.taiphoon-backup \
    || fail "原設定備份不正確或被覆寫"
docker exec "${CLIENT}" grep -q "^server ${master_ip} iburst" /etc/chrony/chrony.conf \
    || fail "客戶端設定缺少時間主機"
docker exec "${CLIENT}" grep -q "restart chrony.service" /tmp/systemctl.log \
    || fail "套用後未重新啟動 chrony"
if docker exec "${CLIENT}" bash /opt/time_sync/setup_time_sync.sh client 300.1.1.1 >/dev/null 2>&1; then
    fail "非法 IPv4 被接受"
fi
pass "客戶端套用：備份一次、設定正確、重新啟動 chrony、拒絕非法 IPv4"

# 3. 客戶端對時
docker exec "${CLIENT}" chronyd -x -f /etc/chrony/chrony.conf
is_synced=false
for _ in $(seq 1 30); do
    if output="$(timesync "${CLIENT}")"; then
        is_synced=true
        break
    fi
    sleep 1
done
[ "${is_synced}" = true ] || fail "客戶端 30 秒內未對時：${output}"
[[ "${output}" == *"${master_ip}"* ]] || fail "客戶端來源不是時間主機：${output}"
pass "客戶端：${output}"

# 4. 時間主機快 5 秒：客戶端必須判定未就緒。
docker exec "${MASTER}" bash -c "pkill chronyd; sleep 1; FAKETIME='+5s' LD_PRELOAD=${FAKETIME_LIBRARY} \
    chronyd -x -f /etc/chrony/chrony.conf"
docker exec "${CLIENT}" bash -c "pkill chronyd; sleep 1; chronyd -x -f /etc/chrony/chrony.conf"
is_detected=false
for _ in $(seq 1 30); do
    if ! output="$(timesync "${CLIENT}")" && [[ "${output}" == *"超過"* ]]; then
        is_detected=true
        break
    fi
    sleep 1
done
[ "${is_detected}" = true ] || fail "主機時間偏移 5 秒未被偵測：${output}"
pass "偏差偵測：${output}"

echo "PASS: chrony 主機／客戶端設定、套用流程、timesync 判定與偏差偵測"
