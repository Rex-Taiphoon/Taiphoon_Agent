#!/usr/bin/env bash
# @file setup_time_sync.sh
# @brief 在樹莓派主機設定 chrony，讓多台 Agent 以 uav_1 的主機為唯一時間基準
# @author Taiphoon Agent Team
# @date 2026-10-20
#
# 用法：
#   sudo bash setup_time_sync.sh master                 # uav_1：時間主機
#   sudo bash setup_time_sync.sh client <主機IPv4>       # 其他機：向時間主機對時
#   bash setup_time_sync.sh render master               # 只輸出設定，不修改系統
#   bash setup_time_sync.sh render client <主機IPv4>     # 只輸出設定，不修改系統
#   bash setup_time_sync.sh status                      # 顯示目前對時狀態
#
# 在主機作業系統執行，不在 Agent 容器內執行；容器與主機共用核心時鐘。
# 設計與失效情境見 docs/time-sync.md。
set -euo pipefail

readonly CHRONY_CONF="/etc/chrony/chrony.conf"
readonly CHRONY_BACKUP="${CHRONY_CONF}.taiphoon-backup"
RENDERED_FILE=""

cleanup() {
    if [ -n "${RENDERED_FILE}" ]; then
        rm -f "${RENDERED_FILE}"
    fi
}

trap cleanup EXIT

usage() {
    sed -n '7,12p' "$0" | sed 's/^# \{0,1\}//'
}

fail() {
    echo "[time_sync] 錯誤：$*" >&2
    exit 1
}

is_ipv4() {
    local address="$1"
    local octet
    [[ "${address}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || return 1
    IFS='.' read -r -a octets <<< "${address}"
    for octet in "${octets[@]}"; do
        # 以十進位比較，避免 08、09 被當成八進位。
        (( 10#${octet} <= 255 )) || return 1
    done
    return 0
}

# 兩種角色共用：只在開機前三次更新允許跳時，之後以每秒 0.5 ms 以內微調，
# 避免飛行中系統時間突然跳動。
render_common() {
    cat <<'EOF'
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony
makestep 0.1 3
maxslewrate 500
EOF
}

render_master() {
    cat <<'EOF'
# Taiphoon 時間主機設定（由 Tools/time_sync/setup_time_sync.sh 產生）
# 不引用公用 NTP：整群只以本機時鐘為唯一基準，外場有無網路行為一致。
local stratum 8
allow 10.0.0.0/8
allow 172.16.0.0/12
allow 192.168.0.0/16
EOF
    render_common
}

render_client() {
    local master_ip="$1"
    cat <<EOF
# Taiphoon 時間客戶端設定（由 Tools/time_sync/setup_time_sync.sh 產生）
# 只向時間主機對時；不提供時間給其他主機。
server ${master_ip} iburst minpoll 2 maxpoll 4
EOF
    render_common
}

render_role() {
    local role="${1:-}"
    case "${role}" in
        master)
            render_master
            ;;
        client)
            local master_ip="${2:-}"
            is_ipv4 "${master_ip}" || fail "client 需要時間主機的 IPv4，例如 192.168.1.101"
            render_client "${master_ip}"
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
}

ensure_chrony_installed() {
    if command -v chronyd >/dev/null 2>&1; then
        return 0
    fi
    echo "[time_sync] 安裝 chrony（需要網路）..."
    if ! apt-get install -y chrony; then
        fail "無法安裝 chrony；請先連上網路執行 sudo apt-get update 後重試"
    fi
}

apply_role() {
    [ "$(id -u)" -eq 0 ] || fail "需要 root 權限；請使用 sudo 執行"
    command -v systemctl >/dev/null 2>&1 || fail "找不到 systemctl；本腳本只支援 Ubuntu 主機"

    RENDERED_FILE="$(mktemp)"
    render_role "$@" > "${RENDERED_FILE}"

    ensure_chrony_installed

    # 先驗證語法再取代原設定，避免寫入無效設定後 chrony 無法啟動。
    if ! chronyd -p -f "${RENDERED_FILE}" >/dev/null; then
        fail "產生的 chrony 設定無法通過語法檢查，未修改系統"
    fi

    mkdir -p "$(dirname "${CHRONY_CONF}")"
    if [ -f "${CHRONY_CONF}" ] && [ ! -f "${CHRONY_BACKUP}" ]; then
        cp -p "${CHRONY_CONF}" "${CHRONY_BACKUP}"
        echo "[time_sync] 原設定已備份：${CHRONY_BACKUP}"
    fi
    install -m 0644 "${RENDERED_FILE}" "${CHRONY_CONF}"

    # 兩個對時服務同時執行會互相改寫系統時鐘。
    if systemctl list-unit-files systemd-timesyncd.service >/dev/null 2>&1; then
        systemctl disable --now systemd-timesyncd.service >/dev/null 2>&1 || true
    fi
    systemctl enable chrony.service >/dev/null
    systemctl restart chrony.service

    echo "[time_sync] 已套用 $1 設定：${CHRONY_CONF}"
    if [ "$1" = "master" ]; then
        echo "[time_sync] 本機為時間主機；請確認其他機可連到本機 UDP 123。"
    else
        echo "[time_sync] 等待對時完成後，執行 taiphoon_agent timesync 確認。"
    fi
}

show_status() {
    command -v chronyc >/dev/null 2>&1 || fail "尚未安裝 chrony"
    chronyc tracking
    echo
    chronyc sources -v
}

main() {
    local command="${1:-}"
    case "${command}" in
        master|client)
            apply_role "$@"
            ;;
        render)
            shift
            render_role "$@"
            ;;
        status)
            show_status
            ;;
        -h|--help|help)
            usage
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
}

main "$@"
