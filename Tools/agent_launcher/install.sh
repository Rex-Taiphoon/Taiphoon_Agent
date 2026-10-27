#!/usr/bin/env bash
# @file install.sh
# @brief 安裝 Taiphoon Agent，或替換本機既有的 Agent；保留設定，新版啟動失敗時切回上一版
# @author Taiphoon Agent Team
# @date 2026-10-27
#
# 用法（在解壓縮後的平台包目錄執行，不要加 sudo）：
#   bash install.sh                       安裝或替換為本套件版本，執行前確認
#   bash install.sh --yes                 不詢問；沿用既有設定與開機自動啟動狀態
#   bash install.sh --yes --agent-id 2    首次安裝實機：建立無人機 2 的 deploy 設定
#   bash install.sh --autostart           同時設定開機自動啟動（Linux systemd）
#   bash install.sh --no-autostart        取消開機自動啟動
#   bash install.sh --no-start            只安裝，不啟動 Agent
#   bash install.sh --rollback            切回上一個安裝版本
#
# 需要管理 systemd 時會自行呼叫 sudo。不刪除設定、Docker 映像或 volume；只保留目前與上一個安裝版本。
# 設計與失效情境見 docs/deployment-installer.md。
set -Eeuo pipefail

readonly SERVICE_NAME="taiphoon-agent.service"
readonly PROJECT_NAME="taiphoon_agent"
readonly STOP_TIMEOUT_SECONDS=20
readonly MAX_AGENT_ID=255
# 安裝位置除套件本身外再保留 1 GiB，避免 SD 卡寫滿造成安裝到一半。
readonly SPACE_MARGIN_KB=1048576
readonly VERSION_PATTERN='^v[0-9]+\.[0-9]+\.[0-9]+([.-][A-Za-z0-9.-]+)?$'
readonly PACKAGE_FILES="taiphoon_agent release.manifest sitl-entrypoint.sh README.md install.sh setup_time_sync.sh"
readonly MACOS_ENTRY="Start Taiphoon Agent.command"
# 舊版 install.sh 直接複製到安裝根目錄的檔案。
readonly FLAT_FILES="taiphoon_agent release.manifest sitl-entrypoint.sh README.md images"
readonly INSTALL_ROOT="${HOME}/.local/share/taiphoon_agent"
readonly RELEASES_DIR="${INSTALL_ROOT}/releases"
readonly BIN_DIR="${HOME}/.local/bin"
# 只供測試隔離 systemd 目錄；一般安裝不需要設定。
readonly UNIT_DIR="${TAIPHOON_SYSTEMD_UNIT_DIR:-/etc/systemd/system}"
readonly UNIT_FILE="${UNIT_DIR}/${SERVICE_NAME}"

ACTION="install"
SHOULD_ASSUME_YES=0
AGENT_ID=""
AUTOSTART_REQUEST="keep"
SHOULD_START=1
PACKAGE_DIR=""
CONFIG_HOME=""
VERSION=""
PACKAGE_ARCH=""
HAS_SYSTEMD=0
HAS_DOCKER=0
DOCKER_ARCH=""
OLD_DIR=""
FLAT_DIR=""
FLAT_VERSION=""
PREVIOUS_DIR=""
IS_UNIT_PRESENT=0
IS_UNIT_ENABLED=0
IS_UNIT_ACTIVE=0
UNIT_HOME=""
RUNNING_HOME=""
RUNNING_FILE=""
LEGACY_CONTAINERS=""
SOURCE_HOME=""
EFFECTIVE_SETTINGS=""
SETTINGS_ACTION=""
MODE=""
IMAGE_KIND="runtime"
SHOULD_ENABLE_AUTOSTART=0
WAS_RUNNING=0
START_NOTE=""
PRUNE_DIRS=""
STAGING_DIR=""
UNIT_DRAFT=""
IS_LOCK_HELD=0
IS_VERSION_CHANGED=0
IS_STARTED=0
PATH_NOTE=""

usage() {
    sed -n '7,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

log() {
    echo "[install] $*"
}

warn() {
    echo "[install] 警告：$*" >&2
}

fail() {
    echo "[install] 錯誤：$*" >&2
    exit 1
}

usage_error() {
    echo "[install] 錯誤：$*" >&2
    usage >&2
    exit 2
}

on_error() {
    echo "[install] 錯誤：install.sh 第 $1 行的指令失敗；請依上方訊息處理" >&2
}

cleanup() {
    if [ -n "${STAGING_DIR}" ] && [ -d "${STAGING_DIR}" ]; then
        rm -rf "${STAGING_DIR}"
    fi
    if [ -n "${UNIT_DRAFT}" ]; then
        rm -f "${UNIT_DRAFT}"
    fi
    if [ "${IS_LOCK_HELD}" -eq 1 ]; then
        rmdir "${INSTALL_ROOT}/.install.lock" 2>/dev/null || true
    fi
}

trap 'on_error "${LINENO}"' ERR
trap cleanup EXIT

# 不存在或無法進入時輸出空字串，讓呼叫端用字串比較判斷是否為同一目錄。
physical_dir() {
    (cd -P "$1" 2>/dev/null && pwd -P) || true
}

absolute_path() {
    case "$1" in
        /*) echo "$1" ;;
        *) echo "$(pwd -P)/$1" ;;
    esac
}

is_interactive() {
    [ -t 0 ]
}

# 讀取 manifest／settings 的 key=value；以 awk 讀完整個檔案，避免 pipefail 下的 SIGPIPE。
key_value() {
    awk -v key="$2" '
        {
            name = $0
            sub(/=.*/, "", name)
            gsub(/[ \t]/, "", name)
        }
        name == key && index($0, "=") > 0 {
            value = substr($0, index($0, "=") + 1)
            sub(/^[ \t]+/, "", value)
            sub(/[ \t\r]+$/, "", value)
            print value
            exit
        }' "$1"
}

settings_value() {
    local value
    value="$(key_value "$1" "$2")"
    echo "${value:-$3}"
}

file_sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | awk '{ print $1 }'
    else
        shasum -a 256 "$1" | awk '{ print $1 }'
    fi
}

run_root() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

ask_yes_no() {
    local answer=""
    local hint="[Y/n]"
    if [ "$2" != "y" ]; then
        hint="[y/N]"
    fi
    read -r -p "$1 ${hint} " answer || answer=""
    case "${answer}" in
        "") [ "$2" = "y" ] ;;
        y|Y|yes|YES|Yes) return 0 ;;
        *) return 1 ;;
    esac
}

confirm_or_stop() {
    if [ "${SHOULD_ASSUME_YES}" -eq 1 ]; then
        return 0
    fi
    if ! is_interactive; then
        fail "非互動執行時請加 --yes 確認上述計畫；未變更任何安裝"
    fi
    if ! ask_yes_no "$1" "y"; then
        fail "已取消；未變更任何安裝"
    fi
}

parse_arguments() {
    while [ "$#" -gt 0 ]; do
        case "$1" in
            -y|--yes) SHOULD_ASSUME_YES=1 ;;
            --agent-id)
                if [ "$#" -lt 2 ]; then
                    usage_error "--agent-id 需要無人機編號"
                fi
                AGENT_ID="$2"
                shift
                ;;
            --agent-id=*) AGENT_ID="${1#*=}" ;;
            --autostart) AUTOSTART_REQUEST="on" ;;
            --no-autostart) AUTOSTART_REQUEST="off" ;;
            --no-start) SHOULD_START=0 ;;
            --rollback) ACTION="rollback" ;;
            -h|--help|help)
                usage
                exit 0
                ;;
            *) usage_error "未知選項：$1" ;;
        esac
        shift
    done
    if [ -n "${AGENT_ID}" ]; then
        if [[ ! "${AGENT_ID}" =~ ^[1-9][0-9]{0,2}$ ]] || [ "${AGENT_ID}" -gt "${MAX_AGENT_ID}" ]; then
            usage_error "--agent-id 必須是 1～${MAX_AGENT_ID}，且與飛控 system ID 一致"
        fi
    fi
    if [ "${ACTION}" = "rollback" ]; then
        if [ -n "${AGENT_ID}" ] || [ "${AUTOSTART_REQUEST}" != "keep" ] || [ "${SHOULD_START}" -eq 0 ]; then
            usage_error "--rollback 只能搭配 --yes"
        fi
    fi
}

require_regular_user() {
    if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
        fail "請不要用 sudo 執行；以要執行 Agent 的使用者執行 bash install.sh，需要時會自行要求 sudo"
    fi
}

detect_platform() {
    case "$(uname -s)" in
        Linux)
            # 指定測試目錄時以替身代表 systemd；實機以 /run/systemd/system 判斷是否由 systemd 開機。
            if command -v systemctl >/dev/null 2>&1 \
                && { [ -d /run/systemd/system ] || [ -n "${TAIPHOON_SYSTEMD_UNIT_DIR:-}" ]; }; then
                HAS_SYSTEMD=1
            fi
            ;;
        Darwin) ;;
        *) fail "install.sh 只支援 Linux 與 macOS；Windows 請使用 install.ps1" ;;
    esac
}

package_architecture() {
    local architecture
    for architecture in amd64 arm64; do
        if [ -f "$1/images/taiphoon-agent-runtime-linux-${architecture}.tar.gz" ]; then
            echo "${architecture}"
            return 0
        fi
    done
}

# 只讀檢查：套件完整、版本一致、啟動工具可在本機 CPU 執行。
validate_package() {
    local name reported
    for name in taiphoon_agent release.manifest sitl-entrypoint.sh; do
        if [ ! -f "${PACKAGE_DIR}/${name}" ]; then
            fail "缺少 ${name}；請在解壓縮後的完整平台包目錄執行 install.sh"
        fi
    done
    VERSION="$(key_value "${PACKAGE_DIR}/release.manifest" version)"
    if [[ ! "${VERSION}" =~ ${VERSION_PATTERN} ]]; then
        fail "release.manifest 的版本無效；請重新下載完整平台包"
    fi
    PACKAGE_ARCH="$(package_architecture "${PACKAGE_DIR}")"
    if [ -z "${PACKAGE_ARCH}" ]; then
        fail "套件缺少 images/ 內的 Agent 映像；請重新下載完整平台包"
    fi
    if [ ! -x "${PACKAGE_DIR}/taiphoon_agent" ]; then
        chmod +x "${PACKAGE_DIR}/taiphoon_agent"
    fi
    if ! reported="$("${PACKAGE_DIR}/taiphoon_agent" --version 2>/dev/null)"; then
        fail "套件內的 taiphoon_agent 無法在這台主機執行；請下載符合 CPU 的平台包（樹莓派選 linux-arm64）"
    fi
    if [ "${reported}" != "${VERSION}" ]; then
        fail "taiphoon_agent 版本 ${reported} 與 release.manifest ${VERSION} 不符；請重新下載完整平台包"
    fi
}

check_docker() {
    local architecture
    if ! command -v docker >/dev/null 2>&1; then
        warn "找不到 docker；只安裝檔案，不載入映像也不啟動。安裝 Docker 後執行 taiphoon_agent start"
        return 0
    fi
    if ! architecture="$(docker info --format '{{.Architecture}}' 2>/dev/null)"; then
        fail "無法連線 Docker Engine，無法確認既有 Agent 是否執行；請啟動 Docker 並確認目前使用者可存取後重試"
    fi
    case "${architecture}" in
        x86_64|amd64) DOCKER_ARCH="amd64" ;;
        aarch64|arm64) DOCKER_ARCH="arm64" ;;
        *) fail "不支援的 Docker 架構：${architecture}" ;;
    esac
    if [ -n "${PACKAGE_ARCH}" ] && [ "${DOCKER_ARCH}" != "${PACKAGE_ARCH}" ]; then
        fail "此套件的 Agent 映像為 ${PACKAGE_ARCH}，Docker 為 ${DOCKER_ARCH}；請下載對應架構的平台包"
    fi
    if ! docker compose version >/dev/null 2>&1; then
        fail "Docker Compose 不可用；Ubuntu 請安裝 docker-compose-plugin，Windows／macOS 請檢查 Docker Desktop"
    fi
    HAS_DOCKER=1
}

detect_installed_release() {
    if [ -L "${INSTALL_ROOT}/current" ] && [ -f "${INSTALL_ROOT}/current/release.manifest" ]; then
        OLD_DIR="$(physical_dir "${INSTALL_ROOT}/current")"
    fi
    # 舊版 install.sh 每次都改寫 wrapper 指向平鋪檔案，存在時它就是使用者實際執行的版本。
    if [ -f "${INSTALL_ROOT}/taiphoon_agent" ] && [ -f "${INSTALL_ROOT}/release.manifest" ]; then
        FLAT_DIR="$(physical_dir "${INSTALL_ROOT}")"
        FLAT_VERSION="$(key_value "${INSTALL_ROOT}/release.manifest" version)"
        if [[ ! "${FLAT_VERSION}" =~ ${VERSION_PATTERN} ]]; then
            FLAT_VERSION="legacy-install"
        fi
        OLD_DIR="${FLAT_DIR}"
    fi
    if [ -L "${INSTALL_ROOT}/previous" ] && [ -f "${INSTALL_ROOT}/previous/release.manifest" ]; then
        PREVIOUS_DIR="$(physical_dir "${INSTALL_ROOT}/previous")"
    fi
}

unit_value() {
    awk -v key="$1=" 'index($0, key) == 1 { print substr($0, length(key) + 1); exit }' "${UNIT_FILE}"
}

# 支援 Environment=K=V、Environment="K=V" 與同一行多個變數。
unit_environment() {
    awk -v key="$1=" '
        index($0, "Environment=") == 1 && index($0, key) > 0 {
            start = index($0, key)
            value = substr($0, start + length(key))
            if (substr($0, start - 1, 1) == "\"") {
                sub(/".*/, "", value)
            } else {
                sub(/[ \t"].*/, "", value)
            }
            print value
            exit
        }' "${UNIT_FILE}"
}

detect_unit() {
    local program unit_user
    if [ "${HAS_SYSTEMD}" -ne 1 ] || [ ! -f "${UNIT_FILE}" ]; then
        return 0
    fi
    IS_UNIT_PRESENT=1
    if systemctl is-enabled --quiet "${SERVICE_NAME}" 2>/dev/null; then
        IS_UNIT_ENABLED=1
    fi
    if systemctl is-active --quiet "${SERVICE_NAME}" 2>/dev/null; then
        IS_UNIT_ACTIVE=1
    fi
    unit_user="$(unit_value User)"
    if [ "${unit_user}" != "$(id -un)" ]; then
        fail "${SERVICE_NAME} 以 ${unit_user:-root} 身分執行；請改用該使用者執行 install.sh"
    fi
    UNIT_HOME="$(unit_environment TAIPHOON_AGENT_HOME)"
    program="$(unit_value ExecStart)"
    program="${program#[-@:+!]}"
    case "${program}" in
        \"*)
            program="${program#\"}"
            program="${program%%\"*}"
            ;;
        *) program="${program%% *}" ;;
    esac
    # 手動部署的服務直接指向某個解壓縮的套件；開機實際執行的是它。
    if [ -n "${program}" ] && [ -f "$(dirname "${program}")/release.manifest" ]; then
        OLD_DIR="$(physical_dir "$(dirname "${program}")")"
    fi
}

project_container_ids() {
    docker ps --quiet --filter "label=com.docker.compose.project=${PROJECT_NAME}"
}

detect_running_project() {
    local container_id config_files
    if [ "${HAS_DOCKER}" -ne 1 ]; then
        return 0
    fi
    container_id="$(project_container_ids | awk 'NR == 1')"
    if [ -z "${container_id}" ]; then
        return 0
    fi
    config_files="$(docker inspect --format \
        '{{index .Config.Labels "com.docker.compose.project.config_files"}}' "${container_id}")"
    RUNNING_FILE="${config_files%%,*}"
    # 原始碼目錄的 docker compose 也可能用相同專案名稱；只有含 settings.ini 的啟動工具設定目錄由這裡停止。
    case "${RUNNING_FILE}" in
        /*)
            if [ -f "$(dirname "${RUNNING_FILE}")/settings.ini" ]; then
                RUNNING_HOME="$(dirname "${RUNNING_FILE}")"
            fi
            ;;
    esac
}

detect_existing() {
    detect_installed_release
    detect_unit
    detect_running_project
}

is_simulation_mode() {
    [ "${MODE}" = "sim" ] || { [ "${MODE}" = "dev" ] && [ "$(settings_value "$1" scenario real)" = "sim" ]; }
}

# 只處理會讓啟動工具拒絕啟動的舊入口容器（同一個無人機編號），其他容器不碰。
detect_blocking_legacy() {
    local settings_file="$1" ids id count name names
    if [ "${HAS_DOCKER}" -ne 1 ]; then
        return 0
    fi
    if is_simulation_mode "${settings_file}"; then
        count="$(settings_value "${settings_file}" count 1)"
        ids=""
        id=1
        while [ "${id}" -le "${count}" ]; do
            ids="${ids} ${id}"
            id=$((id + 1))
        done
    else
        ids="$(settings_value "${settings_file}" agent_id 1)"
    fi
    names="$(docker ps --format '{{.Names}}')"
    for id in ${ids}; do
        for name in "uav_${id}_agent" "fmu${id}_agent"; do
            if printf '%s\n' "${names}" | grep -x "${name}" >/dev/null; then
                LEGACY_CONTAINERS="${LEGACY_CONTAINERS} ${name}"
            fi
        done
    done
    LEGACY_CONTAINERS="${LEGACY_CONTAINERS# }"
}

# 使用中的設定優先：執行中的 Compose 設定目錄，其次是 systemd 指定的 TAIPHOON_AGENT_HOME。
plan_settings() {
    local target_file="${CONFIG_HOME}/settings.ini"
    local source_file="${target_file}"
    SOURCE_HOME="${CONFIG_HOME}"
    if [ -n "${RUNNING_HOME}" ]; then
        SOURCE_HOME="${RUNNING_HOME}"
    elif [ -n "${UNIT_HOME}" ]; then
        SOURCE_HOME="${UNIT_HOME}"
    fi
    SETTINGS_ACTION="keep"
    if [ -f "${SOURCE_HOME}/settings.ini" ] \
        && [ "$(physical_dir "${SOURCE_HOME}")" != "$(physical_dir "${CONFIG_HOME}")" ]; then
        source_file="${SOURCE_HOME}/settings.ini"
        if [ ! -f "${target_file}" ]; then
            SETTINGS_ACTION="copy"
        elif ! cmp -s "${source_file}" "${target_file}"; then
            SETTINGS_ACTION="replace"
        fi
    fi
    if [ -f "${source_file}" ]; then
        EFFECTIVE_SETTINGS="${source_file}"
        MODE="$(settings_value "${source_file}" mode deploy)"
        if [ -n "${AGENT_ID}" ] && { [ "${MODE}" != "deploy" ] \
            || [ "$(settings_value "${source_file}" agent_id 1)" != "${AGENT_ID}" ]; }; then
            fail "既有設定為 ${MODE} 模式、無人機 $(settings_value "${source_file}" agent_id 1)；--agent-id 只用於首次安裝，" \
                "請在安裝後執行 taiphoon_agent stop、taiphoon_agent config 修改"
        fi
        detect_blocking_legacy "${source_file}"
    elif [ -n "${AGENT_ID}" ]; then
        SETTINGS_ACTION="init-agent"
        MODE="deploy"
    elif [ "${SHOULD_ASSUME_YES}" -eq 0 ] && is_interactive; then
        SETTINGS_ACTION="init-interactive"
    else
        SETTINGS_ACTION="missing"
    fi
    if [ "${MODE}" = "dev" ]; then
        IMAGE_KIND="dev"
    fi
}

plan_autostart() {
    case "${AUTOSTART_REQUEST}" in
        on)
            if [ "${HAS_SYSTEMD}" -ne 1 ]; then
                fail "開機自動啟動需要以 systemd 開機的 Linux"
            fi
            if [ "${SETTINGS_ACTION}" = "missing" ]; then
                fail "開機自動啟動需要先有設定；首次安裝請加 --agent-id N，或在互動終端執行"
            fi
            SHOULD_ENABLE_AUTOSTART=1
            ;;
        off) SHOULD_ENABLE_AUTOSTART=0 ;;
        *)
            SHOULD_ENABLE_AUTOSTART="${IS_UNIT_ENABLED}"
            if [ "${HAS_SYSTEMD}" -eq 1 ] && [ "${IS_UNIT_PRESENT}" -eq 0 ] && [ "${SHOULD_ASSUME_YES}" -eq 0 ] \
                && is_interactive && [ "${MODE:-deploy}" = "deploy" ] \
                && ask_yes_no "要設定開機自動啟動 Agent（systemd）嗎？" "y"; then
                SHOULD_ENABLE_AUTOSTART=1
            fi
            ;;
    esac
    if [ "${IS_UNIT_PRESENT}" -eq 1 ] || [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        # systemd 會解讀這些字元；路徑含有時無法安全寫入服務檔。
        case "${INSTALL_ROOT}${CONFIG_HOME}" in
            *[\"\\%\$]*) fail "安裝或設定路徑含 systemd 不支援的字元（\" \\ % \$）；無法管理開機自動啟動" ;;
        esac
    fi
}

plan_start() {
    local has_install=0
    if [ -n "${OLD_DIR}" ] || [ "${IS_UNIT_PRESENT}" -eq 1 ]; then
        has_install=1
    fi
    if [ "${IS_UNIT_ACTIVE}" -eq 1 ] || [ -n "${RUNNING_HOME}" ] || [ -n "${LEGACY_CONTAINERS}" ]; then
        WAS_RUNNING=1
    fi
    if [ "${SHOULD_START}" -eq 0 ]; then
        START_NOTE="不啟動（--no-start）"
    elif [ "${HAS_DOCKER}" -ne 1 ]; then
        SHOULD_START=0
        START_NOTE="不啟動（找不到 Docker）"
    elif [ "${SETTINGS_ACTION}" = "missing" ]; then
        SHOULD_START=0
        START_NOTE="不啟動（尚未設定；安裝後執行 taiphoon_agent init）"
    elif [ "${WAS_RUNNING}" -eq 0 ] && [ "${has_install}" -eq 1 ]; then
        SHOULD_START=0
        START_NOTE="不啟動（安裝前沒有執行，維持停止）"
    elif [ "${WAS_RUNNING}" -eq 1 ] && [ -n "${OLD_DIR}" ]; then
        START_NOTE="停止舊版後啟動新版並等待健康；失敗時切回 $(key_value "${OLD_DIR}/release.manifest" version)"
    else
        START_NOTE="安裝後啟動並等待健康"
    fi
    if [ "${WAS_RUNNING}" -eq 1 ] && [ "${SHOULD_START}" -eq 0 ]; then
        START_NOTE="停止舊版；${START_NOTE}"
    fi
}

# 啟動工具在 Linux 實機預設要求 chrony 已對時（time_sync=auto）；沒有 chronyc 時新版必定拒絕啟動，
# 替換執行中的 Agent 前就停止，避免停機後再切回。
check_time_sync_tool() {
    local policy="auto"
    if [ "${SHOULD_START}" -ne 1 ] || [ "$(uname -s)" != "Linux" ] || command -v chronyc >/dev/null 2>&1; then
        return 0
    fi
    if [ -n "${EFFECTIVE_SETTINGS}" ]; then
        if is_simulation_mode "${EFFECTIVE_SETTINGS}"; then
            return 0
        fi
        policy="$(settings_value "${EFFECTIVE_SETTINGS}" time_sync auto)"
    elif [ "${SETTINGS_ACTION}" != "init-agent" ]; then
        return 0
    fi
    if [ "${policy}" != "auto" ] && [ "${policy}" != "required" ]; then
        return 0
    fi
    if [ "${WAS_RUNNING}" -eq 1 ]; then
        fail "這台主機沒有 chrony，${VERSION} 實機啟動前的對時檢查必定失敗；未變更任何安裝。" \
            "多機飛行請先執行 sudo bash setup_time_sync.sh（見套件 README），" \
            "單機飛行可在 ${EFFECTIVE_SETTINGS} 加入 time_sync=off 後重試"
    fi
    warn "這台主機沒有 chrony，啟動前的對時檢查會失敗；多機飛行請先執行 setup_time_sync.sh，" \
        "單機飛行可在 settings.ini 設定 time_sync=off"
}

# 列出不是指定兩個目錄的安裝版本；只看 releases/ 內的實體目錄。
list_prunable() {
    local directory physical
    for directory in "${RELEASES_DIR}"/*; do
        if [ ! -d "${directory}" ] || [ -L "${directory}" ]; then
            continue
        fi
        physical="$(physical_dir "${directory}")"
        if [ "${physical}" != "$1" ] && [ "${physical}" != "$2" ]; then
            echo "${directory}"
        fi
    done
}

predicted_previous() {
    local target
    target="$(physical_dir "${RELEASES_DIR}/${VERSION}")"
    if [ -z "${OLD_DIR}" ] || [ "${OLD_DIR}" = "${target}" ] || [ "${OLD_DIR}" = "${PACKAGE_DIR}" ]; then
        echo "${PREVIOUS_DIR}"
    elif [ -n "${FLAT_DIR}" ] && [ "${OLD_DIR}" = "${FLAT_DIR}" ]; then
        physical_dir "${RELEASES_DIR}/${FLAT_VERSION}"
    else
        echo "${OLD_DIR}"
    fi
}

plan_prune() {
    PRUNE_DIRS="$(list_prunable "$(physical_dir "${RELEASES_DIR}/${VERSION}")" "$(predicted_previous)")"
}

verify_image_archive() {
    local archive expected
    archive="${PACKAGE_DIR}/images/taiphoon-agent-${IMAGE_KIND}-linux-${PACKAGE_ARCH}.tar.gz"
    if [ ! -f "${archive}" ]; then
        fail "套件缺少 $(basename "${archive}")；請重新下載完整平台包"
    fi
    expected="$(key_value "${PACKAGE_DIR}/release.manifest" "${IMAGE_KIND}_${PACKAGE_ARCH}_sha256")"
    log "核對 $(basename "${archive}") 的 SHA256..."
    if [ "$(file_sha256 "${archive}")" != "${expected}" ]; then
        fail "映像 SHA256 與 release.manifest 不符，未變更任何安裝；請重新下載完整平台包"
    fi
}

check_space() {
    local needed available probe
    if [ "$(physical_dir "${RELEASES_DIR}/${VERSION}")" = "${PACKAGE_DIR}" ]; then
        return 0
    fi
    needed="$(du -sk "${PACKAGE_DIR}" | awk '{ print $1 }')"
    probe="${INSTALL_ROOT}"
    while [ ! -d "${probe}" ]; do
        probe="$(dirname "${probe}")"
    done
    available="$(df -Pk "${probe}" | awk 'NR == 2 { print $4 }')"
    if [ "$((needed + SPACE_MARGIN_KB))" -gt "${available}" ]; then
        fail "安裝位置空間不足：約需 $(((needed + SPACE_MARGIN_KB) / 1024)) MB，可用 $((available / 1024)) MB"
    fi
}

describe_settings() {
    case "${SETTINGS_ACTION}" in
        keep) echo "沿用 ${CONFIG_HOME}/settings.ini（${MODE} 模式）" ;;
        copy) echo "複製 ${SOURCE_HOME}/settings.ini 到 ${CONFIG_HOME}/（原檔保留）" ;;
        replace) echo "以 ${SOURCE_HOME}/settings.ini 取代 ${CONFIG_HOME}/settings.ini（舊檔先備份）" ;;
        init-agent) echo "建立 deploy 設定，無人機編號 ${AGENT_ID}" ;;
        init-interactive) echo "安裝後進行首次設定（taiphoon_agent init）" ;;
        *) echo "尚未設定" ;;
    esac
}

describe_autostart() {
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ] && [ "${IS_UNIT_ENABLED}" -eq 1 ]; then
        echo "維持啟用（${SERVICE_NAME} 指向新安裝位置）"
    elif [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        echo "啟用 ${SERVICE_NAME}"
    elif [ "${IS_UNIT_ENABLED}" -eq 1 ]; then
        echo "停用 ${SERVICE_NAME}"
    elif [ "${HAS_SYSTEMD}" -eq 1 ]; then
        echo "不設定"
    else
        echo "不支援（此系統沒有 systemd）"
    fi
}

print_install_plan() {
    local directory
    echo
    echo "Taiphoon Agent 安裝計畫"
    echo "  新版本：${VERSION}（Agent 映像 ${PACKAGE_ARCH}）"
    if [ -n "${OLD_DIR}" ]; then
        echo "  目前版本：$(key_value "${OLD_DIR}/release.manifest" version)（${OLD_DIR}）"
    else
        echo "  目前版本：未安裝"
    fi
    echo "  安裝位置：${RELEASES_DIR}/${VERSION}"
    echo "  設定：$(describe_settings)"
    echo "  開機自動啟動：$(describe_autostart)"
    if [ -n "${LEGACY_CONTAINERS}" ]; then
        echo "  舊入口容器：${LEGACY_CONTAINERS}（停止並取消自動重啟，不刪除）"
    fi
    echo "  啟動：${START_NOTE}"
    if [ -n "${PRUNE_DIRS}" ]; then
        echo "  移除舊安裝版本（Docker 映像保留）："
        while IFS= read -r directory; do
            echo "    ${directory}"
        done <<< "${PRUNE_DIRS}"
    fi
    echo
}

acquire_lock() {
    mkdir -p "${INSTALL_ROOT}"
    if ! mkdir "${INSTALL_ROOT}/.install.lock" 2>/dev/null; then
        fail "另一個 install.sh 正在執行；確認沒有其他安裝後，移除 ${INSTALL_ROOT}/.install.lock 再重試"
    fi
    IS_LOCK_HELD=1
}

# 先取得 sudo 授權，避免停止舊版後才因密碼錯誤中斷。
prepare_root_access() {
    if [ "$(id -u)" -eq 0 ] || [ "${HAS_SYSTEMD}" -ne 1 ]; then
        return 0
    fi
    if [ "${IS_UNIT_PRESENT}" -eq 1 ] || [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        log "管理 ${SERVICE_NAME} 需要 sudo 權限"
        if ! sudo -v; then
            fail "無法取得 sudo 權限；未變更任何安裝"
        fi
    fi
}

# 舊版仍在執行時完成複製，縮短停機時間；同版本已在安裝位置則不重複複製。
stage_release() {
    local name
    if [ "$(physical_dir "${RELEASES_DIR}/${VERSION}")" = "${PACKAGE_DIR}" ]; then
        log "套件已位於安裝位置，不重新複製"
        return 0
    fi
    mkdir -p "${RELEASES_DIR}"
    STAGING_DIR="$(mktemp -d "${RELEASES_DIR}/.staging-${VERSION}.XXXXXX")"
    log "複製套件到安裝位置..."
    for name in ${PACKAGE_FILES}; do
        if [ -f "${PACKAGE_DIR}/${name}" ]; then
            cp -p "${PACKAGE_DIR}/${name}" "${STAGING_DIR}/"
        fi
    done
    if [ -f "${PACKAGE_DIR}/${MACOS_ENTRY}" ]; then
        cp -p "${PACKAGE_DIR}/${MACOS_ENTRY}" "${STAGING_DIR}/"
    fi
    mkdir "${STAGING_DIR}/images"
    cp -p "${PACKAGE_DIR}/images/"*.tar.gz "${STAGING_DIR}/images/"
    chmod 755 "${STAGING_DIR}" "${STAGING_DIR}/taiphoon_agent"
}

preload_image() {
    local image archive
    if [ "${HAS_DOCKER}" -ne 1 ]; then
        return 0
    fi
    image="taiphoon-agent:${VERSION}-${IMAGE_KIND}-${DOCKER_ARCH}"
    if docker image inspect "${image}" >/dev/null 2>&1; then
        log "Docker 已有 ${image}；啟動時由啟動工具核對內容"
        return 0
    fi
    archive="${PACKAGE_DIR}/images/taiphoon-agent-${IMAGE_KIND}-linux-${DOCKER_ARCH}.tar.gz"
    log "載入 Agent 映像 ${image}（樹莓派約需數分鐘；舊版仍在執行）..."
    if ! docker load --input "${archive}" >/dev/null; then
        fail "docker load 失敗（請確認 Docker 磁碟空間）；未變更既有安裝"
    fi
    if ! docker image inspect "${image}" >/dev/null 2>&1; then
        fail "載入後找不到 ${image}；未變更既有安裝，請重新下載完整平台包"
    fi
}

abort_before_switch() {
    if [ "${IS_UNIT_ACTIVE}" -eq 1 ]; then
        run_root systemctl start "${SERVICE_NAME}" || warn "無法恢復 ${SERVICE_NAME}"
    fi
    fail "$1；未切換版本"
}

stop_existing() {
    local name
    # 服務在 auto-restart 等待中時 is-active 不為 active；一律 stop 才能取消待執行的重試。
    if [ "${IS_UNIT_PRESENT}" -eq 1 ]; then
        log "停止 ${SERVICE_NAME}..."
        run_root systemctl stop "${SERVICE_NAME}" || abort_before_switch "無法停止 ${SERVICE_NAME}"
    fi
    if [ -n "${RUNNING_HOME}" ] && [ -n "$(project_container_ids)" ]; then
        log "停止執行中的 Agent（設定目錄 ${RUNNING_HOME}）..."
        docker compose --project-name "${PROJECT_NAME}" --project-directory "${RUNNING_HOME}" \
            --file "${RUNNING_FILE}" down --timeout "${STOP_TIMEOUT_SECONDS}" \
            || abort_before_switch "無法停止既有 Agent，請查看 docker ps"
    fi
    for name in ${LEGACY_CONTAINERS}; do
        log "停止舊入口容器 ${name}（不刪除）..."
        if ! docker update --restart=no "${name}" >/dev/null \
            || ! docker stop --time "${STOP_TIMEOUT_SECONDS}" "${name}" >/dev/null; then
            abort_before_switch "無法停止 ${name}"
        fi
    done
    if [ -n "${RUNNING_HOME}" ] && [ -n "$(project_container_ids)" ]; then
        abort_before_switch "仍有 ${PROJECT_NAME} 容器執行中，請查看 docker ps"
    fi
}

migrate_flat_install() {
    local destination="${RELEASES_DIR}/${FLAT_VERSION}"
    local name
    if [ -z "${FLAT_DIR}" ]; then
        return 0
    fi
    if [ -e "${destination}" ]; then
        # 同版本已在 releases/；平鋪檔案只是重複副本。
        for name in ${FLAT_FILES}; do
            rm -rf "${INSTALL_ROOT:?}/${name}"
        done
    else
        mkdir -p "${destination}"
        for name in ${FLAT_FILES}; do
            if [ -e "${INSTALL_ROOT}/${name}" ]; then
                mv "${INSTALL_ROOT}/${name}" "${destination}/"
            fi
        done
    fi
    log "舊版平鋪安裝 ${FLAT_VERSION} 已移到 ${destination}"
    if [ "${OLD_DIR}" = "${FLAT_DIR}" ]; then
        OLD_DIR="$(physical_dir "${destination}")"
    fi
}

switch_release() {
    local target="${RELEASES_DIR}/${VERSION}"
    migrate_flat_install
    if [ -n "${STAGING_DIR}" ]; then
        if [ -e "${target}" ]; then
            # 同版本重新安裝；服務已停止，以剛複製的內容取代。
            rm -rf "${target}"
        fi
        mv "${STAGING_DIR}" "${target}"
        STAGING_DIR=""
    fi
    # 從服務正在使用的同一個套件目錄重跑時，舊版與新版相同，不當作上一版。
    if [ -n "${OLD_DIR}" ] && [ "${OLD_DIR}" != "$(physical_dir "${target}")" ] \
        && [ "${OLD_DIR}" != "${PACKAGE_DIR}" ]; then
        ln -sfn "${OLD_DIR}" "${INSTALL_ROOT}/previous"
        IS_VERSION_CHANGED=1
    fi
    ln -sfn "releases/${VERSION}" "${INSTALL_ROOT}/current"
    # 留一份在固定位置，切回不支援 --rollback 的舊版後仍能再切回來。
    cp -p "${INSTALL_ROOT}/current/install.sh" "${INSTALL_ROOT}/install.sh"
    log "已切換安裝版本：${VERSION}"
}

ensure_path() {
    local profile path_line
    case ":${PATH}:" in
        *":${BIN_DIR}:"*) return 0 ;;
    esac
    case "${SHELL:-}" in
        */zsh) profile="${HOME}/.zshrc" ;;
        */bash) profile="${HOME}/.bashrc" ;;
        *) profile="${HOME}/.profile" ;;
    esac
    path_line='export PATH="$HOME/.local/bin:$PATH"'
    if [ ! -f "${profile}" ] || ! grep -Fqx "${path_line}" "${profile}"; then
        printf '\n# Taiphoon Agent\n%s\n' "${path_line}" >> "${profile}"
    fi
    PATH_NOTE="已將 ~/.local/bin 加入 ${profile}；重新開啟終端後可直接使用 taiphoon_agent。"
}

# wrapper 指向 current，升級與切回只需更新 symlink；原生工具仍以實際路徑找到同版 manifest。
install_wrapper() {
    local draft
    mkdir -p "${BIN_DIR}"
    draft="$(mktemp "${BIN_DIR}/.taiphoon_agent.XXXXXX")"
    printf '%s\n' '#!/usr/bin/env bash' \
        '# 由 Taiphoon Agent install.sh 產生；執行目前安裝版本。' \
        'exec "${HOME}/.local/share/taiphoon_agent/current/taiphoon_agent" "$@"' > "${draft}"
    chmod 755 "${draft}"
    mv -f "${draft}" "${BIN_DIR}/taiphoon_agent"
    ensure_path
}

launcher() {
    TAIPHOON_AGENT_HOME="${CONFIG_HOME}" "${INSTALL_ROOT}/current/taiphoon_agent" "$@"
}

apply_settings() {
    local backup
    case "${SETTINGS_ACTION}" in
        copy|replace)
            mkdir -p "${CONFIG_HOME}"
            if [ "${SETTINGS_ACTION}" = "replace" ]; then
                backup="${CONFIG_HOME}/settings.ini.bak-$(date +%Y%m%d-%H%M%S)"
                cp -p "${CONFIG_HOME}/settings.ini" "${backup}"
                log "原設定已備份：${backup}"
            fi
            cp -p "${SOURCE_HOME}/settings.ini" "${CONFIG_HOME}/settings.ini"
            log "設定已複製到 ${CONFIG_HOME}/settings.ini"
            ;;
        init-agent)
            # 與啟動工具首次設定的提問順序一致：模式、無人機編號、GCS IPv4（留空）。
            if ! printf 'deploy\n%s\n\n' "${AGENT_ID}" | launcher init >/dev/null; then
                fail "無法建立設定；請執行 taiphoon_agent init"
            fi
            log "已建立無人機 ${AGENT_ID} 的 deploy 設定：${CONFIG_HOME}/settings.ini"
            ;;
        init-interactive)
            echo "首次設定（之後可用 taiphoon_agent config 修改）："
            if ! launcher init; then
                warn "首次設定未完成；之後請執行 taiphoon_agent init"
            fi
            ;;
    esac
    if [ ! -f "${CONFIG_HOME}/settings.ini" ]; then
        SHOULD_START=0
    fi
}

render_unit() {
    cat <<EOF
# 由 Taiphoon Agent install.sh 產生；升級或切回版本只更新 current，不需修改此檔。
[Unit]
Description=Taiphoon Agent
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target
StartLimitIntervalSec=300
StartLimitBurst=6

[Service]
Type=oneshot
RemainAfterExit=yes
User=$(id -un)
Environment="TAIPHOON_AGENT_HOME=${CONFIG_HOME}"
ExecStart="${INSTALL_ROOT}/current/taiphoon_agent" start
ExecStop="${INSTALL_ROOT}/current/taiphoon_agent" stop
Restart=on-failure
RestartSec=10
# 包含首次載入映像與等待對時（60 秒）的時間。
TimeoutStartSec=600

[Install]
WantedBy=multi-user.target
EOF
}

apply_unit() {
    if [ "${HAS_SYSTEMD}" -ne 1 ]; then
        return 0
    fi
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ] && [ ! -f "${CONFIG_HOME}/settings.ini" ]; then
        warn "尚未設定，不啟用開機自動啟動；完成 taiphoon_agent init 後執行 install.sh --autostart"
        SHOULD_ENABLE_AUTOSTART=0
    fi
    if [ "${IS_UNIT_PRESENT}" -eq 0 ] && [ "${SHOULD_ENABLE_AUTOSTART}" -eq 0 ]; then
        return 0
    fi
    UNIT_DRAFT="$(mktemp)"
    render_unit > "${UNIT_DRAFT}"
    if [ "${IS_UNIT_PRESENT}" -eq 0 ] || ! cmp -s "${UNIT_DRAFT}" "${UNIT_FILE}"; then
        if [ "${IS_UNIT_PRESENT}" -eq 1 ]; then
            mkdir -p "${INSTALL_ROOT}/backup"
            cp "${UNIT_FILE}" "${INSTALL_ROOT}/backup/${SERVICE_NAME}.$(date +%Y%m%d-%H%M%S)"
            log "原服務檔已備份到 ${INSTALL_ROOT}/backup/"
        fi
        run_root install -m 0644 "${UNIT_DRAFT}" "${UNIT_FILE}"
        run_root systemctl daemon-reload
        log "已更新 ${UNIT_FILE}"
    fi
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ] && [ "${IS_UNIT_ENABLED}" -eq 0 ]; then
        run_root systemctl enable "${SERVICE_NAME}"
    elif [ "${SHOULD_ENABLE_AUTOSTART}" -eq 0 ] && [ "${IS_UNIT_ENABLED}" -eq 1 ]; then
        run_root systemctl disable "${SERVICE_NAME}"
    fi
}

# 開機自動啟動時一律經 systemd 啟停，避免服務狀態與實際容器不同步。
start_agent() {
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        run_root systemctl start "${SERVICE_NAME}"
    else
        launcher start
    fi
}

stop_agent() {
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        run_root systemctl stop "${SERVICE_NAME}" || warn "無法停止 ${SERVICE_NAME}"
        run_root systemctl reset-failed "${SERVICE_NAME}" >/dev/null 2>&1 || true
    fi
    # 啟動失敗的 oneshot 服務不會執行 ExecStop；已建立的容器由啟動工具停止。
    if [ -f "${CONFIG_HOME}/compose.yaml" ]; then
        launcher stop || warn "無法停止 ${VERSION} 的容器；請查看 docker ps"
    fi
}

show_failure_log() {
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ] && command -v journalctl >/dev/null 2>&1; then
        echo "---- journalctl -u ${SERVICE_NAME}（最近 30 行）----" >&2
        run_root journalctl -u "${SERVICE_NAME}" -n 30 --no-pager >&2 || warn "無法讀取 journal"
    fi
}

swap_current_previous() {
    local current_target previous_target
    current_target="$(readlink "${INSTALL_ROOT}/current")"
    previous_target="$(readlink "${INSTALL_ROOT}/previous")"
    ln -sfn "${previous_target}" "${INSTALL_ROOT}/current"
    ln -sfn "${current_target}" "${INSTALL_ROOT}/previous"
}

# 只在確實換了版本時切回；同版本重新安裝沒有更適合的版本可回復。
recover_previous() {
    local failed_version="$1" previous_version
    warn "${failed_version} 啟動失敗（原因見上方訊息）"
    show_failure_log
    stop_agent
    if [ "${IS_VERSION_CHANGED}" -ne 1 ]; then
        fail "這次沒有更換版本，不自動切回；請依上方訊息處理後執行 taiphoon_agent start"
    fi
    if [ ! -x "${INSTALL_ROOT}/previous/taiphoon_agent" ]; then
        fail "上一版已不存在，無法切回；請依上方訊息處理後執行 taiphoon_agent start"
    fi
    previous_version="$(key_value "${INSTALL_ROOT}/previous/release.manifest" version)"
    swap_current_previous
    if [ "${WAS_RUNNING}" -ne 1 ]; then
        fail "${failed_version} 啟動失敗，已切回 ${previous_version}（未啟動）"
    fi
    log "切回 ${previous_version} 並啟動..."
    if start_agent; then
        fail "${failed_version} 啟動失敗，已切回 ${previous_version} 並恢復執行"
    fi
    fail "${failed_version} 與 ${previous_version} 都無法啟動；請查看 taiphoon_agent logs"
}

start_installed() {
    if [ "${SHOULD_START}" -ne 1 ]; then
        return 0
    fi
    log "啟動 ${VERSION}（等待容器健康）..."
    if start_agent; then
        IS_STARTED=1
        return 0
    fi
    recover_previous "${VERSION}"
}

prune_releases() {
    local directory physical current previous
    current="$(physical_dir "${INSTALL_ROOT}/current")"
    previous="$(physical_dir "${INSTALL_ROOT}/previous")"
    while IFS= read -r directory; do
        if [ -z "${directory}" ] || [ ! -d "${directory}" ]; then
            continue
        fi
        # 只刪除計畫中列出、且仍不是 current／previous 的目錄。
        physical="$(physical_dir "${directory}")"
        if [ "${physical}" != "${current}" ] && [ "${physical}" != "${previous}" ]; then
            rm -rf "${directory}"
            log "已移除舊安裝版本：${directory}"
        fi
    done <<< "${PRUNE_DIRS}"
}

print_summary() {
    echo
    log "完成：目前版本 ${VERSION}"
    if [ "${IS_STARTED}" -eq 1 ]; then
        launcher status || warn "無法取得狀態；請稍後執行 taiphoon_agent status"
    elif [ -f "${CONFIG_HOME}/settings.ini" ]; then
        echo "未啟動；需要時執行 taiphoon_agent start"
    else
        echo "尚未設定；請執行 taiphoon_agent init，再執行 taiphoon_agent start"
    fi
    if [ "${SHOULD_ENABLE_AUTOSTART}" -eq 1 ]; then
        echo "開機自動啟動已啟用；請用 sudo systemctl start／stop／restart ${SERVICE_NAME} 管理，避免與手動啟停不同步。"
    fi
    if [ -L "${INSTALL_ROOT}/previous" ]; then
        echo "切回上一版：bash ${INSTALL_ROOT}/install.sh --rollback"
    fi
    if [ -n "${PATH_NOTE}" ]; then
        echo "${PATH_NOTE}"
    fi
    echo "舊版 Docker 映像不會自動刪除；需要空間時以 docker image ls taiphoon-agent 確認後自行移除。"
}

install_release() {
    validate_package
    check_docker
    detect_existing
    plan_settings
    plan_autostart
    plan_start
    check_time_sync_tool
    plan_prune
    verify_image_archive
    check_space
    print_install_plan
    confirm_or_stop "開始安裝？"
    acquire_lock
    prepare_root_access
    stage_release
    preload_image
    stop_existing
    switch_release
    install_wrapper
    apply_settings
    apply_unit
    start_installed
    prune_releases
    print_summary
}

rollback() {
    local current_version previous_version
    check_docker
    detect_existing
    if [ -n "${FLAT_DIR}" ] || [ ! -L "${INSTALL_ROOT}/current" ] || [ -z "${PREVIOUS_DIR}" ] \
        || [ ! -x "${INSTALL_ROOT}/previous/taiphoon_agent" ]; then
        fail "沒有可切回的上一版（需要由新版 install.sh 安裝過兩個版本）"
    fi
    current_version="$(key_value "${INSTALL_ROOT}/current/release.manifest" version)"
    previous_version="$(key_value "${INSTALL_ROOT}/previous/release.manifest" version)"
    VERSION="${previous_version}"
    SHOULD_ENABLE_AUTOSTART="${IS_UNIT_ENABLED}"
    IS_VERSION_CHANGED=1
    if [ "${IS_UNIT_ACTIVE}" -eq 1 ] || [ -n "${RUNNING_HOME}" ]; then
        WAS_RUNNING=1
    fi
    echo
    echo "切回上一版：${current_version} → ${previous_version}（${PREVIOUS_DIR}）"
    echo "  設定：沿用 ${CONFIG_HOME}/settings.ini"
    if [ "${WAS_RUNNING}" -eq 1 ]; then
        echo "  啟動：停止 ${current_version} 後啟動 ${previous_version}；失敗時切回 ${current_version}"
    else
        echo "  啟動：目前沒有執行，切換後維持停止"
    fi
    echo
    confirm_or_stop "開始切換？"
    acquire_lock
    prepare_root_access
    stop_existing
    swap_current_previous
    log "目前版本：${previous_version}"
    if [ "${WAS_RUNNING}" -eq 1 ]; then
        log "啟動 ${previous_version}（等待容器健康）..."
        if ! start_agent; then
            recover_previous "${previous_version}"
        fi
        IS_STARTED=1
    fi
    print_summary
}

main() {
    parse_arguments "$@"
    require_regular_user
    PACKAGE_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
    CONFIG_HOME="$(absolute_path "${TAIPHOON_AGENT_HOME:-${HOME}/.config/taiphoon_agent}")"
    detect_platform
    if [ "${ACTION}" = "rollback" ]; then
        rollback
    else
        install_release
    fi
}

# 整個檔案讀入後才執行，避免安裝過程覆寫 install.sh 時 bash 讀到新內容。
main "$@"
