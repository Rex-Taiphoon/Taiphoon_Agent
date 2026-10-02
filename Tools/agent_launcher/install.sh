#!/usr/bin/env bash
# @file install.sh
# @brief 安裝至使用者目錄，不需要 sudo、不修改 Docker 或刪除使用者資料
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eo pipefail
package_dir="$(cd "$(dirname "$0")" && pwd)"
install_dir="${HOME}/.local/share/taiphoon_agent"
bin_dir="${HOME}/.local/bin"
mkdir -p "${install_dir}/images" "${bin_dir}"
cp "${package_dir}/taiphoon_agent" "${package_dir}/release.manifest" \
    "${package_dir}/sitl-entrypoint.sh" "${package_dir}/README.md" "${install_dir}/"
cp "${package_dir}/images/"*.tar.gz "${install_dir}/images/"
chmod +x "${install_dir}/taiphoon_agent"
# wrapper 避免需要覆寫 symlink，也讓原生工具找到同目錄的 Release manifest。
printf '#!/usr/bin/env bash\nexec "${HOME}/.local/share/taiphoon_agent/taiphoon_agent" "$@"\n' \
    > "${bin_dir}/taiphoon_agent"
chmod +x "${bin_dir}/taiphoon_agent"
echo "已安裝。執行：${bin_dir}/taiphoon_agent start"
case ":${PATH}:" in
    *":${bin_dir}:"*) ;;
    *)
        case "${SHELL:-}" in
            */zsh) profile="${HOME}/.zshrc" ;;
            */bash) profile="${HOME}/.bashrc" ;;
            *) profile="${HOME}/.profile" ;;
        esac
        path_line='export PATH="$HOME/.local/bin:$PATH"'
        if [ ! -f "${profile}" ] || ! grep -Fqx "${path_line}" "${profile}"; then
            printf '\n# Taiphoon Agent\n%s\n' "${path_line}" >> "${profile}"
        fi
        echo "已加入 ${profile}；重新開啟終端後可直接使用 taiphoon_agent start。"
        ;;
esac
