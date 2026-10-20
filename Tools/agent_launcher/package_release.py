#!/usr/bin/env python3
"""
@file package_release.py
@brief 將 CI 產物組成五個自包含平台包；校驗資訊只內含於包內
@author Taiphoon Agent Team
@date 2026-10-02
"""
import os
from pathlib import Path
import shutil
import tarfile
import zipfile


MAX_RELEASE_ASSET_BYTES = 2 * 1024 * 1024 * 1024


def main():
    """在 CI 工作目錄組裝 Release 下載包，不呼叫發布 API 或刪除資料。"""
    version = os.environ['VERSION']
    artifacts = Path('artifacts')
    release = Path('release')
    release.mkdir(exist_ok=True)
    manifest = f'version={version}\nrepository=Rex-Taiphoon/Taiphoon_Agent\n'
    manifest += (artifacts / 'linux-amd64/images.manifest').read_text()
    manifest += (artifacts / 'linux-arm64/images.manifest').read_text()
    for platform in ('linux-amd64', 'linux-arm64', 'windows-amd64', 'macos-amd64', 'macos-arm64'):
        architecture = platform.rsplit('-', 1)[1]
        destination = Path('packages') / platform
        (destination / 'images').mkdir(parents=True, exist_ok=True)
        (destination / 'release.manifest').write_text(manifest, encoding='utf-8')
        shutil.copy2('docker/sitl-entrypoint.sh', destination)
        shutil.copy2('Tools/agent_launcher/PACKAGE-README.md', destination / 'README.md')
        for kind in ('runtime', 'dev'):
            image = artifacts / f'linux-{architecture}' / f'taiphoon-agent-{kind}-linux-{architecture}.tar.gz'
            shutil.copy2(image, destination / 'images')
        if platform == 'windows-amd64':
            shutil.copy2(artifacts / 'native-windows-amd64/taiphoon_agent.exe', destination)
            shutil.copy2('Tools/agent_launcher/install.ps1', destination)
            output = release / f'taiphoon_agent-{version}-{platform}.zip'
            with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_STORED) as archive:
                for path in sorted(destination.rglob('*')):
                    if path.is_file():
                        archive.write(path, path.relative_to(destination))
        else:
            native = artifacts / ('native-macos-universal' if platform.startswith('macos-') else platform)
            shutil.copy2(native / 'taiphoon_agent', destination)
            shutil.copy2('Tools/agent_launcher/install.sh', destination)
            executable_names = ['taiphoon_agent', 'install.sh', 'sitl-entrypoint.sh']
            if platform.startswith('linux-'):
                # 樹莓派／Linux 實機多機飛行需要主機對時；腳本在主機執行，不在容器內。
                shutil.copy2('Tools/time_sync/setup_time_sync.sh', destination)
                executable_names.append('setup_time_sync.sh')
            if platform.startswith('macos-'):
                name = 'Start Taiphoon Agent.command'
                shutil.copy2(Path('Tools/agent_launcher') / name, destination)
                executable_names.append(name)
            for name in executable_names:
                (destination / name).chmod(0o755)
            output = release / f'taiphoon_agent-{version}-{platform}.tar.gz'
            with tarfile.open(output, 'w:gz', compresslevel=1) as archive:
                archive.add(destination, arcname='.')
        if output.stat().st_size >= MAX_RELEASE_ASSET_BYTES:
            raise RuntimeError(f'{output.name} 超過 GitHub Release 單檔限制；禁止發布不完整產物')
        print(output)


if __name__ == '__main__':
    main()
