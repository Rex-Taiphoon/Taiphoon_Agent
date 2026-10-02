#!/usr/bin/env python3
"""
@file deployment_package_test.py
@brief 在 Docker 容器執行真正的 CI 打包步驟，驗證自包含平台包與無獨立校驗產物
@author Taiphoon Agent Team
@date 2026-10-02
"""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import zipfile

import yaml


def main():
    """用隔離的編譯產物替身驗證打包流程，不呼叫 Docker build 或發布 API。"""
    source = Path(__file__).resolve().parents[1]
    root = Path(tempfile.mkdtemp(prefix='taiphoon-package-test-'))
    for relative in ('docker/sitl-entrypoint.sh', 'Tools/agent_launcher/PACKAGE-README.md',
                     'Tools/agent_launcher/install.sh', 'Tools/agent_launcher/install.ps1',
                     'Tools/agent_launcher/Start Taiphoon Agent.command', 'Tools/agent_launcher/package_release.py'):
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / relative, destination)
    for architecture in ('amd64', 'arm64'):
        directory = root / 'artifacts' / f'linux-{architecture}'
        directory.mkdir(parents=True)
        (directory / 'taiphoon_agent').write_bytes(b'ELF-native-launcher-fixture')
        metadata = ''
        for kind in ('runtime', 'dev'):
            data = f'{kind}-{architecture}-compiled-image-fixture'.encode()
            (directory / f'taiphoon-agent-{kind}-linux-{architecture}.tar.gz').write_bytes(data)
            metadata += f'{kind}_{architecture}_sha256={hashlib.sha256(data).hexdigest()}\n'
            metadata += f'{kind}_{architecture}_id=sha256:{"a" * 64}\n'
        (directory / 'images.manifest').write_text(metadata)
    for platform, filename in (('windows-amd64', 'taiphoon_agent.exe'), ('macos-universal', 'taiphoon_agent')):
        directory = root / 'artifacts' / f'native-{platform}'
        directory.mkdir(parents=True)
        (directory / filename).write_bytes(b'native-launcher-fixture')
    workflow = yaml.safe_load((source / '.github/workflows/agent-build.yml').read_text())
    steps = workflow['jobs']['package']['steps']
    command = next(step['run'] for step in steps if step.get('name', '').startswith('組裝各平台下載包'))
    environment = dict(os.environ, VERSION='v0.0.0-package-test')
    subprocess.run(['bash', '-c', command], cwd=root, env=environment, check=True, timeout=30)
    artifacts = list((root / 'release').iterdir())
    assert len(artifacts) == 5, 'Release 應只包含五個完整平台包'
    assert all(path.name.startswith('taiphoon_agent-') for path in artifacts), '不應發布映像或校驗的獨立檔案'
    for path in artifacts:
        architecture = 'arm64' if '-arm64.' in path.name else 'amd64'
        if path.suffix == '.zip':
            with zipfile.ZipFile(path) as archive:
                names = archive.namelist()
                assert 'taiphoon_agent.exe' in names
                for kind in ('runtime', 'dev'):
                    assert f'images/taiphoon-agent-{kind}-linux-{architecture}.tar.gz' in names
                assert 'SHA256SUMS' not in names
                assert 'release.manifest' in names
        else:
            with tarfile.open(path, 'r:gz') as archive:
                members = {member.name.removeprefix('./'): member for member in archive.getmembers()}
                assert 'taiphoon_agent' in members and members['taiphoon_agent'].mode & 0o111
                manifest = archive.extractfile(members['release.manifest']).read().decode()
                metadata = dict(line.split('=', 1) for line in manifest.splitlines())
                for kind in ('runtime', 'dev'):
                    filename = f'images/taiphoon-agent-{kind}-linux-{architecture}.tar.gz'
                    payload = archive.extractfile(members[filename]).read()
                    assert hashlib.sha256(payload).hexdigest() == metadata[f'{kind}_{architecture}_sha256']
                other_architecture = 'amd64' if architecture == 'arm64' else 'arm64'
                assert not any(other_architecture in name and name.startswith('images/') for name in members)
                if '-macos-' in path.name:
                    assert members['Start Taiphoon Agent.command'].mode & 0o111
    home = root / 'home with 中文 spaces'
    home.mkdir()
    install_environment = dict(os.environ, HOME=str(home), SHELL='/bin/zsh')
    installer = root / 'packages/linux-amd64/install.sh'
    for _ in range(2):
        subprocess.run(['bash', str(installer)], env=install_environment, check=True,
                       capture_output=True, text=True, timeout=10)
    assert (home / '.local/bin/taiphoon_agent').is_file()
    assert (home / '.local/share/taiphoon_agent/images/taiphoon-agent-dev-linux-amd64.tar.gz').is_file()
    assert (home / '.zshrc').read_text().count('export PATH=') == 1, '安裝應可重複且不重複加入 PATH'
    print('PASS: 五個自包含平台包、兩種映像、正確架構、執行權限、無獨立校驗檔')


if __name__ == '__main__':
    main()
