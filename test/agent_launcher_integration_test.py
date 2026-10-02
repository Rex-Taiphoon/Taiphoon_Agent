#!/usr/bin/env python3
"""
@file agent_launcher_integration_test.py
@brief 在 Docker 開發容器以隔離程序替身驗證實際原生 CLI；不啟動其他容器
@author Taiphoon Agent Team
@date 2026-10-02
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import yaml

IMAGE_CONFIG = '{"Env":["ROS_DISTRO=humble"],"Cmd":null,"Entrypoint":["/runtime-entrypoint.sh"]}'
IMAGE_LAYERS = '["sha256:' + 'c' * 64 + '"]'


DOCKER_FIXTURE = r'''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
arguments = sys.argv[1:]
root = Path(os.environ['FIXTURE_HOME'])
with (root / 'calls.jsonl').open('a') as stream:
    stream.write(json.dumps(arguments) + '\n')
state = root / 'state.json'
values = json.loads(state.read_text()) if state.exists() else {'loaded': False, 'running': False}
if arguments[0] == 'info':
    if os.environ.get('FIXTURE_DAEMON_FAILURE'):
        print('Cannot connect to the Docker daemon', file=sys.stderr)
        sys.exit(1)
    print('linux|' + os.environ.get('FIXTURE_ARCH', 'aarch64') + '|'
          + os.environ.get('FIXTURE_DOCKER_OS', 'Docker Desktop'))
elif arguments[0] == 'context':
    print('unix:///var/run/docker.sock')
elif arguments[0] == 'image':
    if not values['loaded']:
        sys.exit(1)
    fields = {'{{.Architecture}}': os.environ.get('FIXTURE_IMAGE_ARCH', 'arm64'),
              '{{.Os}}': os.environ.get('FIXTURE_IMAGE_OS', 'linux'),
              '{{.Id}}': os.environ.get('FIXTURE_IMAGE_ID', 'sha256:' + 'a' * 64),
              os.environ['FIXTURE_CONFIG_FORMAT']: os.environ.get('FIXTURE_IMAGE_CONFIG', ''),
              '{{json .RootFS.Layers}}': os.environ.get('FIXTURE_IMAGE_LAYERS', '')}
    print(fields[arguments[arguments.index('--format') + 1]] if '--format' in arguments else '{}')
elif arguments[0] == 'load':
    values['loaded'] = True
elif arguments[0] == 'inspect':
    print('running healthy' if values['running'] else 'exited unhealthy')
elif arguments[0] == 'compose':
    if '--file' not in arguments:
        if os.environ.get('FIXTURE_COMPOSE_FAILURE'):
            sys.exit(1)
        print('Docker Compose version v2.24.0')
    else:
        import yaml
        compose = yaml.safe_load(Path(arguments[arguments.index('--file') + 1]).read_text())
        assert compose['services']['agent_1']['pull_policy'] == 'never'
        operation = arguments[arguments.index('--file') + 2]
        if operation == 'ps' and '--quiet' in arguments:
            print('container-one' if values['running'] else '')
        elif operation == 'up':
            assert '--no-build' in arguments and '--wait' in arguments
            values['running'] = True
        elif operation == 'stop':
            values['running'] = False
        elif operation == 'exec':
            print('True')
elif arguments[0] == 'ps':
    print(os.environ.get('FIXTURE_LEGACY_NAME', ''))
else:
    raise AssertionError(arguments)
state.write_text(json.dumps(values))
'''

def run_cli(executable, environment, *arguments, input_text=None, expected=0):
    """執行真正的 C++ CLI，檢查回傳碼與錯誤輸出。"""
    result = subprocess.run([str(executable), *arguments], input=input_text,
                            text=True, capture_output=True, env=environment, timeout=20)
    if result.returncode != expected:
        raise AssertionError(f'{arguments}: code={result.returncode}\n{result.stdout}\n{result.stderr}')
    return result


def main():
    """驗證首次啟動、重複啟動、停止、設定變更與下載失敗的完整邊界。"""
    original = Path(sys.argv[1]).resolve()
    version = subprocess.check_output([str(original), '--version'], text=True).strip()
    # 保留測試資料；容器生命週期負責隔離，不刪除專案檔案或使用者目錄。
    root = Path(tempfile.mkdtemp(prefix='taiphoon-cli-中文 space-'))
    package = root / 'package with spaces'
    package.mkdir()
    executable = package / 'taiphoon_agent'
    shutil.copy2(original, executable)
    digest = hashlib.sha256(b'test image archive').hexdigest()
    manifest = f'version={version}\nrepository=Rex-Taiphoon/Taiphoon_Agent\n'
    for architecture in ('amd64', 'arm64'):
        for kind in ('runtime', 'dev'):
            manifest += f'{kind}_{architecture}_sha256={digest}\n'
            manifest += f'{kind}_{architecture}_id=sha256:{"a" * 64}\n'
            manifest += f'{kind}_{architecture}_config={IMAGE_CONFIG}\n'
            manifest += f'{kind}_{architecture}_layers={IMAGE_LAYERS}\n'
    (package / 'release.manifest').write_text(manifest)
    images = package / 'images'
    images.mkdir()
    archive = images / 'taiphoon-agent-runtime-linux-arm64.tar.gz'
    archive.write_bytes(b'test image archive')
    fixture = root / 'fixture'
    fixture.mkdir()
    for name, text in (('docker', DOCKER_FIXTURE),):
        path = fixture / name
        path.write_text(text)
        path.chmod(0o755)
    environment = dict(os.environ, TAIPHOON_AGENT_HOME=str(root / 'config'), FIXTURE_HOME=str(root),
                       PATH=str(fixture) + os.pathsep + os.environ['PATH'],
                       FIXTURE_IMAGE_ID='sha256:' + 'b' * 64,
                       FIXTURE_IMAGE_CONFIG=IMAGE_CONFIG, FIXTURE_IMAGE_LAYERS=IMAGE_LAYERS,
                       FIXTURE_CONFIG_FORMAT=(Path(__file__).resolve().parents[1]
                                              / 'Tools/agent_launcher/image-config.format').read_text().strip())
    # 初次缺少依賴時不詢問／保存模式，失敗原因須能直接指引修復。
    missing_environment = dict(environment, PATH=str(root / 'no-docker-tools'))
    result = run_cli(executable, missing_environment, expected=1)
    assert '找不到 Docker CLI' in result.stderr
    assert '模式 deploy/dev/sim' not in result.stdout
    assert not (root / 'config' / 'settings.ini').exists()
    for flag, message in (('FIXTURE_DAEMON_FAILURE', '無法連線至 Docker Engine'),
                          ('FIXTURE_COMPOSE_FAILURE', 'Docker Compose 不可用')):
        result = run_cli(executable, dict(environment, **{flag: '1'}), expected=1)
        assert message in result.stderr
        assert not (root / 'config' / 'settings.ini').exists()
    result = run_cli(executable, environment, input_text='deploy\n1\n\n')
    assert '服務程序已啟動' in result.stdout
    config = root / 'config'
    description = yaml.safe_load((config / 'compose.yaml').read_text())
    assert description['services']['agent_1']['environment']['ENABLE_OBSTACLE_CLOUD_SIMULATOR'] == 'false'
    assert description['services']['agent_1']['environment']['AGENT_NAMESPACE'] == 'uav_1_agent'
    assert description['services']['agent_1']['image'].endswith('-runtime-arm64')
    assert '8888:8888/udp' in description['services']['agent_1']['ports']
    assert 'network_mode' not in description['services']['agent_1']
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()]
    assert any(call[0] == 'load' for call in calls)
    before = len(calls)
    run_cli(executable, environment, 'start')
    after = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    assert not any(call[0] == 'load' or 'up' in call for call in after), '重複 start 重新建置服務'
    result = run_cli(executable, environment, 'config', expected=1)
    assert '先執行 taiphoon_agent stop' in result.stderr
    settings_file = config / 'settings.ini'
    settings_file.write_text(settings_file.read_text().replace('agent_id=1', 'agent_id=2'))
    result = run_cli(executable, environment, 'start', expected=1)
    assert '先 stop' in result.stderr
    run_cli(executable, environment, 'stop')
    environment['FIXTURE_LEGACY_NAME'] = 'uav_2_agent'
    result = run_cli(executable, environment, 'start', expected=1)
    assert '其他入口' in result.stderr
    environment.pop('FIXTURE_LEGACY_NAME')
    run_cli(executable, environment, 'start')
    run_cli(executable, environment, 'stop')
    # 已保存 sim 模式遇到缺少 CLI／daemon 時保留原設定；修復後直接重新啟動。
    settings_file = config / 'settings.ini'
    settings_file.write_text(settings_file.read_text().replace('mode=deploy', 'mode=sim'))
    (package / 'sitl-entrypoint.sh').write_text('#!/bin/bash\nexit 0\n')
    saved_settings = settings_file.read_bytes()
    result = run_cli(executable, missing_environment, expected=1)
    assert '找不到 Docker CLI' in result.stderr
    assert settings_file.read_bytes() == saved_settings
    result = run_cli(executable, dict(environment, FIXTURE_DAEMON_FAILURE='1'), expected=1)
    assert '無法連線至 Docker Engine' in result.stderr
    assert settings_file.read_bytes() == saved_settings
    run_cli(executable, environment)
    assert settings_file.read_bytes() == saved_settings
    assert 'px4_1' in yaml.safe_load((config / 'compose.yaml').read_text())['services']
    assert 'px4_2' not in yaml.safe_load((config / 'compose.yaml').read_text())['services']
    run_cli(executable, environment, 'stop')
    result = run_cli(executable, environment, 'build', expected=1)
    assert '需要 dev 模式' in result.stderr
    run_cli(executable, environment, 'unknown', expected=1)
    run_cli(executable, environment, 'start', '--unexpected', expected=1)
    # Mac containerd 首次 load 及已有映像時，ID 不同但完整內容相同都須正常啟動。
    # 已存在不同內容時不得 reload、覆寫或啟動任何服務。
    run_cli(executable, dict(environment, FIXTURE_IMAGE_CONFIG='{"EngineNormalized":true}'), 'start')
    run_cli(executable, environment, 'stop')
    for flag, value, message in (
            ('FIXTURE_IMAGE_LAYERS', '["sha256:' + 'd' * 64 + '"]', '檔案層'),
            ('FIXTURE_IMAGE_ARCH', 'amd64', '映像架構'),
            ('FIXTURE_IMAGE_OS', 'windows', 'Linux 作業系統')):
        before = len((root / 'calls.jsonl').read_text().splitlines())
        result = run_cli(executable, dict(environment, **{flag: value}), 'start', expected=1)
        assert message in result.stderr
        calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
        assert not any(call[0] == 'load' or 'up' in call for call in calls)
    legacy_manifest = '\n'.join(line for line in manifest.splitlines()
                                if '_config=' not in line and '_layers=' not in line) + '\n'
    (package / 'release.manifest').write_text(legacy_manifest)
    result = run_cli(executable, environment, 'start', expected=1)
    assert '舊套件缺少' in result.stderr and '本機 ID' in result.stderr
    # 舊套件於傳統儲存方式仍正常，保留已發布格式的相容性。
    run_cli(executable, dict(environment, FIXTURE_IMAGE_ID='sha256:' + 'a' * 64), 'start')
    run_cli(executable, environment, 'stop')
    (package / 'release.manifest').write_text(manifest)
    # 套件損毀時不得 docker load／compose up，不依賴登入或網路。
    (root / 'state.json').write_text('{"loaded": false, "running": false}')
    environment['TAIPHOON_AGENT_HOME'] = str(root / 'corrupt-config')
    archive.write_bytes(b'corrupt')
    before = len((root / 'calls.jsonl').read_text().splitlines())
    result = run_cli(executable, environment, 'start', input_text='deploy\n1\n\n', expected=1)
    assert 'SHA256 不符' in result.stderr
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    assert not any(call[0] == 'load' or 'up' in call for call in calls)
    print('PASS: Docker 依賴、設定保留、跨儲存方式映像識別、異內容拒絕、舊套件、ARM64 與校驗失敗')


if __name__ == '__main__':
    main()
