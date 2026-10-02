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
    print('linux' if arguments[-1] == '{{.OSType}}' else os.environ.get('FIXTURE_ARCH', 'aarch64'))
elif arguments[0] == 'context':
    print('unix:///var/run/docker.sock')
elif arguments[0] == 'image':
    if not values['loaded']:
        sys.exit(1)
    print(os.environ.get('FIXTURE_IMAGE_ARCH', 'arm64') if '{{.Architecture}}' in arguments else 'sha256:' + 'a' * 64)
elif arguments[0] == 'load':
    values['loaded'] = True
elif arguments[0] == 'inspect':
    print('running healthy' if values['running'] else 'exited unhealthy')
elif arguments[0] == 'compose':
    if '--file' not in arguments:
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
                       PATH=str(fixture) + os.pathsep + os.environ['PATH'])
    result = run_cli(executable, environment, input_text='deploy\n1\nudp\n')
    assert '服務程序已啟動' in result.stdout
    config = root / 'config'
    description = yaml.safe_load((config / 'compose.yaml').read_text())
    assert description['services']['agent_1']['environment']['ENABLE_OBSTACLE_CLOUD_SIMULATOR'] == 'false'
    assert description['services']['agent_1']['environment']['AGENT_NAMESPACE'] == 'uav_1_agent'
    assert description['services']['agent_1']['image'].endswith('-runtime-arm64')
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
    result = run_cli(executable, environment, 'build', expected=1)
    assert '需要 dev 模式' in result.stderr
    run_cli(executable, environment, 'unknown', expected=1)
    run_cli(executable, environment, 'start', '--unexpected', expected=1)
    # 套件損毀時不得 docker load／compose up，不依賴登入或網路。
    (root / 'state.json').write_text('{"loaded": false, "running": false}')
    environment['TAIPHOON_AGENT_HOME'] = str(root / 'corrupt-config')
    archive.write_bytes(b'corrupt')
    before = len((root / 'calls.jsonl').read_text().splitlines())
    result = run_cli(executable, environment, 'start', input_text='deploy\n1\nudp\n', expected=1)
    assert 'SHA256 不符' in result.stderr
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    assert not any(call[0] == 'load' or 'up' in call for call in calls)
    print('PASS: 首次零參數啟動、ARM64 選擇、離線載入、重複啟動、設定 Gate、停止與校驗失敗')


if __name__ == '__main__':
    main()
