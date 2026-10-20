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
    if '{{.State.Running}}' in arguments:
        print('true' if values['running'] else 'false')
    else:
        print('running healthy' if values['running'] else 'exited unhealthy')
elif arguments[0] == 'wait':
    if os.environ.get('FIXTURE_WAIT_EXIT'):
        values['running'] = False
    print('0')
elif arguments[0] == 'rm':
    assert not values['running']
    assert len(arguments) == 2 and len(arguments[1]) == 64
    values['removed'] = arguments[1]
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
        elif operation in ('stop', 'down'):
            values['running'] = False
        elif operation == 'exec':
            print('True')
elif arguments[0] == 'ps':
    print(os.environ.get('FIXTURE_LEGACY_NAME', ''))
else:
    raise AssertionError(arguments)
state.write_text(json.dumps(values))
'''

# chronyc 替身：FIXTURE_CHRONY 為 synced（預設）、unsynced、converging（第二次起才同步）或 down。
CHRONYC_FIXTURE = r'''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
root = Path(os.environ['FIXTURE_HOME'])
with (root / 'chrony_calls.jsonl').open('a') as stream:
    stream.write(json.dumps(sys.argv[1:]) + '\n')
mode = os.environ.get('FIXTURE_CHRONY', 'synced')
counter = root / 'chrony_counter'
count = int(counter.read_text()) + 1 if counter.exists() else 1
counter.write_text(str(count))
if mode == 'down':
    print('506 Cannot talk to daemon', file=sys.stderr)
    sys.exit(1)
synced = 'C0A80165,192.168.1.101,9,1760000000.0,0.000012000,0,0,0,0,0,0.003,0.0005,4.0,Normal'
unsynced = '00000000,,0,0.0,0.000000000,0,0,0,0,0,0,0,0,Not synchronised'
if mode == 'unsynced' or (mode == 'converging' and count < 2):
    print(unsynced)
else:
    print(synced)
'''


def run_cli(executable, environment, *arguments, input_text=None, expected=0):
    """執行真正的 C++ CLI，檢查回傳碼與錯誤輸出。"""
    result = subprocess.run([str(executable), *arguments], input=input_text,
                            text=True, capture_output=True, env=environment, timeout=20)
    if result.returncode != expected:
        raise AssertionError(f'{arguments}: code={result.returncode}\n{result.stdout}\n{result.stderr}')
    return result


def count_lines(path):
    """回傳替身紀錄檔行數；檔案不存在視為 0。"""
    return len(path.read_text().splitlines()) if path.exists() else 0


def compose_up_calls(root, before):
    """回傳指定位置之後的 compose up 呼叫。"""
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    return [call for call in calls if call[0] == 'compose' and 'up' in call]


def verify_time_sync_gate(executable, environment, root, no_chrony_fixture, settings_file):
    """實機 start 前的對時閘門：已同步、等待收斂、未安裝、未啟動與 off／warn 策略。"""
    result = run_cli(executable, environment, 'timesync')
    assert '時間同步正常' in result.stdout and 'stratum 9' in result.stdout
    result = run_cli(executable, dict(environment, FIXTURE_CHRONY='unsynced'), 'timesync', expected=1)
    assert '時間同步未就緒' in result.stdout and 'Not synchronised' in result.stdout

    # chronyd 未啟動：不等待，直接拒絕且不啟動任何服務。
    before = count_lines(root / 'calls.jsonl')
    result = run_cli(executable, dict(environment, FIXTURE_CHRONY='down'), 'start', expected=1)
    assert 'chronyc 無法取得狀態' in result.stderr
    assert not compose_up_calls(root, before)

    # 未安裝 chrony：Linux 實機預設 required，拒絕啟動並指出設定腳本。
    no_chrony = dict(environment, PATH=str(no_chrony_fixture) + os.pathsep + '/usr/bin:/bin')
    before = count_lines(root / 'calls.jsonl')
    result = run_cli(executable, no_chrony, 'start', expected=1)
    assert '找不到 chronyc' in result.stderr and 'time_sync=off' in result.stderr
    assert not compose_up_calls(root, before)
    result = run_cli(executable, no_chrony, 'timesync', expected=1)
    assert '找不到 chronyc' in result.stdout

    # 客戶端開機後等待時間主機：第一次未同步，第二次同步後才啟動。
    (root / 'chrony_counter').unlink(missing_ok=True)
    before = count_lines(root / 'calls.jsonl')
    result = run_cli(executable, dict(environment, FIXTURE_CHRONY='converging'), 'start')
    assert '等待主機時間同步' in result.stdout and '時間同步：已同步' in result.stdout
    assert compose_up_calls(root, before)
    run_cli(executable, environment, 'stop')

    original = settings_file.read_text()
    assert 'time_sync=auto' in original and 'time_sync_max_offset_ms=50' in original
    # 單機飛行可明確關閉；warn 只提示不阻擋。
    settings_file.write_text(original.replace('time_sync=auto', 'time_sync=off'))
    before = count_lines(root / 'calls.jsonl')
    result = run_cli(executable, no_chrony, 'start')
    assert '時間同步' not in result.stdout and compose_up_calls(root, before)
    run_cli(executable, environment, 'stop')
    settings_file.write_text(original.replace('time_sync=auto', 'time_sync=warn'))
    result = run_cli(executable, no_chrony, 'start')
    assert '警告：無法確認時間同步' in result.stdout
    run_cli(executable, environment, 'stop')
    settings_file.write_text(original.replace('time_sync=auto', 'time_sync=sometimes'))
    result = run_cli(executable, environment, 'start', expected=1)
    assert 'time_sync 必須是' in result.stderr
    settings_file.write_text(original)


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
    for name, text in (('docker', DOCKER_FIXTURE), ('chronyc', CHRONYC_FIXTURE)):
        path = fixture / name
        path.write_text(text)
        path.chmod(0o755)
    # 只有 Docker、沒有 chrony 的主機，用於驗證實機啟動前的對時閘門。
    no_chrony_fixture = root / 'fixture-no-chrony'
    no_chrony_fixture.mkdir()
    shutil.copy2(fixture / 'docker', no_chrony_fixture / 'docker')
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
    assert '時間同步：已同步：來源 192.168.1.101' in result.stdout
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
    verify_time_sync_gate(executable, environment, root, no_chrony_fixture, config / 'settings.ini')
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
    # Linux Engine 模擬維持 Taiphoon_Net 原設定，不映射主機 DDS 埠。
    engine_environment = dict(environment, FIXTURE_DOCKER_OS='Ubuntu 22.04.4 LTS')
    chrony_calls_before = count_lines(root / 'chrony_calls.jsonl')
    run_cli(executable, engine_environment)
    assert count_lines(root / 'chrony_calls.jsonl') == chrony_calls_before, '模擬模式不應檢查主機對時'
    assert settings_file.read_bytes() == saved_settings
    assert 'px4_1' in yaml.safe_load((config / 'compose.yaml').read_text())['services']
    assert 'px4_2' not in yaml.safe_load((config / 'compose.yaml').read_text())['services']
    description = yaml.safe_load((config / 'compose.yaml').read_text())
    assert description['networks']['taiphoon_net'] == {'name': 'Taiphoon_Net', 'external': True}
    assert description['services']['agent_1']['networks'] == ['taiphoon_net']
    assert description['services']['px4_1']['networks'] == ['taiphoon_net']
    assert description['services']['px4_1']['environment']['PX4_AGENT_HOST'] == 'agent_1'
    assert 'ports' not in description['services']['agent_1']
    assert 'ExternalNetworkAddress' not in (config / 'cyclonedds.xml').read_text()
    assert description['services']['agent_1']['volumes'][0]['source'] == str(config / 'cyclonedds.xml')
    run_cli(executable, environment, 'stop')
    # Docker Desktop 模擬：每台 Agent 各自單播埠段與 DDS 設定，供同機原生 Taiphoon_QGCS 連線。
    engine_settings = settings_file.read_text()
    assert 'dds_peers=\n' in engine_settings
    settings_file.write_text(engine_settings.replace('count=1', 'count=2')
                             .replace('lan_address=auto', 'lan_address=192.168.1.30')
                             .replace('dds_peers=\n', 'dds_peers=192.168.1.40\n'))
    result = run_cli(executable, environment)
    assert '--interface 192.168.1.30 --external-address 192.168.65.254' in result.stdout
    assert '--participant-index 50' in result.stdout
    # 其他電腦的 QGCS 不在 NAT 之後，啟動參數不含 --external-address。
    assert ('--interface 192.168.1.40 --no-multicast --peer 192.168.1.30 --participant-index 50'
            in result.stdout)
    description = yaml.safe_load((config / 'compose.yaml').read_text())
    for instance, ports, offset in ((1, '7410-7433', 10), (2, '7434-7457', 34)):
        agent = description['services'][f'agent_{instance}']
        assert agent['networks'] == ['taiphoon_net']
        assert agent['ports'] == [f'192.168.1.30:{ports}:{ports}/udp']
        assert agent['volumes'][0]['source'] == str(config / f'cyclonedds_{instance}.xml')
        assert agent['volumes'][0]['target'] == '/home/taiphoon/deployment/cyclonedds.xml'
        xml = (config / f'cyclonedds_{instance}.xml').read_text()
        assert '<ExternalNetworkAddress>192.168.1.30</ExternalNetworkAddress>' in xml
        assert '<Peer Address="192.168.65.254:7510"/>' in xml
        assert '<Peer Address="192.168.1.40:7510"/>' in xml
        assert f'<UnicastMetaOffset>{offset}</UnicastMetaOffset>' in xml
        assert description['services'][f'px4_{instance}']['environment']['PX4_AGENT_HOST'] == f'agent_{instance}'
    run_cli(executable, environment, 'stop')
    settings_file.write_text(engine_settings.replace('lan_address=auto', 'lan_address=192.168.1.30'))
    result = run_cli(executable, environment, 'build', expected=1)
    assert '需要 dev 模式' in result.stderr
    # dev 沿用 Release 的 copy-install，不切換成會與既有 Python build 目錄衝突的 symlink。
    workspace = root / 'source workspace'
    workspace.mkdir()
    (workspace / 'package.xml').write_text('<package/>')
    (workspace / 'CMakeLists.txt').write_text('# fixture')
    before_development = settings_file.read_text()
    settings_file.write_text(before_development.replace('mode=sim', 'mode=dev')
                            .replace('scenario=real', 'scenario=sim')
                            .replace('workspace=\n', f'workspace={workspace}\n'))
    before = len((root / 'calls.jsonl').read_text().splitlines())
    run_cli(executable, environment, 'build')
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    build = next(call[-1] for call in calls if call[0] == 'compose' and 'run' in call)
    assert 'AMENT_CMAKE_SYMLINK_INSTALL=OFF' in build and '--symlink-install' not in build
    run_cli(executable, environment)
    assert 'px4_1' in yaml.safe_load((config / 'compose.yaml').read_text())['services']
    run_cli(executable, environment, 'stop')
    settings_file.write_text(before_development)
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
    # 容器退出清理只刪除已停止的完整 ID；不能誤刪正在運作或名稱重用的容器。
    container_id = 'd' * 64
    (root / 'state.json').write_text('{"loaded": true, "running": true}')
    before = len((root / 'calls.jsonl').read_text().splitlines())
    run_cli(executable, environment, '--watch-container', container_id)
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()][before:]
    assert not any(call[0] == 'rm' for call in calls)
    run_cli(executable, dict(environment, FIXTURE_WAIT_EXIT='1'), '--watch-container', container_id)
    assert json.loads((root / 'state.json').read_text())['removed'] == container_id
    run_cli(executable, environment, '--watch-container', 'container-name', expected=1)
    calls = [json.loads(line) for line in (root / 'calls.jsonl').read_text().splitlines()]
    assert any(call[0] == 'compose' and 'down' in call for call in calls)
    assert not any('--force' in call or '-f' in call for call in calls if call[0] == 'rm')
    print('PASS: Docker 依賴、映像識別、設定保留、關閉後清理、執行中容器保護與實機對時閘門')


if __name__ == '__main__':
    main()
