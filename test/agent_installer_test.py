#!/usr/bin/env python3
"""
@file agent_installer_test.py
@brief 以 docker／systemctl／sudo 與啟動工具替身驗證 install.sh 的安裝、替換、回復與失效處理
@author Taiphoon Agent Team
@date 2026-10-27
"""
import getpass
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

REPOSITORY = 'Rex-Taiphoon/Taiphoon_Agent'
FAKE_ID = 'f' * 64

# 共用：替身把呼叫依序寫入 calls.jsonl，狀態存在 state.json（容器）與 unit.json（systemd）。
FIXTURE_COMMON = r'''
import json, os, sys
from pathlib import Path
root = Path(os.environ['FIXTURE_ROOT'])


def record(entry):
    with (root / 'calls.jsonl').open('a') as stream:
        stream.write(json.dumps(entry) + '\n')


def load(name, default):
    path = root / name
    return json.loads(path.read_text()) if path.exists() else default


def save(name, value):
    (root / name).write_text(json.dumps(value))
'''

# 啟動工具替身：以自身實際位置的 manifest 判定版本，對應原生工具以 /proc/self/exe 找 manifest。
LAUNCHER_FIXTURE = '#!/usr/bin/env python3' + FIXTURE_COMMON + r'''
package = Path(os.path.realpath(sys.argv[0])).parent
manifest = dict(line.split('=', 1) for line in (package / 'release.manifest').read_text().splitlines())
version = manifest['version']
home = Path(os.environ.get('TAIPHOON_AGENT_HOME') or Path(os.environ['HOME']) / '.config/taiphoon_agent')
command = sys.argv[1] if len(sys.argv) > 1 else 'start'
record(['launcher', version, command, str(home)])
state = load('state.json', {'running': None, 'images': [], 'legacy': {}})
if command == '--version':
    print(version)
elif command == 'init':
    mode, agent_id = sys.stdin.readline().strip(), sys.stdin.readline().strip()
    home.mkdir(parents=True, exist_ok=True)
    (home / 'settings.ini').write_text(f'mode={mode}\nagent_id={agent_id}\ntransport=udp\n')
elif command == 'start':
    if not (home / 'settings.ini').exists():
        sys.exit('settings missing')
    (home / 'compose.yaml').write_text('services: {}\n')
    if version in os.environ.get('FIXTURE_FAIL_START', '').split(','):
        sys.exit('fixture: start failed')
    if state['running'] and state['running']['version'] != version:
        sys.exit('執行中服務的模式／版本不同；請先 stop 再 start')
    state['running'] = {'version': version, 'home': str(home)}
elif command == 'stop':
    if state['running'] and state['running']['home'] == str(home):
        state['running'] = None
elif command == 'status':
    print('agent_1: fixture status')
else:
    sys.exit('unexpected command ' + command)
save('state.json', state)
'''

DOCKER_FIXTURE = '#!/usr/bin/env python3' + FIXTURE_COMMON + r'''
arguments = sys.argv[1:]
record(['docker'] + arguments)
state = load('state.json', {'running': None, 'images': [], 'legacy': {}})
if arguments[0] == 'info':
    print(os.environ.get('FIXTURE_ARCH', 'x86_64'))
elif arguments[:2] == ['compose', 'version']:
    print('Docker Compose version v2.24.0')
elif arguments[:2] == ['image', 'inspect']:
    sys.exit(0 if arguments[-1] in state['images'] else 1)
elif arguments[0] == 'load':
    state['images'].append(Path(arguments[arguments.index('--input') + 1]).read_text().strip())
elif arguments[0] == 'ps' and '--filter' in arguments:
    assert 'label=com.docker.compose.project=taiphoon_agent' in arguments
    if state['running']:
        print('f' * 64)
elif arguments[0] == 'ps':
    print('\n'.join(name for name, info in state['legacy'].items() if info['running']))
elif arguments[0] == 'inspect':
    assert arguments[-1] == 'f' * 64 and state['running']
    print(state['running']['home'] + '/compose.yaml')
elif arguments[0] == 'compose' and 'down' in arguments:
    assert state['running']['home'] == arguments[arguments.index('--project-directory') + 1]
    state['running'] = None
elif arguments[0] == 'update':
    assert arguments[1] == '--restart=no'
    state['legacy'][arguments[-1]]['restart'] = 'no'
elif arguments[0] == 'stop':
    state['legacy'][arguments[-1]]['running'] = False
else:
    raise AssertionError(arguments)
save('state.json', state)
'''

# systemctl 替身依服務檔的 Environment／ExecStart／ExecStop 實際執行啟動工具。
SYSTEMCTL_FIXTURE = '#!/usr/bin/env python3' + FIXTURE_COMMON + r'''
import shlex, subprocess
record(['systemctl'] + sys.argv[1:])
arguments = [argument for argument in sys.argv[1:] if not argument.startswith('--')]
unit = load('unit.json', {'enabled': False, 'active': False})
unit_file = Path(os.environ['TAIPHOON_SYSTEMD_UNIT_DIR']) / 'taiphoon-agent.service'


def execute(key):
    lines = unit_file.read_text().splitlines()
    environment = dict(os.environ)
    for line in lines:
        if line.startswith('Environment='):
            for item in shlex.split(line[len('Environment='):]):
                name, value = item.split('=', 1)
                environment[name] = value
    command = next(line for line in lines if line.startswith(key + '='))[len(key) + 1:]
    return subprocess.run(shlex.split(command), env=environment).returncode


command = arguments[0]
if command == 'is-enabled':
    sys.exit(0 if unit['enabled'] else 1)
if command == 'is-active':
    sys.exit(0 if unit['active'] else 3)
if command == 'enable':
    unit['enabled'] = True
elif command == 'disable':
    unit['enabled'] = False
elif command == 'start':
    code = 0 if unit['active'] else execute('ExecStart')
    unit['active'] = code == 0
    save('unit.json', unit)
    sys.exit(code)
elif command == 'stop':
    if unit['active']:
        execute('ExecStop')
    unit['active'] = False
elif command not in ('daemon-reload', 'reset-failed'):
    raise AssertionError(arguments)
save('unit.json', unit)
'''

SUDO_FIXTURE = '#!/usr/bin/env python3' + FIXTURE_COMMON + r'''
record(['sudo'] + sys.argv[1:])
if sys.argv[1:] == ['-v']:
    sys.exit(0)
os.execvp(sys.argv[1], sys.argv[1:])
'''


class Fixture:
    """單一主機的隔離環境：HOME、systemd 目錄、替身與呼叫紀錄。"""

    def __init__(self, base, name, home_name, installer):
        self.root = base / name
        self.home = self.root / home_name
        self.unit_dir = self.root / 'systemd'
        self.bin = self.root / 'bin'
        # chronyc 獨立目錄，移出 PATH 即可模擬尚未安裝 chrony 的實機。
        self.chrony_bin = self.root / 'chrony-bin'
        self.installer = installer
        for directory in (self.home, self.unit_dir, self.bin, self.chrony_bin):
            directory.mkdir(parents=True)
        for command, source in (('docker', DOCKER_FIXTURE), ('systemctl', SYSTEMCTL_FIXTURE),
                                ('sudo', SUDO_FIXTURE)):
            write_executable(self.bin / command, source)
        write_executable(self.chrony_bin / 'chronyc', '#!/bin/sh\nexit 0\n')
        self.environment = {key: value for key, value in os.environ.items()
                            if key not in ('TAIPHOON_AGENT_HOME', 'SUDO_USER')}
        self.environment.update(HOME=str(self.home), SHELL='/bin/bash', FIXTURE_ROOT=str(self.root),
                                TAIPHOON_SYSTEMD_UNIT_DIR=str(self.unit_dir),
                                PATH=f'{self.bin}:{self.chrony_bin}:{os.environ["PATH"]}')
        self.install_root = self.home / '.local/share/taiphoon_agent'
        self.config = self.home / '.config/taiphoon_agent'

    def package(self, version, corrupt=False, directory=None):
        """建立含啟動工具替身與映像 archive 的平台包；archive 內容即映像標籤。"""
        destination = directory or self.root / 'packages' / version
        (destination / 'images').mkdir(parents=True)
        write_executable(destination / 'taiphoon_agent', LAUNCHER_FIXTURE)
        manifest = f'version={version}\nrepository={REPOSITORY}\n'
        for kind in ('runtime', 'dev'):
            payload = f'taiphoon-agent:{version}-{kind}-amd64'.encode()
            manifest += f'{kind}_amd64_sha256={hashlib.sha256(payload).hexdigest()}\n'
            manifest += f'{kind}_amd64_id=sha256:{"a" * 64}\n'
            if corrupt:
                payload += b'-corrupted'
            (destination / 'images' / f'taiphoon-agent-{kind}-linux-amd64.tar.gz').write_bytes(payload)
        (destination / 'release.manifest').write_text(manifest)
        (destination / 'sitl-entrypoint.sh').write_text('#!/bin/sh\n')
        (destination / 'README.md').write_text('# fixture\n')
        shutil.copy2(self.installer, destination / 'install.sh')
        return destination

    def run(self, script, *arguments, expected=0, extra=None):
        """非互動執行安裝腳本（stdin 不是終端機），檢查結束碼。"""
        environment = dict(self.environment, **(extra or {}))
        result = subprocess.run(['bash', str(script), *arguments], env=environment, input='',
                                capture_output=True, text=True, timeout=60)
        if result.returncode != expected:
            raise AssertionError(f'{arguments}: code={result.returncode}\n{result.stdout}\n{result.stderr}')
        return result

    def calls(self):
        path = self.root / 'calls.jsonl'
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

    def state(self):
        return json.loads((self.root / 'state.json').read_text())

    def unit(self):
        return json.loads((self.root / 'unit.json').read_text())

    def link(self, name):
        return os.readlink(self.install_root / name)

    def running_version(self):
        running = self.state()['running']
        return running and running['version']


def write_executable(path, text):
    path.write_text(text)
    path.chmod(0o755)


def index_of(calls, predicate, start=0):
    """回傳第一個符合條件的呼叫位置，用於檢查先後順序。"""
    return next(index for index, call in enumerate(calls) if index >= start and predicate(call))


def verify_rejections(fixture):
    """參數錯誤、非互動未確認與套件不完整都不變更系統。"""
    package = fixture.package('v1.3.0-rc.1')
    assert '用法' in fixture.run(package / 'install.sh', '--help').stdout
    fixture.run(package / 'install.sh', '--unknown', expected=2)
    fixture.run(package / 'install.sh', '--agent-id', '0', expected=2)
    fixture.run(package / 'install.sh', '--rollback', '--no-start', expected=2)
    result = fixture.run(package / 'install.sh', '--agent-id', '3', expected=1)
    assert '--yes' in result.stderr and '安裝計畫' in result.stdout
    assert not fixture.install_root.exists() and not fixture.config.exists(), '未確認前不應建立任何檔案'
    broken = fixture.package('v1.3.0-rc.0')
    (broken / 'sitl-entrypoint.sh').unlink()
    assert 'sitl-entrypoint.sh' in fixture.run(broken / 'install.sh', '--yes', expected=1).stderr
    return package


def verify_fresh_install(fixture, package):
    """首次安裝：建立設定、systemd 開機自啟並以 systemd 啟動。"""
    fixture.run(package / 'install.sh', '--yes', '--agent-id', '3', '--autostart')
    assert fixture.link('current') == 'releases/v1.3.0-rc.1'
    assert not (fixture.install_root / 'previous').exists()
    settings = (fixture.config / 'settings.ini').read_text()
    assert 'mode=deploy' in settings and 'agent_id=3' in settings
    unit = (fixture.unit_dir / 'taiphoon-agent.service').read_text()
    assert f'User={getpass.getuser()}' in unit
    assert f'ExecStart="{fixture.install_root}/current/taiphoon_agent" start' in unit
    assert f'Environment="TAIPHOON_AGENT_HOME={fixture.config}"' in unit
    assert fixture.unit() == {'enabled': True, 'active': True}
    assert fixture.running_version() == 'v1.3.0-rc.1'
    assert fixture.state()['running']['home'] == str(fixture.config)
    assert sum(call[:2] == ['docker', 'load'] for call in fixture.calls()) == 1
    wrapper = subprocess.run(['bash', str(fixture.home / '.local/bin/taiphoon_agent'), '--version'],
                             env=fixture.environment, capture_output=True, text=True, check=True)
    assert wrapper.stdout.strip() == 'v1.3.0-rc.1'
    assert (fixture.install_root / 'install.sh').is_file()
    assert (fixture.home / '.bashrc').read_text().count('export PATH=') == 1

    before = len(fixture.calls())
    fixture.run(package / 'install.sh', '--yes')
    calls = fixture.calls()[before:]
    assert not any(call[:2] == ['docker', 'load'] for call in calls), '已有映像不應重新載入'
    assert ['systemctl', 'stop', 'taiphoon-agent.service'] in calls
    assert fixture.running_version() == 'v1.3.0-rc.1' and fixture.unit()['active']
    assert not (fixture.install_root / 'previous').exists()
    assert (fixture.home / '.bashrc').read_text().count('export PATH=') == 1, '重複安裝不應重複加入 PATH'
    result = fixture.run(package / 'install.sh', '--yes', '--agent-id', '9', expected=1)
    assert '--agent-id 只用於首次安裝' in result.stderr


def verify_upgrades(fixture):
    """升級：先載入映像再停舊版，保留上一版，只留兩個安裝版本。"""
    before = len(fixture.calls())
    fixture.run(fixture.package('v1.3.0-rc.2') / 'install.sh', '--yes')
    calls = fixture.calls()[before:]
    loaded = index_of(calls, lambda call: call[:2] == ['docker', 'load'])
    stopped = index_of(calls, lambda call: call == ['systemctl', 'stop', 'taiphoon-agent.service'])
    started = index_of(calls, lambda call: call[0] == 'launcher' and call[1:3] == ['v1.3.0-rc.2', 'start'])
    assert loaded < stopped < started, '應在舊版執行中預先載入映像'
    assert fixture.link('current') == 'releases/v1.3.0-rc.2'
    assert Path(fixture.link('previous')).name == 'v1.3.0-rc.1'
    assert fixture.running_version() == 'v1.3.0-rc.2' and fixture.unit()['active']

    fixture.run(fixture.package('v1.3.0-rc.3') / 'install.sh', '--yes')
    releases = sorted(path.name for path in (fixture.install_root / 'releases').iterdir())
    assert releases == ['v1.3.0-rc.2', 'v1.3.0-rc.3'], releases
    assert Path(fixture.link('previous')).name == 'v1.3.0-rc.2'
    assert fixture.running_version() == 'v1.3.0-rc.3'

    fixture.run(fixture.install_root / 'install.sh', '--rollback', '--yes')
    assert fixture.link('current').endswith('v1.3.0-rc.2') and fixture.link('previous').endswith('v1.3.0-rc.3')
    assert fixture.running_version() == 'v1.3.0-rc.2' and fixture.unit()['active']


def verify_failures(fixture):
    """校驗、架構不符或缺少 chrony 時不動既有安裝；新版啟動失敗時自動切回並恢復執行。"""
    snapshot = (fixture.link('current'), fixture.link('previous'), fixture.running_version())
    before = len(fixture.calls())
    result = fixture.run(fixture.package('v1.3.0-rc.4', corrupt=True) / 'install.sh', '--yes', expected=1)
    assert 'SHA256' in result.stderr
    result = fixture.run(fixture.package('v1.3.0-rc.5') / 'install.sh', '--yes', expected=1,
                         extra={'FIXTURE_ARCH': 'aarch64'})
    assert 'amd64' in result.stderr and 'arm64' in result.stderr
    if shutil.which('chronyc') is None:
        # 實機預設 time_sync=auto 要求對時；沒有 chrony 時新版必定拒絕啟動，應在停止舊版前就停止。
        result = fixture.run(fixture.package('v1.3.0-rc.7') / 'install.sh', '--yes', expected=1,
                             extra={'PATH': f'{fixture.bin}:{os.environ["PATH"]}'})
        assert 'chrony' in result.stderr and 'time_sync=off' in result.stderr
    assert not any(call[:2] == ['systemctl', 'stop'] for call in fixture.calls()[before:])
    assert (fixture.link('current'), fixture.link('previous'), fixture.running_version()) == snapshot
    leftovers = [path.name for path in (fixture.install_root / 'releases').iterdir()
                 if path.name not in ('v1.3.0-rc.2', 'v1.3.0-rc.3')]
    assert not (fixture.install_root / 'releases').joinpath('v1.3.0-rc.7').exists()
    assert not leftovers and not (fixture.install_root / '.install.lock').exists(), leftovers

    result = fixture.run(fixture.package('v1.3.0-rc.6') / 'install.sh', '--yes', expected=1,
                         extra={'FIXTURE_FAIL_START': 'v1.3.0-rc.6'})
    assert '已切回 v1.3.0-rc.2 並恢復執行' in result.stderr
    assert fixture.link('current').endswith('v1.3.0-rc.2') and fixture.link('previous') == 'releases/v1.3.0-rc.6'
    assert fixture.running_version() == 'v1.3.0-rc.2' and fixture.unit()['active']
    assert (fixture.install_root / 'releases/v1.3.0-rc.3').is_dir(), '失敗時不應清除舊安裝版本'


def verify_manual_systemd_replacement(fixture):
    """替換樹莓派上手動建立、指向解壓縮套件與獨立設定目錄的 systemd 部署。"""
    deployment = fixture.home / 'Taiphoon_Agent_release_test/v1.3.0-rc.0'
    old_package = fixture.package('v1.3.0-rc.0', directory=deployment / 'package')
    old_config = deployment / 'config'
    old_config.mkdir()
    (old_config / 'settings.ini').write_text('mode=deploy\nagent_id=1\nobstacle_source=none\n')
    unit_file = fixture.unit_dir / 'taiphoon-agent.service'
    unit_file.write_text(f'[Service]\nType=oneshot\nRemainAfterExit=yes\nUser={getpass.getuser()}\n'
                         f'WorkingDirectory={old_package}\nEnvironment=TAIPHOON_AGENT_HOME={old_config}\n'
                         f'ExecStart={old_package}/taiphoon_agent start\n'
                         f'ExecStop={old_package}/taiphoon_agent stop\n')
    (fixture.root / 'unit.json').write_text(json.dumps({'enabled': True, 'active': False}))
    subprocess.run([str(fixture.bin / 'systemctl'), 'start', 'taiphoon-agent.service'],
                   env=fixture.environment, check=True)
    assert fixture.running_version() == 'v1.3.0-rc.0'

    result = fixture.run(fixture.package('v1.3.0-rc.1') / 'install.sh', '--yes')
    assert '維持啟用' in result.stdout
    assert (fixture.config / 'settings.ini').read_text() == (old_config / 'settings.ini').read_text()
    assert (old_package / 'taiphoon_agent').is_file(), '不應刪除安裝目錄以外的舊套件'
    assert Path(fixture.link('previous')).resolve() == old_package.resolve()
    assert f'ExecStart="{fixture.install_root}/current/taiphoon_agent" start' in unit_file.read_text()
    backups = list((fixture.install_root / 'backup').iterdir())
    assert len(backups) == 1 and str(old_package) in backups[0].read_text()
    assert fixture.state()['running'] == {'version': 'v1.3.0-rc.1', 'home': str(fixture.config)}
    assert fixture.unit() == {'enabled': True, 'active': True}

    fixture.run(fixture.install_root / 'install.sh', '--rollback', '--yes')
    assert fixture.state()['running'] == {'version': 'v1.3.0-rc.0', 'home': str(fixture.config)}


def verify_flat_install_replacement(fixture):
    """替換舊版 install.sh 的平鋪安裝，並停止會阻擋啟動的同編號舊入口容器。"""
    fixture.package('v1.3.0-rc.0', directory=fixture.install_root)
    (fixture.install_root / 'install.sh').unlink()
    fixture.config.mkdir(parents=True)
    (fixture.config / 'settings.ini').write_text('mode=deploy\nagent_id=2\n')
    (fixture.root / 'state.json').write_text(json.dumps({
        'running': None, 'images': [],
        'legacy': {'fmu2_agent': {'running': True, 'restart': 'always'},
                   'uav_7_agent': {'running': True, 'restart': 'always'}}}))

    result = fixture.run(fixture.package('v1.3.0-rc.1') / 'install.sh', '--yes')
    assert 'fmu2_agent' in result.stdout and 'uav_7_agent' not in result.stdout
    legacy = fixture.state()['legacy']
    assert legacy['fmu2_agent'] == {'running': False, 'restart': 'no'}
    assert legacy['uav_7_agent'] == {'running': True, 'restart': 'always'}, '不應停止其他編號的容器'
    assert not (fixture.install_root / 'taiphoon_agent').exists()
    assert (fixture.install_root / 'releases/v1.3.0-rc.0/images').is_dir()
    assert fixture.link('previous').endswith('releases/v1.3.0-rc.0')
    assert fixture.link('current') == 'releases/v1.3.0-rc.1'
    assert not (fixture.unit_dir / 'taiphoon-agent.service').exists(), '未要求時不應建立 systemd 服務'
    assert fixture.state()['running'] == {'version': 'v1.3.0-rc.1', 'home': str(fixture.config)}


def main():
    """各情境使用獨立 HOME；不呼叫真正的 Docker、systemd 或 sudo。"""
    source = Path(__file__).resolve().parents[1]
    installer = source / 'Tools/agent_launcher/install.sh'
    base = Path(tempfile.mkdtemp(prefix='taiphoon-installer-test-'))
    try:
        fixture = Fixture(base, 'fleet', 'home with 中文 spaces', installer)
        package = verify_rejections(fixture)
        verify_fresh_install(fixture, package)
        verify_upgrades(fixture)
        verify_failures(fixture)
        verify_manual_systemd_replacement(Fixture(base, 'raspberry-pi', 'taiphoon', installer))
        verify_flat_install_replacement(Fixture(base, 'flat', 'pi', installer))
    finally:
        shutil.rmtree(base)
    print('PASS: 首次安裝、重複安裝、升級保留兩版、rollback、校驗／架構／對時預檢失敗不變更、'
          '啟動失敗自動切回、替換手動 systemd 部署與舊版平鋪安裝')


if __name__ == '__main__':
    main()
