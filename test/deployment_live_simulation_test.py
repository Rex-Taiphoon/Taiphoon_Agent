#!/usr/bin/env python3
"""
@file deployment_live_simulation_test.py
@brief 在 Docker 開發容器執行真正 Linux CLI，驗證一／四組模擬與正常停止
@author Taiphoon Agent Team
@date 2026-10-02
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def execute(arguments, **options):
    """argv 不經 shell；保留錯誤供 CI 診斷。"""
    return subprocess.run(arguments, check=True, text=True, **options)


def main():
    """CI 專用，工作區以相同絕對路徑掛載；不建構映像或發布控制命令。"""
    source = Path.cwd()
    root = Path(tempfile.mkdtemp(prefix='simulation-', dir=source / 'output'))
    # mkdtemp 預設 0700；容器與 runner 的 UID 不同，artifact 掃描仍需讀取。
    root.chmod(0o755)
    package = root / 'package'
    package.mkdir()
    executable = package / 'taiphoon_agent'
    shutil.copy2(Path(sys.argv[1]).resolve(), executable)
    shutil.copy2(source / 'docker/sitl-entrypoint.sh', package)
    version = execute([str(executable), '--version'], capture_output=True).stdout.strip()
    runtime = os.environ['TAIPHOON_SIM_RUNTIME_IMAGE']
    development = os.environ['TAIPHOON_SIM_DEV_IMAGE']
    image = json.loads(execute(['docker', 'image', 'inspect', runtime], capture_output=True).stdout)[0]
    architecture = image['Architecture']
    manifest = f'version={version}\nrepository=Rex-Taiphoon/Taiphoon_Agent\n'
    for platform in ('amd64', 'arm64'):
        for kind in ('runtime', 'dev'):
            prefix = f'{kind}_{platform}'
            manifest += f'{prefix}_sha256={"0" * 64}\n'
            manifest += f'{prefix}_id={image["Id"] if kind == "runtime" and platform == architecture else "sha256:" + "0" * 64}\n'
    (package / 'release.manifest').write_text(manifest)
    reports = []
    for count in (1, 4):
        environment = dict(os.environ, TAIPHOON_AGENT_HOME=str(root / f'config-{count}'))
        observer = f'taiphoon-simulation-observer-{count}'
        compose = ['docker', 'compose', '--project-name', 'taiphoon_agent', '--file',
                   str(root / f'config-{count}' / 'compose.yaml')]
        try:
            execute([str(executable), 'start'], env=environment, input=f'sim\n{count}\nlocal\n', timeout=180)
            container = execute(compose + ['ps', '--quiet', 'agent_1'], capture_output=True).stdout.strip()
            execute(['docker', 'run', '--name', observer, '--network', f'container:{container}',
                     '--mount', f'type=bind,source={source}/test/deployment_simulation_observer.py,target=/observer.py,readonly',
                     '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp', '--entrypoint', '/bin/bash', development,
                     '-c', 'set -e; source /opt/ros/humble/setup.bash; '
                     'source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash; '
                     'source /home/taiphoon/Taiphoon_Agent/install/setup.bash; '
                     f'python3 /observer.py {count}'], timeout=120)
            reports.append({'count': count, 'result': 'PASS'})
        except Exception:
            if (root / f'config-{count}' / 'compose.yaml').exists():
                subprocess.run(compose + ['logs', '--tail', '100'], check=False)
            raise
        finally:
            if (root / f'config-{count}' / 'compose.yaml').exists():
                execute([str(executable), 'stop'], env=environment, timeout=60)
    (source / 'output/simulation-results.json').write_text(json.dumps(reports, indent=2))
    print('PASS: 真正 CLI 的 1／4 組 PX4＋Agent、持續狀態與停止；沒有 Arm／起飛')


if __name__ == '__main__':
    main()
