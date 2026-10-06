#!/usr/bin/env python3
"""以未公開 Release 草稿傳遞已驗證產物，避免占用 Actions Artifact 額度。

@file release_transfer.py
@brief 跨 runner 傳輸、重跑校驗及五平台發布 Gate
@author Taiphoon Agent Team
@date 2026-10-06
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


CATEGORIES = ('linux-amd64', 'linux-arm64', 'native-windows-amd64', 'native-macos-universal')
PLATFORMS = ('linux-amd64', 'linux-arm64', 'windows-amd64', 'macos-amd64', 'macos-arm64')


def execute(arguments, payload=None):
    """以 argv 呼叫 gh；憑證只由 GH_TOKEN 環境提供，不寫入檔案。"""
    return subprocess.run(arguments, input=payload, text=True, capture_output=True, check=True).stdout


def api(path, method='GET', payload=None):
    """呼叫此 repository 的 API，沒有跨站 HTTP 轉送憑證。"""
    repository = os.environ['GH_REPO']
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', repository):
        raise ValueError('無效 repository')
    arguments = ['gh', 'api', f'repos/{repository}/{path}', '--method', method]
    body = None
    if payload is not None:
        arguments.extend(['--input', '-'])
        body = json.dumps(payload)
    result = execute(arguments, body)
    return json.loads(result) if result.strip() else None


def validate_version(version):
    """只接受既有發布流程的版本格式。"""
    if not re.fullmatch(r'v[0-9]+\.[0-9]+\.[0-9]+(?:[.-][A-Za-z0-9.-]+)?', version):
        raise ValueError('無效版本標籤')


def draft(version, source_commit=None):
    """禁止修改已發布版本，並固定原始碼提交。"""
    validate_version(version)
    # REST 的 tags endpoint 只找得到公開版本；草稿必須由可見版本清單查找。
    matches = []
    page = 1
    while True:
        releases = api(f'releases?per_page=100&page={page}')
        matches.extend(release for release in releases if release['tag_name'] == version)
        if len(releases) < 100:
            break
        page += 1
    if not matches:
        raise LookupError(f'尚未建立版本草稿：{version}')
    if len(matches) != 1:
        raise RuntimeError('同一標籤有重複版本，停止傳輸')
    release = api(f'releases/{matches[0]["id"]}')
    expected = source_commit or os.environ['RELEASE_SOURCE_COMMIT']
    if not re.fullmatch(r'[0-9a-f]{40}', expected):
        raise ValueError('缺少完整 source commit')
    if not release['draft']:
        raise RuntimeError('版本已發布；禁止修改既有資產')
    if release['target_commitish'] != expected:
        raise RuntimeError('草稿的 source commit 不符')
    return release


def create(version, source_commit):
    """標籤必須已存在；此操作不建立或移動 Git tag。"""
    validate_version(version)
    tag = api(f'git/ref/tags/{version}')['object']
    for _ in range(5):
        if tag['type'] != 'tag':
            break
        tag = api(f'git/tags/{tag["sha"]}')['object']
    if tag['type'] != 'commit' or tag['sha'] != source_commit:
        raise RuntimeError('既有標籤與 source commit 不符')
    try:
        draft(version, source_commit)
    except LookupError:
        api('releases', 'POST', {'tag_name': version, 'target_commitish': source_commit,
                               'name': f'Taiphoon Agent {version}', 'draft': True,
                               'prerelease': '-' in version})
    print(f'草稿已就緒：{version}，source={source_commit}')


def expected_files(category):
    """只傳送打包所需的映像、內部 manifest 與原生入口。"""
    if category not in CATEGORIES:
        raise ValueError('無效平台分類')
    if category.startswith('linux-'):
        architecture = category.split('-')[1]
        return ('taiphoon_agent', 'images.manifest',
                f'taiphoon-agent-runtime-linux-{architecture}.tar.gz',
                f'taiphoon-agent-dev-linux-{architecture}.tar.gz')
    return ('taiphoon_agent.exe' if category == 'native-windows-amd64' else 'taiphoon_agent',)


def checksum(path):
    """串流校驗大型映像，避免將檔案全部放入記憶體。"""
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return 'sha256:' + result.hexdigest()


def upload_file(version, path, name, replace_draft=False):
    """硬連結保留原檔名；只允許更新本次未公開草稿的完整平台包。"""
    release = draft(version)
    digest = checksum(path)
    existing = next((asset for asset in release['assets'] if asset['name'] == name), None)
    if existing is not None:
        if existing.get('digest') == digest:
            return
        if not replace_draft:
            raise RuntimeError(f'重複 staging 資產內容不符：{name}')
        api(f'releases/assets/{existing["id"]}', 'DELETE')
    with tempfile.TemporaryDirectory(prefix='release-upload-', dir=path.parent) as directory:
        staged = Path(directory) / name
        os.link(path, staged)
        execute(['gh', 'release', 'upload', version, str(staged)])
    uploaded = next(asset for asset in draft(version)['assets'] if asset['name'] == name)
    if uploaded.get('digest') != digest or uploaded['size'] != path.stat().st_size:
        raise RuntimeError(f'上傳校驗失敗：{name}')
    print(f'已校驗上傳：{name}')


def upload(version, category, directory):
    """各 runner 分開命名，重跑使用新的 attempt，不覆寫舊 stage。"""
    run_id, attempt = os.environ['GITHUB_RUN_ID'], os.environ['GITHUB_RUN_ATTEMPT']
    if not run_id.isdigit() or not attempt.isdigit():
        raise ValueError('無效 workflow run／attempt')
    for filename in expected_files(category):
        path = directory / filename
        if not path.is_file() or path.stat().st_size <= 0:
            raise RuntimeError(f'缺少建置檔案：{path}')
        upload_file(version, path, f'stage-{run_id}-{attempt}-{category}--{filename}')


def select_assets(assets, run_id):
    """失敗工作重跑時，可使用其他成功工作較早 attempt 的完整產物。"""
    selected = {}
    for category in CATEGORIES:
        groups = {}
        pattern = re.compile(rf'^stage-{re.escape(run_id)}-(\d+)-{re.escape(category)}--(.+)$')
        for asset in assets:
            match = pattern.fullmatch(asset['name'])
            if match and match[2] in expected_files(category):
                groups.setdefault(int(match[1]), {})[match[2]] = asset
        complete = [attempt for attempt, files in groups.items()
                    if set(files) == set(expected_files(category))]
        if not complete:
            raise RuntimeError(f'缺少完整建置產物：{category}')
        selected[category] = groups[max(complete)]
    return selected


def download(version, directory):
    """只下載目前 workflow 的完整平台集合，逐檔核對 GitHub digest。"""
    assets = select_assets(draft(version)['assets'], os.environ['GITHUB_RUN_ID'])
    for category, files in assets.items():
        destination = directory / category
        destination.mkdir(parents=True, exist_ok=True)
        for filename, asset in files.items():
            execute(['gh', 'release', 'download', version, '--pattern', asset['name'],
                     '--dir', str(destination)])
            staged = destination / asset['name']
            if not asset.get('digest') or checksum(staged) != asset['digest']:
                raise RuntimeError(f'下載校驗失敗：{asset["name"]}')
            staged.rename(destination / filename)
    print('四組建置產物下載與校驗完成')


def package_names(version):
    """沿用五個完整下載包的發布契約。"""
    return {f'taiphoon_agent-{version}-{platform}.' +
            ('zip' if platform == 'windows-amd64' else 'tar.gz') for platform in PLATFORMS}


def packages(version, directory):
    """五個包完整且已上傳校驗後，才允許下一個 publish 工作。"""
    names = package_names(version)
    if {path.name for path in directory.iterdir() if path.is_file()} != names:
        raise RuntimeError('完整套件數量或命名不符')
    for name in sorted(names):
        path = directory / name
        if not 0 < path.stat().st_size < 2 * 1024 * 1024 * 1024:
            raise RuntimeError('完整包大小不符 Release 限制')
        upload_file(version, path, name, replace_draft=True)


def publish(version, notes):
    """只公開完整五平台包；刪除草稿中本流程的 stage 後再發布。"""
    release = draft(version)
    names = package_names(version)
    final_assets = {asset['name']: asset for asset in release['assets'] if asset['name'] in names}
    if set(final_assets) != names or any(not asset.get('digest') or asset['size'] <= 0
                                       for asset in final_assets.values()):
        raise RuntimeError('五平台包尚未完整校驗')
    for asset in release['assets']:
        if re.fullmatch(r'stage-\d+-\d+-(?:linux-amd64|linux-arm64|native-windows-amd64|native-macos-universal)--.+',
                        asset['name']):
            api(f'releases/assets/{asset["id"]}', 'DELETE')
    if {asset['name'] for asset in draft(version)['assets']} != names:
        raise RuntimeError('草稿仍含非平台套件資產，停止發布')
    source = os.environ['RELEASE_SOURCE_COMMIT']
    repository = os.environ['GH_REPO']
    body = (f'版本：`{version}`\n原始碼：[{source[:12]}](https://github.com/{repository}/tree/{source})\n\n'
            + notes.read_text(encoding='utf-8')
            + '\n\n建置產物直接經 Release 草稿傳遞，未使用 Actions Artifacts。\n')
    result = api(f'releases/{release["id"]}', 'PATCH', {'draft': False, 'prerelease': '-' in version,
                 'name': f'Taiphoon Agent {version}', 'body': body})
    print(result['html_url'])


def main():
    """所有 remote mutation 均由明確子命令執行。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=('create', 'upload', 'download', 'packages', 'publish'))
    parser.add_argument('version')
    parser.add_argument('arguments', nargs='*')
    args = parser.parse_args()
    validate_version(args.version)
    if args.command == 'create':
        create(args.version, args.arguments[0])
    elif args.command == 'upload':
        upload(args.version, args.arguments[0], Path(args.arguments[1]))
    elif args.command == 'download':
        download(args.version, Path(args.arguments[0]))
    elif args.command == 'packages':
        packages(args.version, Path(args.arguments[0]))
    else:
        publish(args.version, Path(args.arguments[0]))


if __name__ == '__main__':
    main()
