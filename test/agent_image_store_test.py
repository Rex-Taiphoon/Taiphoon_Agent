#!/usr/bin/env python3
"""
@file agent_image_store_test.py
@brief 在 Docker 開發容器核對已發布映像 archive 與 containerd 載入後的內容
@author Taiphoon Agent Team
@date 2026-10-02
"""
import hashlib
import json
from pathlib import Path
import re
import sys
import tarfile


def main():
    """只讀取 Release 與 Docker inspect 觀測資料，不啟動 Agent 或刪除資料。"""
    package = Path(sys.argv[1])
    observation = json.loads(Path(sys.argv[2]).read_text())
    metadata = dict(line.split('=', 1) for line in (package / 'release.manifest').read_text().splitlines())
    expected_id = metadata['dev_amd64_id']
    digest = expected_id.removeprefix('sha256:')
    image_config = None
    with tarfile.open(package / 'images/taiphoon-agent-dev-linux-amd64.tar.gz', 'r|gz') as archive:
        for member in archive:
            if member.name in (digest + '.json', 'blobs/sha256/' + digest):
                payload = archive.extractfile(member).read()
                assert hashlib.sha256(payload).hexdigest() == digest, '原始 config digest 不符'
                image_config = json.loads(payload)
                break
    assert image_config is not None, '映像 archive 缺少已發布 config'
    source = Path(__file__).resolve().parents[1]
    template = (source / 'Tools/agent_launcher/image-config.format').read_text()
    fields = re.findall(r'\{\{json \.Config\.([A-Za-z]+)\}\}', template)
    string_fields = {'User', 'WorkingDir', 'StopSignal'}
    expected_config = {field: image_config['config'].get(field, '' if field in string_fields else None)
                       for field in fields}
    assert observation['config'] == expected_config, '執行設定在載入前後改變'
    assert observation['layers'] == image_config['rootfs']['diff_ids'], '檔案層在載入前後改變'
    assert observation['architecture'] == image_config['architecture'] == 'amd64'
    assert observation['os'] == image_config['os'] == 'linux'
    assert observation['id'] != expected_id, '此測試需使用回傳不同 ID 的 containerd 儲存方式'
    print('PASS: 同一 Release archive 於 containerd 有不同 ID，但執行設定、全部檔案層及平台相同')
    print(f'CI config ID: {expected_id}; containerd ID: {observation["id"]}')


if __name__ == '__main__':
    main()
