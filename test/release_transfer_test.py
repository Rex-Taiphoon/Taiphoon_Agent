#!/usr/bin/env python3
"""驗證 Release 草稿傳輸的重跑及公開版本保護，不連接 GitHub。

@file release_transfer_test.py
@brief 產物集合、發布 Gate 與 source commit 回歸
@author Taiphoon Agent Team
@date 2026-10-06
"""
import importlib.util
import os
import locale
import sys
import tempfile
import subprocess
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'release_transfer', Path(__file__).parents[1] / 'Tools/agent_launcher/release_transfer.py')
transfer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transfer)


class ReleaseTransferTest(unittest.TestCase):
    def test_failed_upload_restores_readonly_build_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'taiphoon_agent'
            path.write_bytes(b'compiled-output')
            path.chmod(0o444)
            with patch.object(transfer, 'draft', return_value={'assets': []}), \
                    patch.object(transfer, 'execute', side_effect=subprocess.CalledProcessError(1, 'gh')):
                with self.assertRaises(subprocess.CalledProcessError):
                    transfer.upload_file('v1.3.0-rc.4', path, 'stage-fixture')
            self.assertEqual(path.read_bytes(), b'compiled-output')
            self.assertEqual([item.name for item in Path(directory).iterdir()], ['taiphoon_agent'])
            path.chmod(0o644)

    def test_api_output_decodes_utf8_under_windows_legacy_locale(self):
        with patch.object(locale, 'getpreferredencoding', return_value='cp1252'):
            result = transfer.execute([sys.executable, '-c',
                                       'import sys; sys.stdout.buffer.write(bytes.fromhex("e6b8ace8a9a6"))'])
        self.assertEqual(result, '測試')

    def release_api(self, release):
        def response(path, *args):
            if path.startswith('releases?'):
                return [{'id': 1, 'tag_name': 'v1.3.0-rc.4'}]
            return release
        return response

    def assets(self, category, attempt):
        return [{'name': f'stage-123-{attempt}-{category}--{filename}', 'digest': 'sha256:fixture', 'size': 1}
                for filename in transfer.expected_files(category)]

    def test_failed_job_rerun_uses_complete_earlier_platform_attempts(self):
        assets = [asset for category in transfer.CATEGORIES for asset in self.assets(category, 1)]
        assets += self.assets('native-windows-amd64', 2)
        assets += self.assets('linux-arm64', 3)[:1]
        assets += [{'name': 'stage-999-9-native-windows-amd64--taiphoon_agent.exe'}]
        selected = transfer.select_assets(assets, '123')
        self.assertIn('-2-', selected['native-windows-amd64']['taiphoon_agent.exe']['name'])
        self.assertTrue(all('-1-' in asset['name'] for asset in selected['linux-arm64'].values()))

    def test_missing_platform_prevents_packaging(self):
        assets = [asset for category in transfer.CATEGORIES[:-1] for asset in self.assets(category, 1)]
        with self.assertRaisesRegex(RuntimeError, '缺少完整建置產物'):
            transfer.select_assets(assets, '123')

    def test_published_release_cannot_be_modified(self):
        with patch.object(transfer, 'api', side_effect=self.release_api({'draft': False})), \
                patch.dict(os.environ, RELEASE_SOURCE_COMMIT='a' * 40):
            with self.assertRaisesRegex(RuntimeError, '版本已發布'):
                transfer.draft('v1.3.0-rc.4')

    def test_wrong_source_commit_prevents_transfer(self):
        with patch.object(transfer, 'api', side_effect=self.release_api(
                {'draft': True, 'target_commitish': 'b' * 40})):
            with self.assertRaisesRegex(RuntimeError, 'source commit 不符'):
                transfer.draft('v1.3.0-rc.4', 'a' * 40)

    def test_incomplete_packages_cannot_publish_or_delete_assets(self):
        release = {'draft': True, 'id': 1, 'target_commitish': 'a' * 40, 'assets': []}
        with patch.object(transfer, 'api', side_effect=self.release_api(release)) as remote, \
                patch.dict(os.environ, RELEASE_SOURCE_COMMIT='a' * 40):
            with self.assertRaisesRegex(RuntimeError, '尚未完整校驗'):
                transfer.publish('v1.3.0-rc.4', Path('unused.md'))
            self.assertEqual(remote.call_count, 2)

    def test_draft_lookup_uses_paginated_list_and_numeric_id(self):
        unrelated = [{'id': index, 'tag_name': 'v0.0.0'} for index in range(100)]
        release = {'id': 404456736, 'draft': True, 'target_commitish': 'a' * 40}
        with patch.object(transfer, 'api', side_effect=[unrelated,
                [{'id': 404456736, 'tag_name': 'v1.3.0-rc.4'}], release]) as remote:
            self.assertEqual(transfer.draft('v1.3.0-rc.4', 'a' * 40), release)
            self.assertEqual([call.args[0] for call in remote.call_args_list],
                             ['releases?per_page=100&page=1', 'releases?per_page=100&page=2',
                              'releases/404456736'])

    def test_duplicate_release_prevents_transfer(self):
        releases = [{'id': index, 'tag_name': 'v1.3.0-rc.4'} for index in (1, 2)]
        with patch.object(transfer, 'api', return_value=releases):
            with self.assertRaisesRegex(RuntimeError, '重複版本'):
                transfer.draft('v1.3.0-rc.4', 'a' * 40)


if __name__ == '__main__':
    unittest.main(verbosity=2)
