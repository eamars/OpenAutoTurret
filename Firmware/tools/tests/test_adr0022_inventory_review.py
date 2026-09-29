"""Local evidence/failure tests; the shell integration uses a synthetic station tree."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest

from Firmware.tools.adr0022_inventory_review import REQUIRED, review

REPO = Path(__file__).resolve().parents[3]
COLLECTOR = REPO / 'Firmware/tools/adr0022_station_inventory.sh'


def capture(overrides=None, omitted=()):
    values = {name: (0, 'observed') for name in REQUIRED}
    values['links'] = (0, '[]')
    values.update(overrides or {})
    return ('\n'.join(f'@@BEGIN {name}\n{body}\n@@END rc={code}'
                      for name, (code, body) in sorted(values.items()) if name not in omitted)
            + '\n@@INVENTORY_COMPLETE errors=0\n').encode()


class ReviewTests(unittest.TestCase):
    def test_clean_collection_never_grants_motion_or_ownership(self):
        result = review(capture())
        self.assertEqual(result['collection_integrity'], 'PASS')
        self.assertFalse(result['motion_allowed'])
        self.assertFalse(result['process_ownership_verified'])
        self.assertEqual(result['physical_acquisition'], 'NOT_RUN')

    def test_successful_ssh_capture_does_not_hide_command_failure(self):
        result = review(capture({'processes': (1, 'error: unknown gnu long option')}))
        self.assertTrue(result['capture_completed'])
        self.assertEqual(result['collection_integrity'], 'FAIL')
        self.assertIn('COMMAND_FAILED:processes:rc=1', result['failures'])
        self.assertFalse(result['process_listing_collected'])

    def test_missing_launcher_cannot_qualify_inventory(self):
        result = review(capture(omitted=('launcher_status',)))
        self.assertIn('MISSING_SECTION:launcher_status', result['failures'])

    def test_truncated_transport_is_rejected(self):
        raw = capture().split(b'@@INVENTORY_COMPLETE')[0]
        result = review(raw)
        self.assertIn('INCOMPLETE_CAPTURE', result['failures'])
        self.assertFalse(result['capture_completed'])

    def test_duplicate_cannot_overwrite_a_failed_section(self):
        raw = capture({'processes': (1, 'failed')})
        raw = raw.replace(b'@@INVENTORY_COMPLETE', b'@@BEGIN processes\nPID\n@@END rc=0\n@@INVENTORY_COMPLETE')
        result = review(raw)
        self.assertIn('DUPLICATE_OR_EMPTY_SECTION:processes', result['failures'])
        self.assertFalse(result['process_listing_collected'])

    def test_malformed_link_data_and_trailing_content_are_rejected(self):
        self.assertIn('INVALID_LINK_DATA', review(capture({'links':(0,'{}')}))['failures'])
        self.assertIn('TRAILING_CONTENT', review(capture()+b'unknown\n')['failures'])
        self.assertIn('INVALID_UTF8', review(b'\xff')['failures'])

    def test_power_history_and_previous_stop_are_not_current_certificates(self):
        result = review(capture({'power': (0, 'throttled=0x50000'),
                                 'runtime/shutdown.result': (0, 'Stopped: STOP FAILED\ncontrold stopped cleanly')}))
        self.assertEqual(result['power']['current_flags'], 0)
        self.assertEqual(result['power']['historical_flags'], 5)
        self.assertFalse(result['power']['loaded_power_qualified'])
        self.assertEqual(result['last_recorded_stop'], 'FAILED')

    def test_cli_returns_failure_and_preserves_prior_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'capture.txt', Path(directory)/'review.json'
            source.write_bytes(capture({'processes': (1, 'failed')}))
            command = [sys.executable, '-m', 'Firmware.tools.adr0022_inventory_review',
                       '--input', str(source), '--output', str(output)]
            result = subprocess.run(command, cwd=REPO, capture_output=True)
            self.assertEqual(result.returncode, 2)
            evidence = output.read_bytes()
            self.assertFalse(json.loads(evidence)['motion_allowed'])
            result = subprocess.run(command, cwd=REPO, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_bytes(), evidence)


@unittest.skipUnless(sys.platform == 'linux', 'shell boundary rehearsal runs locally under WSL')
class CollectorIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        base = Path(self.directory.name)
        self.root = base/'synthetic-station'
        self.runtime = base/'runtime'
        self.bin = base/'commands'
        self.runtime.mkdir()
        self.bin.mkdir()
        launcher = self.root/'Firmware/scripts/run_application.sh'
        launcher.parent.mkdir(parents=True)
        launcher.write_text((REPO/'Firmware/scripts/run_application.sh').read_text(), encoding='utf-8')
        self.trusted = hashlib.sha256(launcher.read_bytes()).hexdigest()
        for args in (['init', '-q', str(self.root)],
                     ['-C', str(self.root), '-c', 'user.name=Local fixture', '-c', 'user.email=fixture@invalid',
                      '-c', 'core.hooksPath=/dev/null', 'commit', '--allow-empty', '-qm', 'Synthetic fixture']):
            subprocess.run(['git', *args], check=True, capture_output=True)
        self.stub('ip', "printf '[]\\n'")
        self.stub('systemctl', "printf 'active\\n'")
        self.stub('vcgencmd', "printf 'SYNTHETIC fixture observation\\n'")
        self.env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ['PATH'],
                        OTA_RUN_DIR=str(self.runtime))

    def stub(self, name, body):
        path = self.bin/name
        path.write_text('#!/usr/bin/env bash\n'+body+'\n')
        path.chmod(0o755)

    def run_collector(self, trusted=None):
        return subprocess.run(['bash', str(COLLECTOR), str(self.root), str(self.runtime),
                               self.trusted if trusted is None else trusted],
                              env=self.env, capture_output=True, timeout=30)

    def test_real_process_command_and_launcher_status_with_optional_paths_absent(self):
        result = self.run_collector()
        self.assertEqual(result.returncode, 0, result.stdout.decode())
        report = review(result.stdout)
        self.assertEqual(report['collection_integrity'], 'PASS', report['failures'])
        self.assertIn(b'PID', result.stdout)
        self.assertIn('STATUS_EXIT_CODE=1', report['launcher_status'])
        self.assertEqual(list(self.runtime.iterdir()), [])  # status did not create runtime files
        self.assertFalse(report['motion_allowed'])

    def test_any_failed_command_propagates_to_collector_and_review(self):
        self.stub('ps', "printf 'injected process command failure\\n' >&2\nexit 42")
        result = self.run_collector()
        self.assertEqual(result.returncode, 2)
        report = review(result.stdout)
        self.assertIn('COMMAND_FAILED:processes:rc=42', report['failures'])
        self.assertIn('COLLECTOR_REPORTED_ERRORS', report['failures'])
        self.assertFalse(report['motion_allowed'])

    def test_untrusted_launcher_is_never_executed(self):
        result = self.run_collector(trusted='0'*64)
        self.assertIn(b'@@STATUS_NOT_EXECUTED', result.stdout)
        report = review(result.stdout)
        self.assertIn('MISSING_SECTION:launcher_status', report['failures'])
        self.assertEqual(list(self.runtime.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
