"""Exercise real stream failures and the verifier's stale-output guard."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXPORTER = str(Path(sys.argv.pop(1)).resolve())
VERIFY = Path(__file__).with_name('verify.py')


class ExportFailures(unittest.TestCase):
    def run_export(self, directory):
        result = subprocess.run([EXPORTER, str(directory)], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Reference export failed:', result.stderr)

    def test_open_failure(self):
        # A directory cannot be opened as an output file, even by root.
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'reference.json').mkdir()
            self.run_export(directory)

    @unittest.skipUnless(Path('/dev/full').exists(), '/dev/full unavailable')
    def test_write_failure(self):
        # Opening succeeds, but writes (including buffered flush/close) fail.
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'reference.json').symlink_to('/dev/full')
            self.run_export(directory)

    def test_verifier_rejects_stale_success_after_export_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            subprocess.run([EXPORTER, str(directory)], check=True)
            output = directory / 'reference.json'
            stale = output.read_bytes()
            self.assertEqual(len(json.loads(stale)['runs']), 9)
            failing_exporter = directory / 'failing-exporter'
            failing_exporter.write_text('#!/bin/sh\nexit 1\n')
            failing_exporter.chmod(0o755)
            result = subprocess.run([sys.executable, str(VERIFY), str(failing_exporter),
                                     str(directory)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('CalledProcessError', result.stderr)
            self.assertEqual(output.read_bytes(), stale)
            self.assertFalse((directory / 'metrics.json').exists())


if __name__ == '__main__':
    unittest.main()
