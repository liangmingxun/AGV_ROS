import tempfile
import unittest
from pathlib import Path

import yaml

from multi_agv_analysis.io_utils import load_yaml, sha256_file


class SnapshotYamlTests(unittest.TestCase):
    def load_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot.yaml"
            path.write_text(text, encoding="utf-8")
            before = sha256_file(path)
            result = load_yaml(path)
            self.assertEqual(before, sha256_file(path))
            return result

    def test_legacy_datetime_is_inert_string(self):
        result = self.load_text(
            "clock: !!python/object:xmlrpc.client.DateTime\n"
            "  value: 20260913T13:52:17\nscale: 0.92\n")
        self.assertEqual(result, {"clock": "20260913T13:52:17", "scale": 0.92})

    def test_plain_string(self):
        self.assertEqual(self.load_text('clock: "2026-09-13T13:52:17+08:00"\n'),
                         {"clock": "2026-09-13T13:52:17+08:00"})

    def test_other_python_tags_still_rejected(self):
        with self.assertRaises(yaml.YAMLError):
            self.load_text('clock: !!python/object/apply:builtins.str [42]\n')

    def test_malformed_datetime_rejected(self):
        for body in ('value: invalid', 'value: 20260913T13:52:17\n  extra: 1'):
            with self.subTest(body=body), self.assertRaises(yaml.YAMLError):
                self.load_text('clock: !!python/object:xmlrpc.client.DateTime\n  ' + body + '\n')


if __name__ == "__main__":
    unittest.main()
