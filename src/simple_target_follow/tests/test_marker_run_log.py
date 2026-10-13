"""Check that marker mission logs are durable and separate per node run."""

import json
import os
import sys
import tempfile
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.marker_run_log import MarkerRunLog  # noqa: E402


class MarkerRunLogTest(unittest.TestCase):
    def test_records_json_lines_without_overwriting_previous_run(self):
        with tempfile.TemporaryDirectory() as folder:
            first = MarkerRunLog(folder)
            self.assertTrue(first.record("marker_observation", 12.3,
                                         state="not_detected", raw_center=None))
            first.close()
            second = MarkerRunLog(folder)
            self.assertNotEqual(first.path, second.path)
            self.assertTrue(second.record("follow_step", 13.0,
                                          image_error_x=0.1,
                                          target_xy_after=[1.0, 2.0]))
            second.close()
            with open(first.path, encoding="utf-8") as stream:
                self.assertEqual(json.loads(stream.readline())["state"], "not_detected")
            with open(second.path, encoding="utf-8") as stream:
                self.assertEqual(json.loads(stream.readline())["target_xy_after"],
                                 [1.0, 2.0])
            self.assertFalse(second.record("after_shutdown", 14.0))

    def test_invalid_value_does_not_raise_from_ros_callback(self):
        with tempfile.TemporaryDirectory() as folder:
            log = MarkerRunLog(folder)
            self.assertFalse(log.record("invalid", 1.0, value=float("nan")))
            self.assertIn("Out of range float", log.error)
            log.close()


if __name__ == "__main__":
    unittest.main()
