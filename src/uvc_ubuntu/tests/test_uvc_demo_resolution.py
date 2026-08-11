import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.modules.setdefault("cv2", mock.MagicMock())
sys.modules.setdefault("numpy", mock.MagicMock())

import thermal_detect


class UvcDemoResolutionTest(unittest.TestCase):
    def test_explicit_existing_path_wins(self):
        with tempfile.NamedTemporaryFile() as executable:
            self.assertEqual(
                thermal_detect.resolve_uvc_demo_path(executable.name),
                os.path.abspath(executable.name),
            )

    @mock.patch("thermal_detect._is_executable_file", return_value=False)
    @mock.patch("thermal_detect._find_ros_uvc_demo")
    @mock.patch("thermal_detect.shutil.which", return_value=None)
    def test_non_executable_source_binary_is_skipped(
        self, _which, find_ros, _is_executable
    ):
        find_ros.return_value = "/tmp/catkin/devel/lib/uvc_ubuntu/uvc_demo"
        self.assertEqual(
            thermal_detect.resolve_uvc_demo_path("uvc_demo"),
            find_ros.return_value,
        )

    @mock.patch("thermal_detect._is_executable_file", return_value=False)
    @mock.patch("thermal_detect._find_ros_uvc_demo")
    @mock.patch("thermal_detect.shutil.which", return_value=None)
    def test_ros_libexec_path_is_used(self, _which, find_ros, _is_executable):
        find_ros.return_value = "/tmp/catkin/devel/lib/uvc_ubuntu/uvc_demo"
        self.assertEqual(
            thermal_detect.resolve_uvc_demo_path("uvc_demo"),
            find_ros.return_value,
        )

    @mock.patch("thermal_detect._is_executable_file", return_value=False)
    @mock.patch("thermal_detect._find_ros_uvc_demo", return_value=None)
    @mock.patch(
        "thermal_detect.shutil.which",
        return_value="/usr/local/bin/uvc_demo",
    )
    def test_path_lookup_is_last_fallback(self, which, _find_ros, _is_executable):
        self.assertEqual(
            thermal_detect.resolve_uvc_demo_path("uvc_demo"),
            which.return_value,
        )


if __name__ == "__main__":
    unittest.main()
