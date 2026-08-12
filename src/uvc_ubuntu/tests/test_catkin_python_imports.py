import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class CatkinPythonImportTest(unittest.TestCase):
    def test_scripts_prioritize_rospack_source_directory_before_shared_import(self):
        for script_name in ("thermal_ros_node.py", "thermal_raw_view.py"):
            source = (ROOT / script_name).read_text(encoding="utf-8")
            lookup = 'rospkg.RosPack().get_path("uvc_ubuntu")'
            bootstrap = "sys.path.insert(0, PACKAGE_DIR)"
            shared_import = "from thermal_detect import"

            self.assertIn(lookup, source, script_name)
            self.assertIn(bootstrap, source, script_name)
            self.assertIn(shared_import, source, script_name)
            self.assertLess(
                source.index(bootstrap),
                source.index(shared_import),
                script_name,
            )


if __name__ == "__main__":
    unittest.main()
