import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RosPackageLayoutTest(unittest.TestCase):
    def test_package_manifest_declares_name_and_runtime_dependencies(self):
        root = ET.parse(ROOT / "package.xml").getroot()
        self.assertEqual(root.findtext("name"), "uvc_ubuntu")
        dependencies = {
            node.text
            for tag in ("depend", "exec_depend")
            for node in root.findall(tag)
        }
        for dependency in (
            "rospy",
            "std_msgs",
            "sensor_msgs",
            "geometry_msgs",
            "tf2_ros",
            "tf2_geometry_msgs",
            "cv_bridge",
        ):
            self.assertIn(dependency, dependencies)

        full_dependencies = {node.text for node in root.findall("depend")}
        self.assertIn("libusb-1.0", full_dependencies)
        self.assertNotIn("libuvc", full_dependencies)

    def test_cmake_builds_and_installs_all_runtime_entry_points(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(uvc_demo", cmake)
        self.assertIn("add_subdirectory(libuvc", cmake)
        self.assertIn("catkin_install_python(PROGRAMS", cmake)
        for script in (
            "thermal_detect.py",
            "thermal_raw_view.py",
            "thermal_ros_node.py",
            "thermal_d435_fusion_node.py",
            "thermal_d435_calib_node.py",
            "check_uvc_fps.py",
        ):
            self.assertIn(script, cmake)
        self.assertIn("install(DIRECTORY launch config", cmake)

    def test_bundled_libuvc_contains_sources_without_old_build_output(self):
        libuvc = ROOT / "libuvc"
        self.assertTrue(libuvc.is_dir())
        self.assertTrue((libuvc / "CMakeLists.txt").is_file())
        self.assertTrue((libuvc / "include" / "libuvc" / "libuvc.h").is_file())
        self.assertTrue(
            (libuvc / "include" / "libuvc" / "libuvc_internal.h").is_file()
        )
        self.assertFalse((libuvc / "build").exists())

    def test_legacy_makefile_builds_the_bundled_libuvc(self):
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        self.assertIn("TARGET := uvc_demo", makefile)
        self.assertIn(
            "SOURCES := main.c hik_sensor.c hik_capture.c uvc_camera.c", makefile
        )
        self.assertIn("LIBUVC_STATIC :=", makefile)
        self.assertIn("--target uvc_static", makefile)
        self.assertNotIn(" -luvc", makefile)

    def test_launch_files_are_valid_and_start_expected_nodes(self):
        detector_root = ET.parse(ROOT / "launch" / "thermal_detector.launch").getroot()
        detector_node = detector_root.find("node")
        self.assertIsNotNone(detector_node)
        self.assertEqual(detector_node.get("pkg"), "uvc_ubuntu")
        self.assertEqual(detector_node.get("type"), "thermal_ros_node.py")
        self.assertIsNotNone(detector_node.find("rosparam"))

        fusion_root = ET.parse(
            ROOT / "launch" / "thermal_d435_fusion.launch"
        ).getroot()
        fusion_node = fusion_root.find("node")
        self.assertIsNotNone(fusion_node)
        self.assertEqual(fusion_node.get("type"), "thermal_d435_fusion_node.py")
        self.assertIsNotNone(fusion_node.find("rosparam"))

        system_root = ET.parse(
            ROOT / "launch" / "thermal_d435_system.launch"
        ).getroot()
        included_files = [node.get("file", "") for node in system_root.findall("include")]
        self.assertTrue(any("thermal_detector.launch" in path for path in included_files))
        self.assertTrue(any("thermal_d435_fusion.launch" in path for path in included_files))

    def test_detector_yaml_keeps_competition_defaults(self):
        config = (ROOT / "config" / "thermal_detector.yaml").read_text(
            encoding="utf-8"
        )
        for expected in (
            "width: 384",
            "height: 288",
            "shift_x: 0",
            "shift_y: 0",
            "roi_margin_x: 30",
            "roi_margin_y: 20",
            "uvc_demo_path: \"uvc_demo\"",
            "uvc_offset_fix: 0",
            "stable_filter_enable: true",
            "stable_max_len: 3",
            "stable_min_hits: 2",
            "stable_max_pixel_jump: 25",
            "reject_roi_border_touching: false",
        ):
            self.assertIn(expected, config)

    def test_fusion_publishes_d435_mapping_debug_image(self):
        source = (ROOT / "thermal_d435_fusion_node.py").read_text(encoding="utf-8")
        config = (ROOT / "config" / "thermal_d435_fusion.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("d435_debug_image_pub", source)
        self.assertIn("publish_d435_debug_image", source)
        self.assertIn("d435_exposure_ready", source)
        self.assertIn("map_thermal_bbox_to_d435", source)
        self.assertIn("D435 XYZ=", source)
        self.assertIn(
            "thermal_candidate_status_topic: /UAV0/thermal/target_candidate_status",
            config,
        )
        self.assertIn("d435_color_topic: /camera/color/image_raw", config)
        self.assertIn(
            "d435_debug_image_topic: /UAV0/thermal/d435_debug_image",
            config,
        )
        self.assertIn("d435_exposure_warmup_seconds: 5.0", config)

    def test_ros_package_guide_documents_both_workflows(self):
        guide = (ROOT / "README_ROS_PACKAGE.md").read_text(encoding="utf-8")
        for command in (
            "catkin_make",
            "rosrun uvc_ubuntu",
            "roslaunch uvc_ubuntu thermal_detector.launch",
            "make clean",
        ):
            self.assertIn(command, guide)


if __name__ == "__main__":
    unittest.main()
