#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import sys
import types
import unittest


def install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.Time = type("Time", (), {"now": staticmethod(lambda: None)})
    sys.modules["rospy"] = rospy

    message_modules = {
        "geometry_msgs.msg": ["Accel", "PoseStamped"],
        "mavros_msgs.msg": ["AttitudeTarget", "ESCStatus", "ExtendedState", "RCIn", "State"],
        "nav_msgs.msg": ["Odometry", "Path"],
        "quadrotor_msgs.msg": ["PolynomialTraj", "PositionCommand"],
        "rosgraph_msgs.msg": ["Log"],
        "sensor_msgs.msg": ["BatteryState"],
        "std_msgs.msg": ["Bool", "Float64"],
    }
    for module_name, class_names in message_modules.items():
        package_name = module_name.split(".")[0]
        sys.modules.setdefault(package_name, types.ModuleType(package_name))
        module = types.ModuleType(module_name)
        for class_name in class_names:
            setattr(module, class_name, type(class_name, (), {}))
        sys.modules[module_name] = module

    rostopic = types.ModuleType("rostopic")
    rostopic.get_topic_class = lambda *args, **kwargs: (None, None, None)
    sys.modules["rostopic"] = rostopic


def load_logger_module():
    install_ros_stubs()
    script_path = pathlib.Path(__file__).resolve().parents[1] / "autotrans_mpc_logger.py"
    spec = importlib.util.spec_from_file_location("autotrans_mpc_logger", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BatteryLoggingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_logger_module()

    def make_logger(self):
        logger = self.module.AutoTransMpcLogger.__new__(self.module.AutoTransMpcLogger)
        logger.filtered_battery_voltage = None
        logger.latest_battery_percentage = None
        return logger

    @staticmethod
    def battery(cell_voltage, percentage):
        return types.SimpleNamespace(cell_voltage=cell_voltage, percentage=percentage)

    def test_first_valid_sample_initializes_without_zero_voltage_ramp(self):
        logger = self.make_logger()
        logger.battery_cb(self.battery([3.70, 3.71, 3.69, 3.72], 0.65))

        self.assertAlmostEqual(logger.filtered_battery_voltage, 14.82)
        self.assertAlmostEqual(logger.latest_battery_percentage, 0.65)
        self.assertEqual(logger.extract_battery_values(), [14.82, 0.65])

    def test_following_sample_uses_controller_matching_low_pass_filter(self):
        logger = self.make_logger()
        logger.battery_cb(self.battery([3.70, 3.70, 3.70, 3.70], 0.60))
        logger.battery_cb(self.battery([3.50, 3.50, 3.50, 3.50], 0.55))

        self.assertAlmostEqual(logger.filtered_battery_voltage, 0.8 * 14.8 + 0.2 * 14.0)
        self.assertAlmostEqual(logger.latest_battery_percentage, 0.55)

    def test_invalid_voltage_does_not_overwrite_last_valid_sample(self):
        logger = self.make_logger()
        logger.battery_cb(self.battery([3.70, 3.70, 3.70, 3.70], 0.60))
        logger.battery_cb(self.battery([], 0.50))
        logger.battery_cb(self.battery([3.60, math.nan, 3.60, 3.60], 0.40))

        self.assertEqual(logger.extract_battery_values(), [14.8, 0.60])

    def test_missing_battery_sample_writes_empty_columns(self):
        logger = self.make_logger()
        self.assertEqual(logger.extract_battery_values(), ["", ""])


class ForceAttitudeAlignmentLoggingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_logger_module()

    def make_logger(self):
        logger = self.module.AutoTransMpcLogger.__new__(self.module.AutoTransMpcLogger)
        logger.latest_force_attitude_aligned = None
        logger.latest_force_attitude_yaw_offset = None
        return logger

    def test_csv_header_appends_alignment_columns(self):
        self.assertEqual(
            self.module.CSV_HEADER[-2:],
            ["force_attitude_aligned", "force_attitude_yaw_offset_rad"])

    def test_composed_csv_row_matches_header_column_count(self):
        section_lengths = [8, 6, 3, 3, 7, 5, 5, 6, 4, 4, 4, 2, 2]
        sections = [[index] * length for index, length in enumerate(section_lengths)]
        row = self.module.AutoTransMpcLogger.compose_csv_row(*sections)

        self.assertEqual(len(row), len(self.module.CSV_HEADER))
        self.assertEqual(row[-2:], [12, 12])

    def test_missing_alignment_messages_write_empty_columns(self):
        logger = self.make_logger()
        self.assertEqual(logger.extract_force_attitude_alignment_values(), ["", ""])

    def test_alignment_callbacks_record_boolean_and_yaw_offset(self):
        logger = self.make_logger()
        logger.force_attitude_aligned_cb(types.SimpleNamespace(data=True))
        logger.force_attitude_yaw_offset_cb(types.SimpleNamespace(data=0.5235987756))

        values = logger.extract_force_attitude_alignment_values()
        self.assertEqual(values[0], 1)
        self.assertAlmostEqual(values[1], 0.5235987756)

    def test_non_finite_yaw_offset_is_not_written(self):
        logger = self.make_logger()
        logger.force_attitude_aligned_cb(types.SimpleNamespace(data=False))
        logger.force_attitude_yaw_offset_cb(types.SimpleNamespace(data=math.nan))

        self.assertEqual(logger.extract_force_attitude_alignment_values(), [0, ""])


if __name__ == "__main__":
    unittest.main()
