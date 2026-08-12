#!/usr/bin/env python3

import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class ChineseLogLocaleStaticTest(unittest.TestCase):
    def test_locale_is_initialized_before_ros(self):
        source = (PACKAGE_ROOT / "src" / "mpc_controller_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("#include <clocale>", source)
        locale_pos = source.index('std::setlocale(LC_ALL, "");')
        ros_init_pos = source.index('ros::init(argc, argv, "MPCctrl");')
        self.assertLess(locale_pos, ros_init_pos)

    def test_takeoff_failure_reasons_describe_the_failure(self):
        source = (PACKAGE_ROOT / "src" / "mpc_fsm.cpp").read_text(
            encoding="utf-8"
        )

        expected_reasons = {
            "OFFBOARD": "PX4 尚未进入 OFFBOARD",
            "SENSOR_STALE": "定位、IMU 或 RPM 数据超时",
            "SENSOR_INVALID": "传感器数据无效",
            "DISARMED": "飞控尚未解锁",
            "NOT_ON_GROUND": "PX4 尚未确认在地面",
            "SPEED_UNSAFE": "定位速度超过安全阈值",
            "RC_STALE": "遥控器数据超时",
            "STATE_STALE": "PX4 状态数据超时",
            "EXTENDED_STATE_STALE": "PX4 扩展状态数据超时",
            "TARGET_NOT_ABOVE_UAV": "起飞目标高度未高于当前高度",
            "INITIAL_XY_TOO_FAR": "当前位置距离固定悬停点过远",
            "DISABLED": "起飞功能已关闭",
        }
        for reason, message in expected_reasons.items():
            self.assertIn(
                'std::strcmp(reason, "%s") == 0) reason_zh = "%s"'
                % (reason, message),
                source,
            )

        self.assertIn("[AUTO_TAKEOFF] 条件未满足：%s。", source)


if __name__ == "__main__":
    unittest.main()
