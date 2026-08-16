#!/usr/bin/env python3
"""检查轨迹日志接口和 launch 参数，避免后续回归时漏记轨迹。"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
LOGGER = ROOT / "controller" / "payload_mpc_controller" / "scripts" / "autotrans_mpc_logger.py"
LAUNCH = ROOT / "controller" / "payload_mpc_controller" / "launch" / "quad_wind_mpc_controller.launch"
CMAKE = ROOT / "controller" / "payload_mpc_controller" / "CMakeLists.txt"


class TrajectoryLoggingStaticTest(unittest.TestCase):
    def test_logger_has_both_trajectory_outputs_and_fields(self):
        source = LOGGER.read_text(encoding="utf-8")
        for filename in ("trajectory.csv", "planner_trajectory.csv"):
            self.assertIn(filename, source)
        for field in (
            "received_stamp",
            "header_stamp",
            "trajectory_id",
            "action",
            "piece_index",
            "duration",
            "num_order",
            "num_dim",
            "data",
        ):
            self.assertIn(field, source)
        for field in ("drone_id", "traj_id", "start_time", "coef_x", "coef_y", "coef_z"):
            self.assertIn(field, source)


    def test_launch_passes_raw_and_mpc_trajectory_topics_to_logger(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('arg name="raw_trajectory_topic"', source)
        self.assertIn('param name="trajectory_topic" value="$(arg trajectory_topic)"', source)
        self.assertIn('param name="raw_trajectory_topic" value="$(arg raw_trajectory_topic)"', source)

    def test_shutdown_stops_log_timer_before_closing_files(self):
        source = LOGGER.read_text(encoding="utf-8")
        timer_shutdown = source.index("self.log_timer.shutdown()")
        csv_close = source.index("self.csv_file.close()")
        self.assertLess(
            timer_shutdown,
            csv_close,
            "主日志定时器必须在关闭 CSV 文件前停止",
        )

    def test_evo_report_runs_after_csv_close_and_is_configurable(self):
        logger_source = LOGGER.read_text(encoding="utf-8")
        launch_source = LAUNCH.read_text(encoding="utf-8")
        cmake_source = CMAKE.read_text(encoding="utf-8")
        self.assertLess(
            logger_source.index("self.csv_file.close()"),
            logger_source.index("self.generate_evo_report()"),
        )
        self.assertIn('arg name="enable_evo_report" default="true"', launch_source)
        self.assertIn(
            'param name="enable_evo_report" value="$(arg enable_evo_report)"',
            launch_source,
        )
        self.assertIn("scripts/generate_evo_report.py", cmake_source)


if __name__ == "__main__":
    unittest.main()
