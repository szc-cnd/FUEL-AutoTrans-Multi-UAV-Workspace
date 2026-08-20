#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import pathlib
import subprocess
import tempfile
import unittest
from unittest import mock


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "generate_evo_report.py"
SPEC = importlib.util.spec_from_file_location("generate_evo_report", SCRIPT)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class EvoReportTest(unittest.TestCase):
    def write_csv(self, path, fieldnames, rows):
        with path.open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    def test_active_intervals_merge_replans_and_clip_on_abort(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = pathlib.Path(temp_dir) / "trajectory.csv"
            fields = ["received_stamp", "header_stamp", "trajectory_id", "action", "duration"]
            self.write_csv(path, fields, [
                {"received_stamp": 10.0, "header_stamp": 10.0, "trajectory_id": 1,
                 "action": 1, "duration": 4.0},
                {"received_stamp": 10.0, "header_stamp": 10.0, "trajectory_id": 1,
                 "action": 1, "duration": 3.0},
                {"received_stamp": 12.0, "header_stamp": 12.0, "trajectory_id": 2,
                 "action": 1, "duration": 8.0},
                {"received_stamp": 18.0, "header_stamp": 18.0, "trajectory_id": 3,
                 "action": 2, "duration": ""},
                {"received_stamp": 30.0, "header_stamp": 30.0, "trajectory_id": 4,
                 "action": 1, "duration": 2.0},
            ])

            self.assertEqual(REPORT.read_active_intervals(path), [(10.0, 18.0), (30.0, 32.0)])

    def test_setpoint_conversion_filters_invalid_rows_and_orders_quaternion(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = pathlib.Path(temp_dir) / "setpoint.csv"
            fields = [
                "stamp", "odom_x", "odom_y", "odom_z",
                "actual_roll", "actual_pitch", "actual_yaw",
                "ref_x", "ref_y", "ref_z", "ref_qw", "ref_qx", "ref_qy", "ref_qz",
            ]
            self.write_csv(path, fields, [
                {"stamp": 1.0, "odom_x": 1, "odom_y": 2, "odom_z": 3,
                 "actual_roll": 0, "actual_pitch": 0, "actual_yaw": math.pi,
                 "ref_x": 4, "ref_y": 5, "ref_z": 6,
                 "ref_qw": 2, "ref_qx": 0, "ref_qy": 0, "ref_qz": 0},
                {"stamp": 2.0, "odom_x": "", "odom_y": 2, "odom_z": 3,
                 "ref_x": 4, "ref_y": 5, "ref_z": 6},
            ])

            samples = REPORT.read_setpoint_samples(path)
            self.assertEqual(len(samples), 1)
            self.assertEqual(samples[0][4], (0.0, 0.0, 0.0, 1.0))
            self.assertAlmostEqual(samples[0][2][2], 1.0)
            self.assertAlmostEqual(samples[0][2][3], 0.0, places=7)

    def test_report_uses_headless_evo_without_alignment(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = pathlib.Path(temp_dir) / "run"
            run_dir.mkdir()
            setpoint = run_dir / "run_setpoint.csv"
            trajectory = run_dir / "trajectory.csv"
            setpoint_fields = [
                "stamp", "odom_x", "odom_y", "odom_z",
                "actual_roll", "actual_pitch", "actual_yaw",
                "ref_x", "ref_y", "ref_z", "ref_qw", "ref_qx", "ref_qy", "ref_qz",
            ]
            self.write_csv(setpoint, setpoint_fields, [
                {"stamp": stamp, "odom_x": stamp, "odom_y": 0, "odom_z": 1,
                 "actual_roll": 0, "actual_pitch": 0, "actual_yaw": 0,
                 "ref_x": stamp + 0.1, "ref_y": 0, "ref_z": 1,
                 "ref_qw": 1, "ref_qx": 0, "ref_qy": 0, "ref_qz": 0}
                for stamp in (1.0, 2.0, 3.0)
            ])
            self.write_csv(
                trajectory,
                ["received_stamp", "header_stamp", "trajectory_id", "action", "duration"],
                [{"received_stamp": 2.0, "header_stamp": 2.0,
                  "trajectory_id": 1, "action": 1, "duration": 1.0}],
            )

            completed = subprocess.CompletedProcess([], 0, stdout=(
                "max 0.079710\n"
                "mean 0.032907\n"
                "median 0.030445\n"
                "min 0.000073\n"
                "rmse 0.036563\n"
                "sse 6.494604\n"
                "std 0.015938\n"))
            def fake_run(command, **_kwargs):
                plot_path = pathlib.Path(command[command.index("--save_plot") + 1])
                plot_path.write_bytes(b"fake png")
                return completed

            with mock.patch.object(REPORT.subprocess, "run", side_effect=fake_run) as runner:
                output = REPORT.generate_report(
                    run_dir,
                    evo_traj_command="/fake/evo_traj",
                    evo_ape_command="/fake/evo_ape",
                )

            commands = [call.args[0] for call in runner.call_args_list]
            self.assertEqual(len(commands), 3)
            self.assertTrue(all("--align" not in command and "-a" not in command
                                for command in commands))
            self.assertEqual({command[command.index("--plot_mode") + 1]
                              for command in commands}, {"xy", "xyz"})
            config_path = pathlib.Path(output) / "evo_headless.json"
            self.assertEqual(json.loads(config_path.read_text())["plot_backend"], "Agg")
            self.assertEqual(
                len((pathlib.Path(output) / "reference_full.tum").read_text().splitlines()), 3)
            self.assertEqual(
                len((pathlib.Path(output) / "reference_active.tum").read_text().splitlines()), 2)
            self.assertEqual(
                (pathlib.Path(output) / "ape_active_stats.txt").read_text(),
                "max 0.079710\nmean 0.032907\nmedian 0.030445\nmin 0.000073\n"
                "rmse 0.036563\nsse 6.494604\nstd 0.015938\n")
            for filename in (
                    "trajectory_full_xy.png", "trajectory_full_xyz.png", "ape_active.png"):
                self.assertTrue((pathlib.Path(output) / filename).is_file())
            summary = (pathlib.Path(output) / "summary.md").read_text()
            self.assertIn("# Evo 轨迹分析摘要", summary)
            self.assertIn("状态：成功", summary)
            self.assertIn("全程有效样本：3", summary)
            self.assertIn("规划区间有效样本：2", summary)
            for label in ("最大误差", "平均误差", "中位数误差", "最小误差",
                          "均方根误差（RMSE）", "误差平方和（SSE）", "误差标准差"):
                self.assertIn(label, summary)
            self.assertIn("最大误差：0.079710 m（7.97 cm）", summary)
            self.assertIn("均方根误差（RMSE）：0.036563 m（3.66 cm）", summary)
            self.assertIn("误差平方和（SSE）：6.494604 m²（64946.04 cm²）", summary)

    def test_missing_active_interval_leaves_failure_log(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = pathlib.Path(temp_dir) / "run"
            run_dir.mkdir()
            self.write_csv(
                run_dir / "run_setpoint.csv",
                ["stamp", "odom_x", "odom_y", "odom_z", "ref_x", "ref_y", "ref_z"],
                [{"stamp": 1, "odom_x": 0, "odom_y": 0, "odom_z": 0,
                  "ref_x": 0, "ref_y": 0, "ref_z": 0}],
            )
            self.write_csv(
                run_dir / "trajectory.csv",
                ["received_stamp", "header_stamp", "trajectory_id", "action", "duration"],
                [],
            )

            with self.assertRaisesRegex(RuntimeError, "valid planned trajectory interval"):
                REPORT.generate_report(run_dir)
            failure_log = (run_dir / "evo_report" / "report.log").read_text()
            self.assertIn("status: failed", failure_log)
            failure_summary = (run_dir / "evo_report" / "summary.md").read_text()
            self.assertIn("状态：失败", failure_summary)
            self.assertIn("没有样本落在有效的规划执行区间内", failure_summary)

    def test_missing_evo_leaves_install_hint_in_failure_log(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = pathlib.Path(temp_dir) / "run"
            run_dir.mkdir()
            self.write_csv(
                run_dir / "run_setpoint.csv",
                ["stamp", "odom_x", "odom_y", "odom_z", "ref_x", "ref_y", "ref_z"],
                [{"stamp": 1, "odom_x": 0, "odom_y": 0, "odom_z": 0,
                  "ref_x": 0, "ref_y": 0, "ref_z": 0}],
            )
            self.write_csv(
                run_dir / "trajectory.csv",
                ["received_stamp", "header_stamp", "trajectory_id", "action", "duration"],
                [{"received_stamp": 1, "header_stamp": 1, "trajectory_id": 1,
                  "action": 1, "duration": 1}],
            )

            with mock.patch.object(REPORT, "find_evo_command", return_value=None):
                with self.assertRaisesRegex(RuntimeError, "evo is not installed"):
                    REPORT.generate_report(run_dir)
            failure_log = (run_dir / "evo_report" / "report.log").read_text()
            self.assertIn("python3 -m pip install --user evo", failure_log)
            failure_summary = (run_dir / "evo_report" / "summary.md").read_text()
            self.assertIn("状态：失败", failure_summary)
            self.assertIn("未安装 evo", failure_summary)

    def test_evo_command_failure_writes_chinese_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = pathlib.Path(temp_dir) / "run"
            run_dir.mkdir()
            self.write_csv(
                run_dir / "run_setpoint.csv",
                ["stamp", "odom_x", "odom_y", "odom_z", "ref_x", "ref_y", "ref_z"],
                [{"stamp": 1, "odom_x": 0, "odom_y": 0, "odom_z": 0,
                  "ref_x": 0, "ref_y": 0, "ref_z": 0}],
            )
            self.write_csv(
                run_dir / "trajectory.csv",
                ["received_stamp", "header_stamp", "trajectory_id", "action", "duration"],
                [{"received_stamp": 1, "header_stamp": 1, "trajectory_id": 1,
                  "action": 1, "duration": 1}],
            )

            failed = subprocess.CompletedProcess([], 2, stdout="evo failed\n")
            with mock.patch.object(REPORT.subprocess, "run", return_value=failed):
                with self.assertRaisesRegex(RuntimeError, "command failed with exit code 2"):
                    REPORT.generate_report(
                        run_dir,
                        evo_traj_command="/fake/evo_traj",
                        evo_ape_command="/fake/evo_ape",
                    )

            failure_summary = (run_dir / "evo_report" / "summary.md").read_text()
            self.assertIn("状态：失败", failure_summary)
            self.assertIn("evo 命令执行失败", failure_summary)


if __name__ == "__main__":
    unittest.main()
