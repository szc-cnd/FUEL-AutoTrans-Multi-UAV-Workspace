#!/usr/bin/env python3
"""Generate headless evo trajectory and tracking-error reports for one run."""

import argparse
import concurrent.futures
import csv
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import threading
from datetime import datetime


ACTION_ADD = 1
TERMINATING_ACTIONS = {2, 5}
REPORT_DIR_NAME = "evo_report"
SUMMARY_FILE_NAME = "summary.md"
APE_STAT_LABELS = {
    "max": "最大误差",
    "mean": "平均误差",
    "median": "中位数误差",
    "min": "最小误差",
    "rmse": "均方根误差（RMSE）",
    "sse": "误差平方和（SSE）",
    "std": "误差标准差",
}


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def normalize_quaternion(x, y, z, w):
    values = [finite_float(value) for value in (x, y, z, w)]
    if any(value is None for value in values):
        return (0.0, 0.0, 0.0, 1.0)
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in values)


def rpy_to_quaternion(roll, pitch, yaw):
    values = [finite_float(value) for value in (roll, pitch, yaw)]
    if any(value is None for value in values):
        return (0.0, 0.0, 0.0, 1.0)
    roll, pitch, yaw = values
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quaternion(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def read_setpoint_samples(csv_path):
    samples = []
    with open(csv_path, newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            stamp = finite_float(row.get("stamp"))
            actual_xyz = tuple(finite_float(row.get(key)) for key in
                               ("odom_x", "odom_y", "odom_z"))
            reference_xyz = tuple(finite_float(row.get(key)) for key in
                                  ("ref_x", "ref_y", "ref_z"))
            if (stamp is None or any(value is None for value in actual_xyz) or
                    any(value is None for value in reference_xyz)):
                continue

            actual_q = rpy_to_quaternion(
                row.get("actual_roll"), row.get("actual_pitch"), row.get("actual_yaw"))
            reference_q = normalize_quaternion(
                row.get("ref_qx"), row.get("ref_qy"),
                row.get("ref_qz"), row.get("ref_qw"))
            samples.append((stamp, actual_xyz, actual_q, reference_xyz, reference_q))
    return samples


def merge_intervals(intervals):
    merged = []
    for start, end in sorted(intervals):
        if end < start:
            continue
        if not merged or start > merged[-1][1]:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    return [tuple(interval) for interval in merged]


def read_active_intervals(trajectory_csv_path):
    events = {}
    with open(trajectory_csv_path, newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            received = finite_float(row.get("received_stamp"))
            header = finite_float(row.get("header_stamp"))
            action_value = finite_float(row.get("action"))
            if received is None or action_value is None:
                continue
            action = int(action_value)
            key = (received, header, row.get("trajectory_id", ""), action)
            event = events.setdefault(key, [])
            duration = finite_float(row.get("duration"))
            if duration is not None and duration > 0.0:
                event.append(duration)

    active = []
    for (received, header, _trajectory_id, action), durations in sorted(
            events.items(), key=lambda item: item[0][0]):
        if action == ACTION_ADD and header is not None and durations:
            active.append((header, header + sum(durations)))
        elif action in TERMINATING_ACTIONS:
            active = [
                (start, min(end, received))
                for start, end in active
                if start < received
            ]
    return merge_intervals(active)


def sample_in_intervals(stamp, intervals):
    return any(start <= stamp <= end for start, end in intervals)


def write_tum_pair(samples, reference_path, actual_path):
    with open(reference_path, "w") as reference_file, open(actual_path, "w") as actual_file:
        for stamp, actual_xyz, actual_q, reference_xyz, reference_q in samples:
            reference_file.write(format_tum_pose(stamp, reference_xyz, reference_q))
            actual_file.write(format_tum_pose(stamp, actual_xyz, actual_q))


def format_tum_pose(stamp, xyz, quaternion):
    return ("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
            ((stamp,) + tuple(xyz) + tuple(quaternion)))


def find_evo_command(name):
    executable = shutil.which(name)
    if executable:
        return executable
    user_executable = os.path.expanduser("~/.local/bin/%s" % name)
    return user_executable if os.path.isfile(user_executable) else None


def run_command(command, report_log, timeout, stdout_path=None, log_lock=None):
    log_lock = log_lock or threading.Lock()
    with log_lock:
        report_log.write("$ %s\n" % " ".join(shlex.quote(value) for value in command))
        report_log.flush()
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        timeout=timeout,
        check=False,
    )
    output = result.stdout or ""
    with log_lock:
        report_log.write(output)
        if output and not output.endswith("\n"):
            report_log.write("\n")
        report_log.write("exit_code: %d\n\n" % result.returncode)
        report_log.flush()
    if stdout_path is not None:
        with open(stdout_path, "w") as output_file:
            output_file.write(output)
    if result.returncode != 0:
        raise RuntimeError("command failed with exit code %d: %s" %
                           (result.returncode, command[0]))


def promote_evo_plot(base_path, preferred_suffix, discard_suffixes=()):
    stem, extension = os.path.splitext(base_path)
    generated_path = "%s%s%s" % (stem, preferred_suffix, extension)
    if os.path.exists(generated_path):
        os.replace(generated_path, base_path)
    elif not os.path.exists(base_path):
        raise RuntimeError("evo did not create expected plot: %s" % generated_path)

    for suffix in discard_suffixes:
        extra_path = "%s%s%s" % (stem, suffix, extension)
        if os.path.exists(extra_path):
            os.remove(extra_path)


def format_local_time(timestamp):
    return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")


def parse_ape_stats(stats_path):
    stats = {}
    with open(stats_path) as stats_file:
        for line in stats_file:
            fields = line.split()
            if len(fields) >= 2 and fields[0].lower() in APE_STAT_LABELS:
                value = finite_float(fields[1])
                if value is not None:
                    stats[fields[0].lower()] = value
    return stats


def localize_error(error):
    if not error:
        return "未知错误"
    if "no rows contain both odometry and reference positions" in error:
        return "没有同时包含有效里程计和参考位置的数据行。"
    if "no samples fall inside a valid planned trajectory interval" in error:
        return "没有样本落在有效的规划执行区间内。"
    if "evo is not installed" in error:
        return "未安装 evo；请执行 `python3 -m pip install --user evo`。"
    if "command failed with exit code" in error:
        return "evo 命令执行失败：%s" % error
    return error


def write_summary(summary_path, status, run_dir, full_samples=0, active_samples=0,
                  intervals=(), ape_stats=None, error=None):
    ape_stats = ape_stats or {}
    with open(summary_path, "w") as summary:
        summary.write("# Evo 轨迹分析摘要\n\n")
        summary.write("## 运行信息\n\n")
        summary.write("- 状态：%s\n" % ("成功" if status == "success" else "失败"))
        summary.write("- 运行目录：`%s`\n" % run_dir)
        summary.write("- 全程有效样本：%d\n" % full_samples)
        summary.write("- 规划区间有效样本：%d\n" % active_samples)
        if intervals:
            interval_text = "；".join(
                "%s 至 %s" % (format_local_time(start), format_local_time(end))
                for start, end in intervals)
            summary.write("- 规划执行区间：%s\n" % interval_text)
        else:
            summary.write("- 规划执行区间：无有效区间\n")

        if status == "success":
            summary.write("\n## 位置误差\n\n")
            summary.write("- 误差类型：平移误差\n")
            summary.write("- 轨迹对齐：未对齐\n")
            summary.write("- 统计范围：仅统计规划执行区间\n")
            for key, label in APE_STAT_LABELS.items():
                if key in ape_stats:
                    value = ape_stats[key]
                    if key == "sse":
                        summary.write("- %s：%.6f m²（%.2f cm²）\n" %
                                      (label, value, value * 10000.0))
                    else:
                        summary.write("- %s：%.6f m（%.2f cm）\n" %
                                      (label, value, value * 100.0))
        else:
            summary.write("\n## 失败原因\n\n")
            summary.write("- %s\n" % localize_error(error))
            summary.write("- 建议检查 report.log、CSV 数据和 evo 安装状态。\n")

        summary.write("\n## 输出文件\n\n")
        for filename, label in (
                ("trajectory_full_xy.png", "全程 XY 轨迹"),
                ("trajectory_full_xyz.png", "全程 XYZ 轨迹"),
                ("ape_active.png", "规划区间位置误差图"),
                ("ape_active_stats.txt", "evo 原始误差统计"),
                ("report.log", "evo 原始命令和输出"),
        ):
            summary.write("- %s：`%s`\n" % (label, filename))


def generate_report(run_dir, setpoint_csv_path=None, trajectory_csv_path=None,
                    evo_traj_command=None, evo_ape_command=None, timeout=60.0):
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    run_name = os.path.basename(run_dir.rstrip(os.sep))
    if setpoint_csv_path is None:
        setpoint_csv_path = os.path.join(run_dir, "%s_setpoint.csv" % run_name)
    if trajectory_csv_path is None:
        trajectory_csv_path = os.path.join(run_dir, "trajectory.csv")

    report_dir = os.path.join(run_dir, REPORT_DIR_NAME)
    os.makedirs(report_dir, exist_ok=True)
    report_log_path = os.path.join(report_dir, "report.log")
    summary_path = os.path.join(report_dir, SUMMARY_FILE_NAME)
    samples = []
    active_samples = []
    intervals = []
    with open(report_log_path, "w") as report_log:
        report_log.write("run_dir: %s\n" % run_dir)
        report_log.write("setpoint_csv: %s\n" % setpoint_csv_path)
        report_log.write("trajectory_csv: %s\n" % trajectory_csv_path)
        try:
            samples = read_setpoint_samples(setpoint_csv_path)
            intervals = read_active_intervals(trajectory_csv_path)
            active_samples = [sample for sample in samples
                              if sample_in_intervals(sample[0], intervals)]
            report_log.write("full_samples: %d\n" % len(samples))
            report_log.write("active_samples: %d\n" % len(active_samples))
            report_log.write("active_intervals: %s\n" % json.dumps(intervals))
            report_log.flush()

            if not samples:
                raise RuntimeError("no rows contain both odometry and reference positions")
            if not intervals or not active_samples:
                raise RuntimeError("no samples fall inside a valid planned trajectory interval")

            paths = {
                "reference_full": os.path.join(report_dir, "reference_full.tum"),
                "actual_full": os.path.join(report_dir, "actual_full.tum"),
                "reference_active": os.path.join(report_dir, "reference_active.tum"),
                "actual_active": os.path.join(report_dir, "actual_active.tum"),
            }
            write_tum_pair(samples, paths["reference_full"], paths["actual_full"])
            write_tum_pair(
                active_samples, paths["reference_active"], paths["actual_active"])

            evo_traj_command = evo_traj_command or find_evo_command("evo_traj")
            evo_ape_command = evo_ape_command or find_evo_command("evo_ape")
            if not evo_traj_command or not evo_ape_command:
                raise RuntimeError(
                    "evo is not installed; run: python3 -m pip install --user evo")

            config_path = os.path.join(report_dir, "evo_headless.json")
            with open(config_path, "w") as config_file:
                json.dump({"plot_backend": "Agg"}, config_file, indent=2)

            common_traj = [
                evo_traj_command, "tum", paths["reference_full"], paths["actual_full"],
                "--ref", paths["reference_full"], "--no_warnings",
            ]
            jobs = []
            for mode in ("xy", "xyz"):
                jobs.append((
                    common_traj + [
                        "--plot_mode", mode,
                        "--save_plot", os.path.join(
                            report_dir, "trajectory_full_%s.png" % mode),
                        "--config", config_path,
                    ],
                    None,
                ))

            jobs.append(([
                    evo_ape_command, "tum",
                    paths["reference_active"], paths["actual_active"],
                    "--pose_relation", "trans_part",
                    "--plot_mode", "xyz",
                    "--plot_x_dimension", "seconds",
                    "--save_plot", os.path.join(report_dir, "ape_active.png"),
                    "--save_results", os.path.join(report_dir, "ape_active_results.zip"),
                    "--no_warnings", "--config", config_path,
                ], os.path.join(report_dir, "ape_active_stats.txt")))

            log_lock = threading.Lock()
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(jobs)) as executor:
                futures = [
                    executor.submit(
                        run_command,
                        command,
                        report_log,
                        timeout,
                        stdout_path,
                        log_lock,
                    )
                    for command, stdout_path in jobs
                ]
                for future in futures:
                    future.result()

            for mode in ("xy", "xyz"):
                promote_evo_plot(
                    os.path.join(report_dir, "trajectory_full_%s.png" % mode),
                    "_trajectories",
                    discard_suffixes=("_xyz", "_rpy", "_speeds"),
                )
            promote_evo_plot(
                os.path.join(report_dir, "ape_active.png"),
                "_raw",
            )
            ape_stats = parse_ape_stats(os.path.join(report_dir, "ape_active_stats.txt"))
            write_summary(
                summary_path, "success", run_dir, len(samples), len(active_samples),
                intervals, ape_stats=ape_stats)
            report_log.write("status: success\n")
            return report_dir
        except Exception as exc:
            write_summary(
                summary_path, "failed", run_dir, len(samples), len(active_samples),
                intervals, error=str(exc))
            report_log.write("status: failed\n")
            report_log.write("error: %s\n" % exc)
            raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--setpoint-csv")
    parser.add_argument("--trajectory-csv")
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        report_dir = generate_report(
            args.run_dir,
            setpoint_csv_path=args.setpoint_csv,
            trajectory_csv_path=args.trajectory_csv,
            timeout=args.timeout,
        )
    except Exception as exc:
        print("evo report failed: %s" % exc, file=sys.stderr)
        return 1
    print("evo report: %s" % report_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
