#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Accel, PoseStamped
from mavros_msgs.msg import AttitudeTarget, ESCStatus, ExtendedState, RCIn, State
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PolynomialTraj, PositionCommand
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64
try:
    from std_msgs.msg import Empty
except ImportError:  # 兼容不包含 Empty 的轻量单元测试消息桩。
    class Empty:
        pass
import rostopic


MAX_EXPERIMENT_TAG_LENGTH = 96
COMPENSATION_PARAM = (
    "/mpc_controller_node/force_estimator/enable_disturbance_compensation")
FORCE_CONFIG_PARAMS = (
    ("force_config_attitude_from_odom",
     "/mpc_controller_node/force_estimator/force_attitude_from_odom"),
    ("force_config_input_sync_enabled",
     "/mpc_controller_node/force_estimator/enable_input_sync"),
    ("force_config_sync_history_duration",
     "/mpc_controller_node/force_estimator/force_sync_history_duration"),
    ("force_config_sync_max_interp_gap",
     "/mpc_controller_node/force_estimator/force_sync_max_interp_gap"),
    ("force_config_sync_max_age",
     "/mpc_controller_node/force_estimator/force_sync_max_age"),
    ("force_config_sync_reset_backjump",
     "/mpc_controller_node/force_estimator/force_sync_reset_backjump"),
    ("force_config_max_applied_force",
     "/mpc_controller_node/force_estimator/max_applied_force"),
    ("force_config_max_applied_force_rate_xy",
     "/mpc_controller_node/force_estimator/max_applied_force_rate_xy"),
    ("force_config_max_applied_force_rate_z",
     "/mpc_controller_node/force_estimator/max_applied_force_rate_z"),
    ("force_config_axis_gain_x",
     "/mpc_controller_node/force_estimator/force_axis_gain_x"),
    ("force_config_axis_gain_y",
     "/mpc_controller_node/force_estimator/force_axis_gain_y"),
    ("force_config_axis_gain_z",
     "/mpc_controller_node/force_estimator/force_axis_gain_z"),
)

CSV_HEADER = [
    "stamp", "seq", "frame_id", "type_mask",
    "body_rate_x", "body_rate_y", "body_rate_z", "thrust",
    "odom_x", "odom_y", "odom_z", "odom_vx", "odom_vy", "odom_vz",
    "actual_roll", "actual_pitch", "actual_yaw",
    "goal_x", "goal_y", "goal_z",
    "ref_x", "ref_y", "ref_z", "ref_qw", "ref_qx", "ref_qy", "ref_qz",
    "err_x", "err_y", "err_z", "err_xy", "err_norm",
    "px4_mode", "px4_armed", "px4_connected", "px4_guided", "landed_state",
    "rc_ch1", "rc_ch2", "rc_ch3", "rc_ch4", "rc_ch8", "rc_ch10",
    "force_x", "force_y", "force_z", "force_norm",
    "applied_force_x", "applied_force_y", "applied_force_z", "applied_force_norm",
    "rpm_0", "rpm_1", "rpm_2", "rpm_3",
    "battery_voltage", "battery_percentage",
    "force_attitude_aligned", "force_attitude_yaw_offset_rad",
]

POSITION_CMD_HEADER = [
    "received_stamp", "header_stamp", "header_seq", "frame_id",
    "trajectory_id", "trajectory_flag",
    "position_x", "position_y", "position_z",
    "velocity_x", "velocity_y", "velocity_z",
    "acceleration_x", "acceleration_y", "acceleration_z",
    "jerk_x", "jerk_y", "jerk_z",
    "yaw", "yaw_dot",
    "kx_x", "kx_y", "kx_z", "kv_x", "kv_y", "kv_z",
]


def sanitize_experiment_tag(value):
    """将实验标签转换为安全、可读的单级目录名。"""
    raw_value = "" if value is None else str(value).strip()
    cleaned_parts = []
    for part in re.split(r"(__)", raw_value):
        if part == "__":
            cleaned_parts.append(part)
            continue

        cleaned_chars = []
        for char in part:
            if char.isspace() or char in "/\\:":
                cleaned_chars.append("_")
            elif char.isascii() and (char.isalnum() or char in "_-."):
                cleaned_chars.append(char)
            else:
                cleaned_chars.append("_")
        cleaned_parts.append(re.sub(r"_+", "_", "".join(cleaned_chars)))

    # 单下划线用于分词；仅保留用户原始标签中的双下划线作为实验条件分隔符。
    cleaned = re.sub(r"_{3,}", "__", "".join(cleaned_parts))
    cleaned = cleaned[:MAX_EXPERIMENT_TAG_LENGTH].strip("_.")
    if not cleaned or cleaned in (".", ".."):
        return "unlabeled"
    return cleaned


def build_generated_run_name(timestamp, tag, compensation_status):
    """生成同一实验目录、CSV 和文本日志共用的名称前缀。"""
    return "%s__%s__%s" % (
        timestamp, sanitize_experiment_tag(tag), compensation_status)


def select_unique_run_name(log_root, candidate):
    """为自动名称选择未被占用的目录，避免同一秒重复启动覆盖日志。"""
    if not os.path.exists(os.path.join(log_root, candidate)):
        return candidate

    suffix = 2
    while True:
        unique_candidate = "%s__%02d" % (candidate, suffix)
        if not os.path.exists(os.path.join(log_root, unique_candidate)):
            return unique_candidate
        suffix += 1


def validate_explicit_run_name(run_name):
    """显式名称必须是单级目录名，禁止跳出日志根目录。"""
    if any(char in run_name for char in ("/", "\\", ":")) or ".." in run_name:
        raise ValueError(
            "run_name must not contain '/', '\\', ':', or '..': %s" % run_name)


def normalize_rosbag_topics(topics):
    """清理并去重 rosbag 话题，同时保持 launch 中配置的顺序。"""
    if not isinstance(topics, (list, tuple)):
        return []
    normalized = []
    seen = set()
    for topic in topics:
        topic = str(topic).strip()
        if not topic or topic in seen:
            continue
        normalized.append(topic)
        seen.add(topic)
    return normalized


def force_config_log_lines(get_param):
    """生成控制器外力补偿参数快照，便于从单份日志复现实验配置。"""
    lines = []
    for label, path in FORCE_CONFIG_PARAMS:
        value = get_param(path, None)
        if value is None:
            value = "unknown"
        elif isinstance(value, bool):
            value = "true" if value else "false"
        lines.append("%s: %s" % (label, value))
    return lines


class AutoTransMpcLogger:
    @staticmethod
    def compose_csv_row(*sections):
        row = []
        for section in sections:
            row.extend(section)
        if len(row) != len(CSV_HEADER):
            raise ValueError(
                "CSV row has %d columns, expected %d" %
                (len(row), len(CSV_HEADER)))
        return row

    def __init__(self):
        default_root = os.path.expanduser("~/.ros/autotrans_mpc_logs")
        self.log_root = rospy.get_param("~log_dir", default_root)
        explicit_run_name = str(rospy.get_param("~run_name", "")).strip()
        self.experiment_tag_raw = rospy.get_param("~experiment_tag", "unlabeled")
        self.experiment_tag_clean = sanitize_experiment_tag(self.experiment_tag_raw)

        compensation_value = rospy.get_param(COMPENSATION_PARAM, None)
        if isinstance(compensation_value, bool):
            self.compensation_config = "true" if compensation_value else "false"
            compensation_status = "comp_on" if compensation_value else "comp_off"
        else:
            self.compensation_config = "unknown"
            compensation_status = "comp_unknown"

        if explicit_run_name:
            validate_explicit_run_name(explicit_run_name)
            self.run_name_source = "explicit"
            self.run_name = explicit_run_name
            if os.path.exists(os.path.join(self.log_root, self.run_name)):
                raise RuntimeError(
                    "explicit run_name already exists; refusing to overwrite: %s"
                    % self.run_name)
        else:
            self.run_name_source = "generated"
            generated_name = build_generated_run_name(
                time.strftime("%Y%m%d_%H%M%S"),
                self.experiment_tag_clean,
                compensation_status)
            self.run_name = select_unique_run_name(self.log_root, generated_name)

        self.run_dir = os.path.join(self.log_root, self.run_name)
        self.csv_path = os.path.join(self.run_dir, "%s_setpoint.csv" % self.run_name)
        self.text_log_path = os.path.join(self.run_dir, "%s_autotrans.log" % self.run_name)
        self.flush_every = int(rospy.get_param("~flush_every", 20))
        self.log_all_rosout = bool(rospy.get_param("~log_all_rosout", False))
        self.log_rate = float(rospy.get_param("~log_rate", 100.0))
        self.setpoint_timeout = float(rospy.get_param("~setpoint_timeout", 0.2))
        self.enable_evo_report = bool(rospy.get_param("~enable_evo_report", True))

        self.latest_setpoint = None
        self.latest_setpoint_received = rospy.Time(0)
        self.latest_odom = None
        self.latest_goal = None
        self.latest_reference = None
        self.latest_state = None
        self.latest_extended_state = None
        self.latest_rc = None
        self.latest_force = None
        self.latest_applied_force = None
        self.latest_esc_status = None
        # 电池总电压单位 V；仅用于日志分析，不参与控制器的推力或安全判断。
        self.filtered_battery_voltage = None
        # PX4/MAVROS 提供的剩余电量比例，通常范围为 0~1。
        self.latest_battery_percentage = None
        # 外力姿态偏航对齐状态及偏航差；偏航差单位 rad，正方向遵循 camera_init 的 Z 轴。
        self.latest_force_attitude_aligned = None
        self.latest_force_attitude_yaw_offset = None
        self.rows_since_flush = 0
        self.trajectory_rows_since_flush = 0
        self.planner_trajectory_rows_since_flush = 0
        self.position_cmd_rows_since_flush = 0
        # 所有日志文件共用同一把锁，避免回调线程与关闭流程同时操作文件。
        self.file_lock = threading.Lock()
        self.closed = False
        self.log_timer = None
        self.trajectory_subscriber = None
        self.raw_trajectory_subscriber = None
        self.position_cmd_subscriber = None
        self.raw_trajectory_retry_timer = None
        self.rosbag_process = None
        self.rosbag_console_file = None
        self.enable_rosbag = bool(rospy.get_param("~enable_rosbag", True))
        self.rosbag_topics = normalize_rosbag_topics(
            rospy.get_param("~rosbag_topics", []))

        if not os.path.exists(self.run_dir):
            os.makedirs(self.run_dir)

        self.csv_file = open(self.csv_path, "w", newline="")
        self.text_file = open(self.text_log_path, "w")
        self.trajectory_path = os.path.join(self.run_dir, "trajectory.csv")
        self.planner_trajectory_path = os.path.join(self.run_dir, "planner_trajectory.csv")
        self.position_cmd_path = os.path.join(self.run_dir, "position_cmd.csv")
        self.trajectory_file = open(self.trajectory_path, "w", newline="")
        self.planner_trajectory_file = open(self.planner_trajectory_path, "w", newline="")
        self.position_cmd_file = open(self.position_cmd_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.planner_trajectory_writer = csv.writer(self.planner_trajectory_file)
        self.position_cmd_writer = csv.writer(self.position_cmd_file)
        self.writer.writerow(CSV_HEADER)
        self.csv_file.flush()
        self.trajectory_writer.writerow([
            "received_stamp", "header_stamp", "header_seq", "frame_id",
            "trajectory_id", "action", "piece_index", "duration",
            "num_order", "num_dim", "data",
        ])
        self.planner_trajectory_writer.writerow([
            "received_stamp", "drone_id", "traj_id", "start_time", "order",
            "coef_x", "coef_y", "coef_z", "duration",
        ])
        self.position_cmd_writer.writerow(POSITION_CMD_HEADER)
        self.trajectory_file.flush()
        self.planner_trajectory_file.flush()
        self.position_cmd_file.flush()

        self.write_text("===== AutoTrans MPC real-flight log =====")
        self.write_text("start_time: %.6f" % rospy.Time.now().to_sec())
        self.write_text("run_name: %s" % self.run_name)
        self.write_text("experiment_tag_raw: %s" % self.experiment_tag_raw)
        self.write_text("experiment_tag_clean: %s" % self.experiment_tag_clean)
        self.write_text("compensation_config: %s" % self.compensation_config)
        for config_line in force_config_log_lines(rospy.get_param):
            self.write_text(config_line)
        for label in (
                "odom_topic", "imu_topic", "force_attitude_odom_topic",
                "comparison_imu_topic", "comparison_attitude_odom_topic"):
            self.write_text("force_input_%s: %s" % (
                label, rospy.get_param("~" + label, "unknown")))
        self.write_text("run_name_source: %s" % self.run_name_source)
        self.write_text("generated_run_name: %s" % self.run_name)
        self.write_text("run_dir: %s" % self.run_dir)
        self.write_text("csv_file: %s" % self.csv_path)
        self.write_text("text_file: %s" % self.text_log_path)
        self.write_text("trajectory_file: %s" % self.trajectory_path)
        self.write_text("planner_trajectory_file: %s" % self.planner_trajectory_path)
        self.write_text("position_cmd_file: %s" % self.position_cmd_path)
        self.write_text("body_rate_x/y/z: body-frame angular-rate commands, rad/s")
        self.write_text("thrust: MAVROS/PX4 normalized thrust, not force in newtons")
        self.write_text("ref_*: first pose of MPC reference path; used for tracking error")
        self.write_text("force_*: estimated world-frame external force, N")
        self.write_text("applied_force_*: world-frame force actually passed to NMPC OnlineData, N")
        self.write_text("battery_voltage: filtered total battery voltage, V")
        self.write_text("battery_percentage: PX4/MAVROS remaining battery ratio, normally 0..1")
        self.write_text("force_attitude_aligned: 1 when the configured force-attitude source is ready")
        self.write_text("force_attitude_yaw_offset_rad: PX4-to-FAST-LIO yaw offset in fallback alignment mode; otherwise 0, rad")
        self.write_text("trajectory.csv: MPC input quadrotor_msgs/PolynomialTraj; one row per polynomial piece")
        self.write_text("trajectory.csv data: original polynomial coefficient array, JSON encoded")
        self.write_text("planner_trajectory.csv: raw Diff-Planner traj_utils/PolyTraj")
        self.write_text("planner_trajectory.csv arrays: JSON encoded without coefficient reordering")
        self.write_text("position_cmd.csv: planner PositionCommand actually published to the NMPC input topic")
        self.write_text("position_cmd position/velocity/acceleration/jerk: planner reference in the configured world frame")
        self.write_text("position_cmd yaw/yaw_dot: planner yaw and yaw rate, rad/rad/s; not the RViz goal point")

        setpoint_topic = rospy.get_param("~setpoint_topic", "/mavros/setpoint_raw/attitude")
        odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        reference_topic = rospy.get_param("~reference_topic", "/mpc_controller_node/mpc/reference_trajectory")
        state_topic = rospy.get_param("~state_topic", "/mavros/state")
        extended_state_topic = rospy.get_param("~extended_state_topic", "/mavros/extended_state")
        rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        force_topic = rospy.get_param("~force_topic", "/mpc_controller_node/mpc/force")
        applied_force_topic = rospy.get_param(
            "~applied_force_topic", "/mpc_controller_node/mpc/force_applied")
        esc_status_topic = rospy.get_param("~esc_status_topic", "/mavros/esc_status")
        battery_topic = rospy.get_param("~battery_topic", "/mavros/battery")
        force_attitude_aligned_topic = rospy.get_param(
            "~force_attitude_aligned_topic",
            "/mpc_controller_node/mpc/force_attitude_aligned")
        force_attitude_yaw_offset_topic = rospy.get_param(
            "~force_attitude_yaw_offset_topic",
            "/mpc_controller_node/mpc/force_attitude_yaw_offset")
        trajectory_topic = rospy.get_param(
            "~trajectory_topic", "/drone_1_planning/autotrans_trajectory")
        raw_trajectory_topic = rospy.get_param(
            "~raw_trajectory_topic", "/drone_1_planning/trajectory")
        position_cmd_topic = rospy.get_param(
            "~position_cmd_topic", "/UAV0/planning/pos_cmd")
        planning_stop_topic = rospy.get_param(
            "~planning_stop_topic", "/UAV0/planning_stop_trigger")
        planning_restart_topic = rospy.get_param(
            "~planning_restart_topic", "/UAV0/planning_restart_trigger")
        self.write_text("trajectory_topic: %s" % trajectory_topic)
        self.write_text("trajectory_type: quadrotor_msgs/PolynomialTraj")
        self.write_text("raw_trajectory_topic_config: %s" % raw_trajectory_topic)
        self.write_text("raw_trajectory_type_expected: traj_utils/PolyTraj")
        self.write_text("position_cmd_topic: %s" % position_cmd_topic)
        self.write_text("planning_stop_topic: %s" % planning_stop_topic)
        self.write_text("planning_restart_topic: %s" % planning_restart_topic)

        rospy.Subscriber(setpoint_topic, AttitudeTarget, self.setpoint_cb, queue_size=100)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_cb, queue_size=20)
        self.position_cmd_subscriber = rospy.Subscriber(
            position_cmd_topic, PositionCommand, self.position_cmd_cb, queue_size=100)
        rospy.Subscriber(
            planning_stop_topic, Empty, self.planning_stop_cb, queue_size=20)
        rospy.Subscriber(
            planning_restart_topic, Empty, self.planning_restart_cb, queue_size=20)
        rospy.Subscriber(reference_topic, Path, self.reference_cb, queue_size=20)
        rospy.Subscriber(state_topic, State, self.state_cb, queue_size=20)
        rospy.Subscriber(extended_state_topic, ExtendedState, self.extended_state_cb, queue_size=20)
        rospy.Subscriber(rc_topic, RCIn, self.rc_cb, queue_size=20)
        rospy.Subscriber(force_topic, Accel, self.force_cb, queue_size=20)
        rospy.Subscriber(applied_force_topic, Accel, self.applied_force_cb, queue_size=20)
        rospy.Subscriber(esc_status_topic, ESCStatus, self.esc_status_cb, queue_size=20)
        rospy.Subscriber(battery_topic, BatteryState, self.battery_cb, queue_size=20)
        rospy.Subscriber(
            force_attitude_aligned_topic, Bool,
            self.force_attitude_aligned_cb, queue_size=20)
        rospy.Subscriber(
            force_attitude_yaw_offset_topic, Float64,
            self.force_attitude_yaw_offset_cb, queue_size=20)
        self.trajectory_subscriber = rospy.Subscriber(
            trajectory_topic, PolynomialTraj, self.trajectory_cb, queue_size=20)
        self.raw_trajectory_topic = raw_trajectory_topic
        self.try_subscribe_raw_trajectory()
        rospy.Subscriber("/rosout", Log, self.rosout_cb, queue_size=200)
        self.log_timer = rospy.Timer(rospy.Duration(1.0 / self.log_rate), self.log_timer_cb)

        self.start_rosbag_recording()

        rospy.loginfo("[autotrans_mpc_logger] CSV: %s", self.csv_path)
        rospy.loginfo("[autotrans_mpc_logger] LOG: %s", self.text_log_path)

    def start_rosbag_recording(self):
        """在本次 CSV 日志目录中启动同名 rosbag。"""
        if not self.enable_rosbag:
            self.write_text("rosbag_enabled: false")
            rospy.loginfo("[autotrans_mpc_logger] 自动 rosbag 已关闭")
            return
        if not self.rosbag_topics:
            self.write_text("rosbag_enabled: true")
            self.write_text("rosbag_status: not_started_no_topics")
            rospy.logwarn("[autotrans_mpc_logger] rosbag 已启用，但话题列表为空")
            return

        bag_path = os.path.join(self.run_dir, "%s.bag" % self.run_name)
        console_path = os.path.join(
            self.run_dir, "%s_rosbag.log" % self.run_name)
        command = ["rosbag", "record", "--lz4", "-O", bag_path]
        command.extend(self.rosbag_topics)
        try:
            self.rosbag_console_file = open(console_path, "a")
            self.rosbag_process = subprocess.Popen(
                command,
                stdout=self.rosbag_console_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
        except (OSError, ValueError) as exc:
            if self.rosbag_console_file is not None:
                self.rosbag_console_file.close()
                self.rosbag_console_file = None
            self.rosbag_process = None
            self.write_text("rosbag_enabled: true")
            self.write_text("rosbag_status: start_failed: %s" % exc)
            rospy.logerr("[autotrans_mpc_logger] rosbag 启动失败：%s", exc)
            return

        self.write_text("rosbag_enabled: true")
        self.write_text("rosbag_file: %s" % bag_path)
        self.write_text("rosbag_console_file: %s" % console_path)
        self.write_text("rosbag_topics: %s" % ", ".join(self.rosbag_topics))
        rospy.loginfo("[autotrans_mpc_logger] BAG: %s", bag_path)

    def stop_rosbag_recording(self):
        """让 rosbag 正常写入索引并退出，尽量不留下 .bag.active。"""
        process = self.rosbag_process
        self.rosbag_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("[autotrans_mpc_logger] rosbag 未及时退出，发送 SIGTERM")
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except OSError:
                        pass
            except OSError as exc:
                rospy.logwarn("[autotrans_mpc_logger] rosbag 无法正常停止：%s", exc)

        if self.rosbag_console_file is not None:
            self.rosbag_console_file.flush()
            self.rosbag_console_file.close()
            self.rosbag_console_file = None

    def odom_cb(self, msg):
        self.latest_odom = msg

    def goal_cb(self, msg):
        self.latest_goal = msg

    def position_cmd_cb(self, msg):
        # 记录规划器实际发送给 NMPC 的 PositionCommand；位置、速度、加速度和 jerk
        # 使用配置的世界坐标系，单位分别为 m、m/s、m/s^2 和 m/s^3。
        received_stamp = rospy.Time.now().to_sec()
        p = msg.position
        v = msg.velocity
        a = msg.acceleration
        j = msg.jerk
        with self.file_lock:
            if self.closed:
                return
            self.position_cmd_writer.writerow([
                received_stamp,
                msg.header.stamp.to_sec(),
                msg.header.seq,
                msg.header.frame_id,
                msg.trajectory_id,
                msg.trajectory_flag,
                p.x, p.y, p.z,
                v.x, v.y, v.z,
                a.x, a.y, a.z,
                j.x, j.y, j.z,
                msg.yaw,
                msg.yaw_dot,
                msg.kx[0], msg.kx[1], msg.kx[2],
                msg.kv[0], msg.kv[1], msg.kv[2],
            ])
            self.position_cmd_rows_since_flush += 1
            if self.position_cmd_rows_since_flush >= self.flush_every:
                self.position_cmd_file.flush()
                self.position_cmd_rows_since_flush = 0

    def planning_stop_cb(self, _msg):
        self.write_text("%.6f [EVENT] planning_stop" % rospy.Time.now().to_sec())

    def planning_restart_cb(self, _msg):
        self.write_text("%.6f [EVENT] planning_restart" % rospy.Time.now().to_sec())

    def reference_cb(self, msg):
        self.latest_reference = msg

    def state_cb(self, msg):
        self.latest_state = msg

    def extended_state_cb(self, msg):
        self.latest_extended_state = msg

    def rc_cb(self, msg):
        self.latest_rc = msg

    def force_cb(self, msg):
        self.latest_force = msg

    def applied_force_cb(self, msg):
        self.latest_applied_force = msg

    def esc_status_cb(self, msg):
        self.latest_esc_status = msg

    def trajectory_cb(self, msg):
        """记录桥接后实际送给 MPC 的 PolynomialTraj。"""
        received_stamp = rospy.Time.now().to_sec()
        header_stamp = msg.header.stamp.to_sec()
        pieces = list(msg.trajectory)

        with self.file_lock:
            if self.closed:
                return
            if not pieces:
                # 即使 ACTION_ABORT 没有轨迹片段，也保留这个控制事件。
                self.trajectory_writer.writerow([
                    received_stamp, header_stamp, msg.header.seq, msg.header.frame_id,
                    msg.trajectory_id, msg.action, -1, "", "", "", "[]",
                ])
            else:
                for piece_index, piece in enumerate(pieces):
                    self.trajectory_writer.writerow([
                        received_stamp,
                        header_stamp,
                        msg.header.seq,
                        msg.header.frame_id,
                        msg.trajectory_id,
                        msg.action,
                        piece_index,
                        piece.duration,
                        piece.num_order,
                        piece.num_dim,
                        json.dumps(list(piece.data), separators=(",", ":")),
                    ])

            self.trajectory_rows_since_flush += 1
            if self.trajectory_rows_since_flush >= self.flush_every:
                self.trajectory_file.flush()
                self.trajectory_rows_since_flush = 0

    def try_subscribe_raw_trajectory(self):
        """按 ROS master 中的真实类型订阅 Diff-Planner 原始 PolyTraj。"""
        if self.raw_trajectory_subscriber is not None:
            return

        try:
            message_class, resolved_topic, _ = rostopic.get_topic_class(
                self.raw_trajectory_topic, blocking=False)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[autotrans_mpc_logger] Cannot resolve raw trajectory topic %s: %s",
                self.raw_trajectory_topic,
                exc,
            )
            return

        if message_class is None:
            rospy.logwarn_throttle(
                5.0,
                "[autotrans_mpc_logger] Raw trajectory topic is not available: %s",
                self.raw_trajectory_topic,
            )
            if self.raw_trajectory_retry_timer is None:
                self.raw_trajectory_retry_timer = rospy.Timer(
                    rospy.Duration(1.0), self.raw_trajectory_retry_cb)
            return

        message_type = getattr(message_class, "_type", "")
        if message_type != "traj_utils/PolyTraj":
            rospy.logwarn(
                "[autotrans_mpc_logger] Raw trajectory topic %s has type %s; "
                "expected traj_utils/PolyTraj, raw logging disabled",
                self.raw_trajectory_topic,
                message_type,
            )
            return

        self.raw_trajectory_subscriber = rospy.Subscriber(
            resolved_topic, message_class, self.planner_trajectory_cb, queue_size=20)
        self.write_text("raw_trajectory_topic: %s" % resolved_topic)
        self.write_text("raw_trajectory_type: %s" % message_type)
        if self.raw_trajectory_retry_timer is not None:
            self.raw_trajectory_retry_timer.shutdown()
            self.raw_trajectory_retry_timer = None

    def raw_trajectory_retry_cb(self, _event):
        self.try_subscribe_raw_trajectory()

    def planner_trajectory_cb(self, msg):
        """记录 Diff-Planner 的原始 traj_utils/PolyTraj，不改变系数排列。"""
        received_stamp = rospy.Time.now().to_sec()
        start_time = msg.start_time.to_sec()
        with self.file_lock:
            if self.closed:
                return
            self.planner_trajectory_writer.writerow([
                received_stamp,
                msg.drone_id,
                msg.traj_id,
                start_time,
                msg.order,
                json.dumps(list(msg.coef_x), separators=(",", ":")),
                json.dumps(list(msg.coef_y), separators=(",", ":")),
                json.dumps(list(msg.coef_z), separators=(",", ":")),
                json.dumps(list(msg.duration), separators=(",", ":")),
            ])
            self.planner_trajectory_rows_since_flush += 1
            if self.planner_trajectory_rows_since_flush >= self.flush_every:
                self.planner_trajectory_file.flush()
                self.planner_trajectory_rows_since_flush = 0

    def battery_cb(self, msg):
        # 与控制器一致，优先将各单体电压相加得到电池包总电压。
        # 首帧直接初始化，避免滤波器从 0 V 收敛产生无物理意义的启动值。
        if not msg.cell_voltage:
            return
        cell_voltage = list(msg.cell_voltage)
        if not all(math.isfinite(value) and value > 0.0 for value in cell_voltage):
            return
        if not math.isfinite(msg.percentage) or msg.percentage < 0.0:
            return

        total_voltage = sum(cell_voltage)
        if self.filtered_battery_voltage is None:
            self.filtered_battery_voltage = total_voltage
        else:
            self.filtered_battery_voltage = (
                0.8 * self.filtered_battery_voltage + 0.2 * total_voltage)
        self.latest_battery_percentage = msg.percentage

    def force_attitude_aligned_cb(self, msg):
        # 该布尔量只描述外力估计姿态是否完成世界系偏航对齐，不改变 NMPC 状态来源。
        self.latest_force_attitude_aligned = bool(msg.data)

    def force_attitude_yaw_offset_cb(self, msg):
        # 偏航差单位 rad；非有限诊断值留空，避免离线分析把 NaN 当作有效标定结果。
        self.latest_force_attitude_yaw_offset = (
            float(msg.data) if math.isfinite(msg.data) else None)

    def setpoint_cb(self, msg):
        self.latest_setpoint = msg
        self.latest_setpoint_received = rospy.Time.now()

    def log_timer_cb(self, _event):
        msg = self.latest_setpoint
        odom_values, actual_rpy = self.extract_odom_values()
        goal_values = self.extract_goal_values()
        reference_values = self.extract_reference_values()
        error_values = self.compute_error_values(odom_values, reference_values)
        state_values = self.extract_state_values()
        rc_values = self.extract_rc_values()
        force_values = self.extract_force_values()
        applied_force_values = self.extract_applied_force_values()
        rpm_values = self.extract_rpm_values()
        battery_values = self.extract_battery_values()
        force_attitude_alignment_values = self.extract_force_attitude_alignment_values()

        setpoint_fresh = msg is not None and (
            rospy.Time.now() - self.latest_setpoint_received).to_sec() < self.setpoint_timeout
        sample_stamp = rospy.Time.now().to_sec()
        if not setpoint_fresh:
            setpoint_values = [sample_stamp, "", "", "", "", "", "", ""]
        else:
            setpoint_values = [
                sample_stamp,
                msg.header.seq,
                msg.header.frame_id,
                msg.type_mask,
                msg.body_rate.x,
                msg.body_rate.y,
                msg.body_rate.z,
                msg.thrust,
            ]

        with self.file_lock:
            if self.closed:
                return
            self.writer.writerow(self.compose_csv_row(
                setpoint_values,
                odom_values,
                actual_rpy,
                goal_values,
                reference_values,
                error_values,
                state_values,
                rc_values,
                force_values,
                applied_force_values,
                rpm_values,
                battery_values,
                force_attitude_alignment_values))

            self.rows_since_flush += 1
            if self.rows_since_flush >= self.flush_every:
                self.csv_file.flush()
                self.text_file.flush()
                self.rows_since_flush = 0

    def extract_odom_values(self):
        if self.latest_odom is None:
            return ["", "", "", "", "", ""], ["", "", ""]

        p = self.latest_odom.pose.pose.position
        v = self.latest_odom.twist.twist.linear
        q = self.latest_odom.pose.pose.orientation
        odom_values = [p.x, p.y, p.z, v.x, v.y, v.z]
        actual_rpy = self.quaternion_to_rpy(q.x, q.y, q.z, q.w)
        return odom_values, actual_rpy

    def extract_goal_values(self):
        if self.latest_goal is None:
            return ["", "", ""]
        p = self.latest_goal.pose.position
        return [p.x, p.y, p.z]

    def extract_reference_values(self):
        if self.latest_reference is None or not self.latest_reference.poses:
            return ["", "", "", "", "", "", ""]
        pose = self.latest_reference.poses[0].pose
        p = pose.position
        q = pose.orientation
        return [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    def compute_error_values(self, odom_values, reference_values):
        try:
            ox, oy, oz = [float(v) for v in odom_values[:3]]
            rx, ry, rz = [float(v) for v in reference_values[:3]]
        except (TypeError, ValueError):
            return ["", "", "", "", ""]
        ex = ox - rx
        ey = oy - ry
        ez = oz - rz
        err_xy = math.sqrt(ex * ex + ey * ey)
        err_norm = math.sqrt(ex * ex + ey * ey + ez * ez)
        return [ex, ey, ez, err_xy, err_norm]

    def extract_state_values(self):
        if self.latest_state is None:
            state_values = ["", "", "", ""]
        else:
            state_values = [
                self.latest_state.mode,
                int(self.latest_state.armed),
                int(self.latest_state.connected),
                int(self.latest_state.guided),
            ]

        if self.latest_extended_state is None:
            return state_values + [""]
        return state_values + [self.latest_extended_state.landed_state]

    def extract_rc_values(self):
        values = ["", "", "", "", "", ""]
        if self.latest_rc is None:
            return values
        channels = self.latest_rc.channels
        indices = [0, 1, 2, 3, 7, 9]
        for out_i, ch_i in enumerate(indices):
            if len(channels) > ch_i:
                values[out_i] = channels[ch_i]
        return values

    def extract_force_values(self):
        if self.latest_force is None:
            return ["", "", "", ""]
        f = self.latest_force.linear
        force_norm = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        return [f.x, f.y, f.z, force_norm]

    def extract_applied_force_values(self):
        if self.latest_applied_force is None:
            return ["", "", "", ""]
        f = self.latest_applied_force.linear
        force_norm = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        return [f.x, f.y, f.z, force_norm]

    def extract_rpm_values(self):
        values = ["", "", "", ""]
        if self.latest_esc_status is None:
            return values
        for i in range(min(4, len(self.latest_esc_status.esc_status))):
            values[i] = self.latest_esc_status.esc_status[i].rpm
        return values

    def extract_battery_values(self):
        if (self.filtered_battery_voltage is None or
                self.latest_battery_percentage is None):
            return ["", ""]
        return [self.filtered_battery_voltage, self.latest_battery_percentage]

    def extract_force_attitude_alignment_values(self):
        if self.latest_force_attitude_aligned is None:
            aligned_value = ""
        else:
            aligned_value = int(self.latest_force_attitude_aligned)
        yaw_offset_value = (
            "" if self.latest_force_attitude_yaw_offset is None
            else self.latest_force_attitude_yaw_offset)
        return [aligned_value, yaw_offset_value]

    def rosout_cb(self, msg):
        if not self.log_all_rosout and msg.name not in [
            "/mpc_controller_node",
            "/autotrans_mpc_logger",
        ]:
            return

        level_name = {
            Log.DEBUG: "DEBUG",
            Log.INFO: "INFO",
            Log.WARN: "WARN",
            Log.ERROR: "ERROR",
            Log.FATAL: "FATAL",
        }.get(msg.level, str(msg.level))
        self.write_text("%.6f [%s] %s: %s" %
                        (msg.header.stamp.to_sec(), level_name, msg.name, msg.msg))

    def write_text(self, line):
        with self.file_lock:
            if self.closed:
                return
            self.text_file.write(line + "\n")
            self.text_file.flush()

    @staticmethod
    def quaternion_to_rpy(x, y, z, w):
        # 从 odom 四元数计算实际姿态角，单位 rad；ROS 常见顺序为 x,y,z,w。
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [roll, pitch, yaw]

    def close(self):
        if self.closed:
            return
        # 先让 rosbag 收到 SIGINT，完成索引并移除 .bag.active 后缀。
        self.stop_rosbag_recording()
        if self.raw_trajectory_retry_timer is not None:
            self.raw_trajectory_retry_timer.shutdown()
            self.raw_trajectory_retry_timer = None
        if self.trajectory_subscriber is not None:
            self.trajectory_subscriber.unregister()
            self.trajectory_subscriber = None
        if self.raw_trajectory_subscriber is not None:
            self.raw_trajectory_subscriber.unregister()
            self.raw_trajectory_subscriber = None
        if self.position_cmd_subscriber is not None:
            self.position_cmd_subscriber.unregister()
            self.position_cmd_subscriber = None
        if self.log_timer is not None:
            # 先停止 100 Hz 日志定时器，再关闭文件，避免回调访问已关闭的 CSV。
            self.log_timer.shutdown()
            self.log_timer = None

        with self.file_lock:
            if self.closed:
                return
            self.closed = True
            self.text_file.write("shutdown_time: %.6f\n" % rospy.Time.now().to_sec())
            self.text_file.flush()
            self.trajectory_file.flush()
            self.planner_trajectory_file.flush()
            self.position_cmd_file.flush()
            self.trajectory_file.close()
            self.planner_trajectory_file.close()
            self.position_cmd_file.close()
        self.csv_file.flush()
        self.csv_file.close()
        self.text_file.flush()
        self.text_file.close()
        if self.enable_evo_report:
            report_script = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "generate_evo_report.py")
            try:
                subprocess.run(
                    [sys.executable, report_script, "--run-dir", self.run_dir],
                    check=False,
                    timeout=180.0,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                rospy.logwarn("[autotrans_mpc_logger] Evo 报告生成失败：%s", exc)


if __name__ == "__main__":
    rospy.init_node("autotrans_mpc_logger")
    logger = AutoTransMpcLogger()
    rospy.on_shutdown(logger.close)
    rospy.spin()
