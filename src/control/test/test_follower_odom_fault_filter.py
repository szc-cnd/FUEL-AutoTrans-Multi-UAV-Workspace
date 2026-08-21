#!/usr/bin/env python3
"""Regression checks for high-frequency follower odometry burst handling."""

from pathlib import Path
from math import hypot


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "leader_safe_path_follower.cpp").read_text(encoding="utf-8")
LAUNCH = (ROOT / "launch" / "leader_safe_path_follower.launch").read_text(
    encoding="utf-8"
)


def test_velocity_uses_message_stamp_and_requires_confirmation():
    callback = SOURCE.split("void followerOdomCallback", maxsplit=1)[1].split(
        "void cloudCallback", maxsplit=1
    )[0]
    assert "const ros::Time sample_stamp = msg->header.stamp;" in callback
    assert "sample_stamp - follower_odom_sample_stamp_" in callback
    assert "now - follower_odom_stamp_" not in callback
    assert "reject non-monotonic follower odometry stamp" in callback
    assert "follower_odom_jump_consecutive_samples_" in callback
    assert "follower_odom_jump_confirm_samples_" in callback


def test_flight_sample_is_not_a_jump_when_sensor_time_is_used():
    before = (0.047026, -0.054971, 0.504515)
    after = (0.048475, -0.054622, 0.506320)
    sensor_dt = 0.005544186
    callback_dt = 0.0013

    horizontal_delta = hypot(after[0] - before[0], after[1] - before[1])
    vertical_delta = abs(after[2] - before[2])
    assert horizontal_delta / sensor_dt < 2.0
    assert vertical_delta / sensor_dt < 1.2
    assert vertical_delta / callback_dt > 1.2


def test_launch_requires_three_consecutive_jump_samples():
    assert 'name="follower_odom_jump_confirm_samples" value="3"' in LAUNCH
