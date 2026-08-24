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


def test_diff_planning_failure_reselects_route_point_after_retreat():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "bool getLaggedTarget", maxsplit=1
    )[0]
    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]

    assert 'diff_recovery_retreat_requested_ = status == "PLANNING_FAILED"' in status_callback
    assert 'if (status == "PLANNING_FAILED") diff_route_subgoal_valid_ = false;' in status_callback
    assert "getDiffFailureRetreatTarget" in execution
    assert '"failure retreat reached; select new route subgoal"' in execution
    assert "if (completed_retreat) return true;" in execution
    assert "diff_failure_retreat_attempts_ >= diff_failure_retreat_max_attempts_" in execution


def test_failure_retreat_reaches_full_distance_and_stops_before_replan():
    assert 'name="diff_failure_retreat_distance" value="0.40"' in LAUNCH
    assert 'name="diff_failure_retreat_arrive_radius" value="0.10"' in LAUNCH
    assert 'name="diff_failure_retreat_max_attempts" value="2"' in LAUNCH
    assert "recovery_error <= diff_failure_retreat_arrive_radius_" in SOURCE
    assert "follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_" in SOURCE
    assert "follower_vertical_speed_ <= relay_arrive_max_vertical_speed_" in SOURCE


def test_outside_map_uses_forward_recovery_instead_of_retreat():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "bool getLaggedTarget", maxsplit=1
    )[0]
    assert 'diff_recovery_retreat_requested_ = status == "PLANNING_FAILED"' in status_callback
    assert 'status == "GOAL_REJECTED_OUTSIDE_MAP"' in status_callback
    assert 'diff_recovery_retreat_requested_ ? "retreat" : "forward"' in status_callback


def test_matching_delayed_success_is_not_misclassified_as_stale():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "bool getLaggedTarget", maxsplit=1
    )[0]
    assert "late_success_for_active_goal" in status_callback
    assert "response_matches_active_goal && !diff_goal_published_" in status_callback
    assert "diff_goal_published_ = true;" in status_callback


def test_successful_new_route_subgoal_resets_failure_budget():
    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]
    route_completion = execution.split(
        "if (route_subgoal_completed)", maxsplit=1
    )[1].split("relay_arrival_stamp_", maxsplit=1)[0]
    assert "diff_route_subgoal_valid_ = false;" in route_completion
    assert "diff_failure_retreat_attempts_ = 0;" in route_completion
