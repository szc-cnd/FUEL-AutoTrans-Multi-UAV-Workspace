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


def test_dual_uav_accepts_relay_trajectory_after_both_reach_030m():
    assert 'name="leader_start_height" value="0.3"' in LAUNCH
    assert 'name="follower_start_height" value="0.3"' in LAUNCH
    assert 'name="min_record_height" value="0.3"' in LAUNCH
    assert 'pnh_.param("leader_start_height", leader_start_height_, 0.3)' in SOURCE
    assert 'pnh_.param("follower_start_height", follower_start_height_, 0.3)' in SOURCE
    assert 'pnh_.param("min_record_height", min_record_height_, 0.30)' in SOURCE


def test_diff_planning_failure_reselects_route_point_after_retreat():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "bool getLaggedTarget", maxsplit=1
    )[0]
    failure_handler = SOURCE.split(
        "void handleDiffPlanningFailure", maxsplit=1
    )[1].split("void leaderTaskStatusCallback", maxsplit=1)[0]
    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]

    assert "handleDiffPlanningFailure(status" in status_callback
    assert "blacklistRouteCandidate" in failure_handler
    assert "diff_forward_failures_before_retreat_" in failure_handler
    assert "getFollowerHistoryRetreatTarget" in execution
    assert '"failure retreat reached; select new route subgoal"' in execution
    assert "if (completed_retreat) return true;" in execution
    assert "DIFF_FAILURE_RETREAT_EXHAUSTED" not in execution
    assert "operator recovery" not in execution


def test_failure_retreat_reaches_full_distance_and_stops_before_replan():
    assert 'name="diff_failure_retreat_distance" value="0.40"' in LAUNCH
    assert 'name="diff_failure_retreat_arrive_radius" value="0.10"' in LAUNCH
    assert 'name="follower_history_sample_spacing" value="0.08"' in LAUNCH
    assert 'name="follower_history_max_length" value="30.0"' in LAUNCH
    assert "recovery_error <= diff_failure_retreat_arrive_radius_" in SOURCE
    assert "follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_" in SOURCE
    assert "follower_vertical_speed_ <= relay_arrive_max_vertical_speed_" in SOURCE


def test_outside_map_uses_forward_recovery_instead_of_retreat():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "bool getLaggedTarget", maxsplit=1
    )[0]
    failure_handler = SOURCE.split(
        "void handleDiffPlanningFailure", maxsplit=1
    )[1].split("void leaderTaskStatusCallback", maxsplit=1)[0]
    assert 'status == "GOAL_REJECTED_OUTSIDE_MAP"' in status_callback
    assert '"select another UAV0-history candidate"' in failure_handler


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
    assert "diff_forward_candidate_failures_ = 0;" in route_completion


def test_legacy_nonterminal_diff_goal_still_requires_verified_history():
    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]
    assert "return handleSimpleDiffPlannerExecution(now);" in execution
    assert "selectForwardRouteCandidate" in execution
    assert '"nonterminal relay requires verified history candidate"' in execution
    selector = SOURCE.split("bool selectForwardRouteCandidate", maxsplit=1)[1].split(
        "bool getFollowerHistoryRetreatTarget", maxsplit=1
    )[0]
    assert "plausible_progress_upper" in selector
    assert "diff_allow_route_backtrack_attachment_" in selector


def test_legacy_diff_prefers_confirmed_leader_segment_endpoints_within_7m():
    selector = SOURCE.split("bool selectForwardRouteCandidate", maxsplit=1)[1].split(
        "bool getFollowerHistoryRetreatTarget", maxsplit=1
    )[0]
    trajectory_callback = SOURCE.split(
        "void leaderTrajectoryCallback", maxsplit=1
    )[1].split("void confirmPendingLeaderSegmentEndpoint", maxsplit=1)[0]
    endpoint_confirmation = SOURCE.split(
        "void confirmPendingLeaderSegmentEndpoint", maxsplit=1
    )[1].split("void leaderOdomCallback", maxsplit=1)[0]

    assert 'name="leader_trajectory_topic" value="$(arg leader_trajectory_topic)"' in LAUNCH
    assert 'name="diff_history_target_max_distance" value="7.00"' in LAUNCH
    assert "evaluateDeBoor(trajectory_end)" in trajectory_callback
    assert "leader_segment_endpoint_max_speed_" in endpoint_confirmation
    assert "leader_segment_endpoint_dwell_" in endpoint_confirmation
    assert "leader_segment_endpoints_.rbegin()" in selector
    assert 'accept_candidate(candidate, "stopped-segment-endpoint")' in selector
    assert "current_progress + diff_history_target_max_distance_" in selector
    assert "route_.back().progress" in selector
    assert "segment_end_progress" not in selector
    assert "step_ratios" in selector
    assert "step = std::min(step, max_target_step_)" not in selector

    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]
    assert "remaining_relay_progress" in execution
    assert "remaining_relay_progress <= path_sample_spacing_" in execution


def test_simple_mode_queues_each_confirmed_segment_endpoint_as_fifo():
    endpoint_confirmation = SOURCE.split(
        "void confirmPendingLeaderSegmentEndpoint", maxsplit=1
    )[1].split("void leaderOdomCallback", maxsplit=1)[0]
    odom_callback = SOURCE.split("void leaderOdomCallback", maxsplit=1)[1].split(
        "void appendFollowerExecutedPoint", maxsplit=1
    )[0]

    assert 'name="simple_segment_endpoint_following" value="true"' in LAUNCH
    assert 'name="simple_diff_retry_delay" value="0.20"' in LAUNCH
    assert 'appendRelayWaypoint(confirmed, "SEGMENT_ENDPOINT")' in endpoint_confirmation
    assert "door_waypoint_released_" in endpoint_confirmation
    assert "!exit_waypoint_released_" in endpoint_confirmation
    assert "if (simple_segment_endpoint_following_) return;" in odom_callback


def test_simple_mode_gives_diff_only_the_fifo_front_and_consumes_on_arrival():
    execution = SOURCE.split(
        "bool handleSimpleDiffPlannerExecution", maxsplit=1
    )[1].split("bool getLaggedTarget", maxsplit=1)[0]
    consume = SOURCE.split("void consumeSimpleRelayFront", maxsplit=1)[1].split(
        "bool handleSimpleDiffPlannerExecution", maxsplit=1
    )[0]

    assert "const RoutePoint desired_world = relay_waypoints_[active_relay_index_]" in execution
    assert "goal.pose.position = desired_local;" in execution
    assert "selectForwardRouteCandidate" not in execution
    assert "diff_recovery_goal_local_" not in execution
    assert "blacklistRouteCandidate" not in execution
    assert "consumeSimpleRelayFront();" in execution
    assert "relay_waypoints_.erase" in consume
    assert "const bool waypoint_arrival = !terminal_relay" in execution
    assert 'name="relay_arrive_radius" value="0.40"' in LAUNCH
    ordinary_arrival = execution.split(
        "const bool waypoint_arrival", maxsplit=1
    )[1].split("const bool terminal_arrival", maxsplit=1)[0]
    assert "horizontal_error <= relay_arrive_radius_" in ordinary_arrival
    assert "follower_horizontal_speed_" not in ordinary_arrival
    assert "diff_endpoint_capture_radius_" not in ordinary_arrival
    terminal_arrival = execution.split(
        "const bool terminal_arrival", maxsplit=1
    )[1].split("if (waypoint_arrival", maxsplit=1)[0]
    assert "horizontal_error <= terminal_arrive_radius_" in terminal_arrival


def test_simple_mode_skips_occupied_endpoint_and_waits_for_leader_to_clear_next():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "void consumeSimpleRelayFront", maxsplit=1
    )[0]
    execution = SOURCE.split(
        "bool handleSimpleDiffPlannerExecution", maxsplit=1
    )[1].split("bool getLaggedTarget", maxsplit=1)[0]

    assert 'status == "OCCUPIED_RECOVERY_SUCCEEDED"' in status_callback
    assert "consumeSimpleRelayFront();" in status_callback
    assert "simple_occupied_recovery_active_" in status_callback
    assert "leader_to_waypoint" in execution
    assert "leader_world.x - desired_world.position.x" in execution
    assert "leader_to_waypoint + 1.0e-6 < waypoint_release_min_separation_" in execution
    assert "DIFF_FIFO_WAIT_LEADER_CLEAR_NEXT_POINT" in execution
    assert 'name="waypoint_release_min_separation" value="1.50"' in LAUNCH


def test_simple_mode_has_unbounded_dynamic_waypoint_queue():
    assert 'name="max_internal_relay_points" value="0"' in LAUNCH
    endpoint_confirmation = SOURCE.split(
        "void confirmPendingLeaderSegmentEndpoint", maxsplit=1
    )[1].split("void leaderOdomCallback", maxsplit=1)[0]
    assert 'appendRelayWaypoint(confirmed, "SEGMENT_ENDPOINT")' in endpoint_confirmation
    assert "max_internal_relay_points_" not in endpoint_confirmation


def test_simple_mode_retries_same_endpoint_without_wrapper_recovery():
    status_callback = SOURCE.split("void diffStatusCallback", maxsplit=1)[1].split(
        "void consumeSimpleRelayFront", maxsplit=1
    )[0]
    timer = SOURCE.split("void timerCallback", maxsplit=1)[1].split(
        "void hold", maxsplit=1
    )[0]

    assert '"retry the same coordinates (no alternate/recovery point)."' in status_callback
    simple_failure = status_callback.split(
        "if (simple_segment_endpoint_following_)", maxsplit=1
    )[1].split("return;", maxsplit=1)[0]
    assert "handleDiffPlanningFailure" not in simple_failure
    assert "blacklistRouteCandidate" not in simple_failure
    assert "diff_recovery_requested_" not in simple_failure
    assert "!simple_segment_endpoint_following_ && handleStuckRecovery(now)" in timer


def test_candidate_blacklist_and_clipped_retry_are_bounded():
    assert 'name="diff_candidate_blacklist_duration" value="4.0"' in LAUNCH
    assert 'name="diff_forward_failures_before_retreat" value="3"' in LAUNCH
    assert 'name="diff_clipped_retry_limit" value="2"' in LAUNCH
    assert "temporarily blacklist route candidate" in SOURCE
    assert "clipped endpoint retry limit reached" in SOURCE
    assert "stale-command retry limit reached" in SOURCE
    assert "if (reselect_after_stale) return true;" in SOURCE


def test_planner_no_response_has_total_timeout_and_switches_candidate():
    execution = SOURCE.split("bool handleDiffPlannerExecution", maxsplit=1)[1].split(
        "void timerCallback", maxsplit=1
    )[0]
    assert 'name="diff_goal_response_timeout" value="2.0"' in LAUNCH
    assert "diff_goal_first_publish_stamp_" in execution
    assert "TOTAL RESPONSE TIMEOUT" in execution
    assert 'handleDiffPlanningFailure("PLANNER_RESPONSE_TIMEOUT", now)' in execution
    assert "if (starts_new_response_window) diff_goal_first_publish_stamp_ = now;" in execution


def test_follower_odom_fault_recovers_without_restart():
    callback = SOURCE.split("void followerOdomCallback", maxsplit=1)[1].split(
        "void cloudCallback", maxsplit=1
    )[0]
    assert "follower_odom_recovery_good_samples_ >= 10" in callback
    assert "follower_odom_fault_latched_ = false;" in callback
    assert "request a fresh Diff trajectory" in callback
