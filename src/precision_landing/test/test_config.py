#!/usr/bin/env python3

"""Contract tests for the shipped precision-landing configuration."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "precision_landing.yaml"
NODE_SOURCE_PATH = (
    Path(__file__).resolve().parents[1] / "src" / "precision_landing_node.cpp"
)
STATE_MACHINE_SOURCE_PATH = (
    Path(__file__).resolve().parents[1] / "src" / "landing_state_machine.cpp"
)
PACKAGE_XML_PATH = Path(__file__).resolve().parents[1] / "package.xml"
CMAKE_PATH = Path(__file__).resolve().parents[1] / "CMakeLists.txt"
ROSTEST_LAUNCH_PATH = (
    Path(__file__).resolve().parent / "precision_landing_node.test"
)
ROSTEST_SOURCE_PATH = (
    Path(__file__).resolve().parent / "test_precision_landing_node.py"
)
LANDING_TEST_NODE_PATH = (
    Path(__file__).resolve().parents[1] / "src" / "landing_test_mission_node.cpp"
)
LANDING_TEST_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "landing_test.launch"
)


def _leaf_paths(value, prefix=""):
    if isinstance(value, dict):
        for key, nested_value in value.items():
            nested_prefix = f"{prefix}/{key}" if prefix else key
            yield from _leaf_paths(nested_value, nested_prefix)
    else:
        yield prefix


def test_conservative_configuration_contract():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)

    assert cfg["marker"]["dictionary"] == "DICT_4X4_250"
    assert cfg["marker"]["size_m"] == 0.60
    assert cfg["topics"]["trigger"] == "/need_to_land"
    assert cfg["topics"]["image"] == "/usb_cam/image_raw"
    assert cfg["control"]["publish_rate_hz"] >= 20.0
    assert cfg["safety"]["require_camera_info"] is True
    assert cfg["safety"]["near_ground_loss_height_m"] == 1.20
    assert cfg["stages"]["high"]["min_height_m"] == 1.50
    assert cfg["stages"]["mid"]["min_height_m"] == 1.30
    assert cfg["stages"]["final"]["min_height_m"] == 1.20
    assert cfg["stages"]["auto_land"]["height_m"] == 1.20
    assert cfg["stages"]["auto_land"]["error_m"] == 0.08


def test_real_vehicle_landing_test_auto_starts_after_safe_readiness_delay():
    node_source = LANDING_TEST_NODE_PATH.read_text(encoding="utf-8")
    launch_source = LANDING_TEST_LAUNCH_PATH.read_text(encoding="utf-8")
    cmake_source = CMAKE_PATH.read_text(encoding="utf-8")
    package_source = PACKAGE_XML_PATH.read_text(encoding="utf-8")

    ET.parse(LANDING_TEST_LAUNCH_PATH)
    assert 'param<bool>("auto_start", false)' in node_source
    assert '"auto_start_delay_sec", 5.0' in node_source
    assert "autoStartReady" in node_source
    assert 'serviceClient<mavros_msgs::CommandBool>' in node_source
    assert 'serviceClient<mavros_msgs::SetMode>' in node_source
    assert '"/need_to_land"' in node_source
    assert "landingStateOwnsSetpoints" in node_source
    assert '<arg name="allow_arming" default="true"/>' in launch_source
    assert '<arg name="auto_start" default="true"/>' in launch_source
    assert '<arg name="auto_start_delay_sec" default="5.0"/>' in launch_source
    assert '<arg name="takeoff_height_m" default="2.00"/>' in launch_source
    assert "add_executable(landing_test_mission_node" in cmake_source
    assert "<depend>std_srvs</depend>" in package_source


def test_mavros_state_and_local_pose_use_independent_freshness_limits():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)
    node_source = LANDING_TEST_NODE_PATH.read_text(encoding="utf-8")
    launch_source = LANDING_TEST_LAUNCH_PATH.read_text(encoding="utf-8")

    assert cfg["safety"]["state_timeout_sec"] == 1.50
    assert cfg["safety"]["pose_timeout_sec"] == 0.30
    assert '"state_timeout_sec", 1.50' in node_source
    assert '"pose_timeout_sec", 0.50' in node_source
    assert "state_timeout_sec_" in node_source
    assert "pose_timeout_sec_" in node_source
    assert "input_timeout_sec_" not in node_source
    assert '<param name="state_timeout_sec" value="1.50"/>' in launch_source
    assert '<param name="pose_timeout_sec" value="0.50"/>' in launch_source
    assert "input_timeout_sec" not in launch_source


def test_align_and_descent_near_ground_loss_share_direct_auto_land_branch():
    state_source = STATE_MACHINE_SOURCE_PATH.read_text(encoding="utf-8")
    visual_control_branch = state_source.split(
        "(state_ == LandingState::ALIGN || isDescentState(state_))",
        maxsplit=1,
    )[1].split(
        "} else if (!entered_precheck && state_ == LandingState::REACQUIRE)",
        maxsplit=1,
    )[0]

    assert "config_.target_loss_timeout_sec" in visual_control_branch
    assert "config_.near_ground_loss_height_m" in visual_control_branch
    assert (
        "input.marker_height_m <=\n"
        "            config_.near_ground_loss_height_m"
        in visual_control_branch
    )
    assert "state_ = LandingState::REQUEST_AUTO_LAND;" in visual_control_branch
    assert "isDescentState(state_) &&" not in visual_control_branch


def test_every_shipped_leaf_is_a_canonical_node_private_parameter():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    expected_parameters = {
        "topics/trigger",
        "topics/target_id",
        "topics/image",
        "topics/camera_info",
        "topics/local_pose",
        "topics/mavros_state",
        "topics/setpoint",
        "topics/set_mode_service",
        "marker/dictionary",
        "marker/size_m",
        "marker/requested_id",
        "marker/stable_frames",
        "marker/max_reprojection_error_px",
        "marker/min_distance_m",
        "marker/max_distance_m",
        "marker/max_position_jump_m",
        "marker/max_tilt_deg",
        "control/publish_rate_hz",
        "control/kp_xy",
        "control/max_xy_speed_mps",
        "control/max_xy_accel_mps2",
        "control/error_deadband_m",
        "control/filter_alpha",
        "safety/require_camera_info",
        "safety/image_timeout_sec",
        "safety/pose_timeout_sec",
        "safety/state_timeout_sec",
        "safety/max_header_future_sec",
        "safety/target_loss_timeout_sec",
        "safety/reacquire_timeout_sec",
        "safety/total_timeout_sec",
        "safety/near_ground_loss_height_m",
        "stages/high/min_height_m",
        "stages/high/error_m",
        "stages/high/descent_mps",
        "stages/mid/min_height_m",
        "stages/mid/error_m",
        "stages/mid/descent_mps",
        "stages/final/min_height_m",
        "stages/final/error_m",
        "stages/final/descent_mps",
        "stages/auto_land/height_m",
        "stages/auto_land/error_m",
        "stages/auto_land/stable_sec",
        "camera_to_body/rotation",
        "camera_to_body/translation_m",
    }

    assert set(_leaf_paths(cfg)) == expected_parameters
    assert all(parameter in node_source for parameter in expected_parameters)


def test_camera_info_requirement_cannot_be_disabled():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    assert cfg["safety"]["require_camera_info"] is True
    assert "if (!require_camera_info_)" in node_source
    assert "~safety/require_camera_info must be true" in node_source


def test_downward_optical_to_body_flu_rotation_has_correct_axes_and_height():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)
    rotation = cfg["camera_to_body"]["rotation"]

    assert rotation == [
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]

    def rotate(vector):
        return [
            sum(rotation[row][column] * vector[column] for column in range(3))
            for row in range(3)
        ]

    image_right_body = rotate([1.0, 0.0, 1.0])
    image_top_body = rotate([0.0, -1.0, 1.0])
    centered_body = rotate([0.0, 0.0, 2.0])
    assert image_right_body[1] < 0.0  # FLU negative-Y is aircraft-right.
    assert image_top_body[0] > 0.0  # FLU positive-X is aircraft-forward.
    assert -centered_body[2] == 2.0  # Downward distance remains positive.

    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")
    assert "marker_height_m = -position_body.z();" in node_source


def test_node_separates_held_yaw_from_live_body_to_enu_yaw():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    assert "current_vehicle_yaw_rad" in node_source
    assert "current_error_body, current_vehicle_yaw_rad" in node_source
    assert "publishSetpoint(velocity_enu, captured_yaw_rad_, now)" in node_source


def test_precheck_captures_first_later_fresh_pose_yaw_once_per_mission():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")
    pose_callback = node_source.split(
        "void poseCallback", maxsplit=1
    )[1].split("void mavrosStateCallback", maxsplit=1)[0]

    assert "captureYawDuringPrecheck" in node_source
    assert "captureYawDuringPrecheck" in pose_callback
    assert "!yaw_captured_" in node_source
    assert "previous_state_ == LandingState::PRECHECK" in node_source
    assert "current_error_body, current_vehicle_yaw_rad" in node_source


def test_landing_lock_is_reset_and_requested_id_is_frozen_at_mission_start():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")
    trigger_callback = node_source.split(
        "void triggerCallback", maxsplit=1
    )[1].split("void targetIdCallback", maxsplit=1)[0]

    assert "if (rising_edge" in trigger_callback
    assert "tracker_.reset();" in trigger_callback
    assert "controller_.reset();" in trigger_callback
    assert "active_requested_id_ = next_requested_id_;" in trigger_callback
    assert "latest_observation_ = TargetObservation();" in trigger_callback
    assert "trackerMutationAllowed() && fresh_message" in node_source
    assert "active_requested_id_, allow_lock_mutation" in node_source


def test_tracker_mutation_follows_active_mission_after_trigger_falls():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    assert "bool trackerMutationAllowed() const" in node_source
    assert "trigger_ || missionKeepsTrackerActive(previous_state_)" in node_source
    assert "trackerMutationAllowed() && fresh_message" in node_source


def test_invalid_transformed_height_never_overwrites_last_valid_height():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")
    control_callback = node_source.split(
        "void controlTimerCallback", maxsplit=1
    )[1].split("void publishSetpoint", maxsplit=1)[0]

    validation_index = control_callback.find(
        "candidate_marker_height_m <= 0.0"
    )
    guarded_cache_index = control_callback.find(
        "if (target_visible) {", validation_index
    )
    cache_index = control_callback.find(
        "last_marker_height_m_ = marker_height_m;", validation_index
    )
    current_height_assignment_index = control_callback.find(
        "marker_height_m = candidate_marker_height_m;"
    )
    assert validation_index >= 0
    assert guarded_cache_index > validation_index
    assert current_height_assignment_index > guarded_cache_index
    assert cache_index > current_height_assignment_index


def test_flight_control_parameters_fail_closed_on_invalid_values():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")
    assert "void validateControllerConfig" in node_source
    assert "void validateStateMachineConfig" in node_source
    controller_validation = node_source.split(
        "void validateControllerConfig", maxsplit=1
    )[1].split("ControllerConfig loadControllerConfig", maxsplit=1)[0]
    state_validation = node_source.split(
        "void validateStateMachineConfig", maxsplit=1
    )[1].split("StateMachineConfig loadStateMachineConfig", maxsplit=1)[0]
    runtime_loader = node_source.split(
        "void loadRuntimeParameters", maxsplit=1
    )[1].split("void configureRosInterfaces", maxsplit=1)[0]

    for member in (
        "config.kp_xy",
        "config.max_xy_speed",
        "config.max_xy_accel",
        "config.error_deadband",
        "config.filter_alpha",
    ):
        assert member in controller_validation

    for member in (
        "config.acquire_stable_sec",
        "config.align_stable_sec",
        "config.align_error_m",
        "config.high_align_error_m",
        "config.mid_align_error_m",
        "config.final_align_error_m",
        "config.auto_land_error_m",
        "config.high_descent_mps",
        "config.mid_descent_mps",
        "config.final_descent_mps",
        "config.target_loss_timeout_sec",
        "config.reacquire_timeout_sec",
        "config.total_timeout_sec",
    ):
        assert member in state_validation

    for runtime_parameter in (
        "image_timeout_sec_",
        "pose_timeout_sec_",
        "state_timeout_sec_",
        "control_rate_hz_",
        "auto_land_request_rate_hz_",
        "auto_land_authorization_delay_sec_",
    ):
        assert runtime_parameter in runtime_loader
    assert "requireFinitePositive" in runtime_loader
    assert "requireFiniteNonNegative" in runtime_loader
    assert "std::max(0.0, image_timeout_sec_)" not in runtime_loader


def test_freshness_checks_receive_and_header_age_with_explicit_stamp_policy():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        cfg = yaml.safe_load(config_file)
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    assert cfg["safety"]["max_header_future_sec"] == 0.05
    assert "headerFresh" in node_source
    assert "header_stamp.isZero()" in node_source
    assert "max_header_future_sec_" in node_source
    assert "messageFresh" in node_source
    for header_stamp in (
        "image_header_stamp_",
        "camera_info_header_stamp_",
        "pose_header_stamp_",
        "mavros_state_header_stamp_",
    ):
        assert header_stamp in node_source


def test_controller_and_tracker_resets_are_wired_to_loss_and_reacquisition():
    node_source = NODE_SOURCE_PATH.read_text(encoding="utf-8")

    assert "output.reset_controller" in node_source
    assert "output.reset_target_jump_history" in node_source
    assert "tracker_.beginReacquisition();" in node_source


def test_direct_opencv_system_dependency_is_declared():
    package = ET.parse(PACKAGE_XML_PATH).getroot()
    dependencies = {
        dependency.text
        for tag in ("depend", "build_depend", "build_export_depend", "exec_depend")
        for dependency in package.findall(tag)
    }

    assert "libopencv-dev" in dependencies


def test_rostest_node_name_is_distinct_and_service_cleanup_is_failure_safe():
    launch = ET.parse(ROSTEST_LAUNCH_PATH).getroot()
    production_node = launch.find("node")
    test_node = launch.find("test")
    test_source = ROSTEST_SOURCE_PATH.read_text(encoding="utf-8")

    assert production_node is not None
    assert test_node is not None
    assert production_node.attrib["name"] != test_node.attrib["test-name"]
    assert test_node.attrib["test-name"] == "precision_landing_node_test"
    assert 'self.addCleanup(self.mode_service.shutdown, "test complete")' in test_source


def test_rostest_marker_generation_supports_old_and_new_opencv_aruco_apis():
    test_source = ROSTEST_SOURCE_PATH.read_text(encoding="utf-8")

    assert 'hasattr(cv2.aruco, "generateImageMarker")' in test_source
    assert "cv2.aruco.generateImageMarker(" in test_source
    assert "cv2.aruco.drawMarker(" in test_source


def test_rostest_resets_between_cases_and_uses_consistent_final_gate():
    test_source = ROSTEST_SOURCE_PATH.read_text(encoding="utf-8")
    state_machine_source = STATE_MACHINE_SOURCE_PATH.read_text(encoding="utf-8")
    launch_source = ROSTEST_LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'mode="POSCTL"' in test_source
    assert "state == LandingState::PASSIVE_ABORT" in state_machine_source
    assert 'name="state_machine/final_align_error_m" value="0.20"' in launch_source
    assert 'name="state_machine/mid_align_error_m" value="0.20"' in launch_source
