#!/usr/bin/env python3
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "src" / "cxr_egoctrl_v1.cpp"
LAUNCH = Path(__file__).parents[1] / "launch" / "simple_controller.launch"


def test_landing_search_speed_is_state_gated():
    code = SOURCE.read_text(encoding="utf-8")
    assert "landing_search_state_cb" in code
    assert "isLandingSearchHighSpeedState" in code
    assert 'state == "FRONT_ARUCO_HINT_DIFF_APPROACH"' in code
    assert 'state == "FRONT_ARUCO_YAW_SCAN_LEFT"' in code
    assert 'state == "FRONT_ARUCO_YAW_SCAN_RIGHT"' in code
    assert 'state == "ARUCO_LOCKED_DIFF_APPROACH"' in code
    assert "landing_search_max_cmd_speed_xy" in code
    assert "landing_search_max_reverse_speed" in code
    assert "landing_search_speed_active && !landing_requested" in code


def test_platform_handoff_does_not_bypass_precision_landing():
    code = SOURCE.read_text(encoding="utf-8")
    callback = code.split("void Ctrl::landing_request_cb", maxsplit=1)[1].split(
        "void Ctrl::landing_search_state_cb", maxsplit=1
    )[0]
    control_prefix = code.split("void Ctrl::control", maxsplit=1)[1].split(
        "if (safety_hold_active)", maxsplit=1
    )[0]

    assert "landing_requested = true" in callback
    assert "safety_hold_active = true" in callback
    assert "AUTO.LAND" not in control_prefix
    assert "SetMode" not in code


def test_landing_search_speed_defaults_are_wired_in_launch():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'name="landing_search_state_topic"' in launch
    assert 'name="max_cmd_speed_xy" value="0.20"' in launch
    assert 'name="landing_search_max_cmd_speed_xy" value="0.50"' in launch
