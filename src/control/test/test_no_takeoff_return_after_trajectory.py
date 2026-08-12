#!/usr/bin/env python3
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "src" / "cxr_egoctrl_v1.cpp"


def test_trajectory_timeout_does_not_reuse_takeoff_xy():
    code = SOURCE.read_text(encoding="utf-8")

    assert "bool ever_received_trajectory;" in code
    assert "ever_received_trajectory = true;" in code
    assert "if (!ever_received_trajectory)" in code
    assert "timeout_hold_x = position_x;" in code
    assert "timeout_hold_y = position_y;" in code
    assert "timeout_hold_yaw = current_yaw;" in code
    assert "hold_speed_xy_max = 0.08;" in code
    assert "hold_kp_xy * (timeout_hold_x - position_x)" in code
    assert "hold_kp_xy * (timeout_hold_y - position_y)" in code
