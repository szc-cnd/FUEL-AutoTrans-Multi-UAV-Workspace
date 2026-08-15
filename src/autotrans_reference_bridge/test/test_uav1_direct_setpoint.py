#!/usr/bin/env python3
"""Static contract for the CAV1 AutoTrans MAVROS output path."""

from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "uav1_diff_autotrans.launch"


def test_uav1_controller_publishes_directly_to_mavros():
    launch = LAUNCH.read_text(encoding="utf-8")

    assert (
        '<arg name="setpoint_topic" '
        'default="/UAV1/mavros/setpoint_raw/attitude"/>'
    ) in launch
    assert 'default="/UAV1/control/attitude_setpoint"' not in launch


if __name__ == "__main__":
    test_uav1_controller_publishes_directly_to_mavros()
    print("uav1 direct setpoint contract: PASS")
