from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src/reference_bridge_node.cpp").read_text()


def test_diff_polynomial_trajectory_uses_two_second_loss_timeout():
    assert 'private_nh_.param("trajectory_timeout", trajectory_timeout_, 2.0)' in SOURCE
    assert "timeoutCallback" in SOURCE
    assert "ACTION_ABORT" in SOURCE


def test_launches_use_two_second_loss_timeout():
    uav0 = (PACKAGE / "launch/uav0_autotrans_controller.launch").read_text()
    uav1 = (PACKAGE / "launch/uav1_diff_autotrans.launch").read_text()
    assert '<param name="trajectory_timeout" value="2.0"/>' in uav0
    assert '<arg name="trajectory_timeout" default="2.0"/>' in uav1
