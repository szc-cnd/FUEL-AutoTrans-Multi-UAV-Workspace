from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src/reference_bridge_node.cpp").read_text()
FUEL_SOURCE = (PACKAGE / "src/fuel_autotrans_bridge_node.cpp").read_text()
AUTOTRANS_POLYNOMIAL = (
    PACKAGE.parent
    / "AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/include"
    / "payload_mpc_controller/polynomial_trajectory.h"
).read_text()


def test_diff_polynomial_trajectory_uses_two_second_loss_timeout():
    assert 'private_nh_.param("trajectory_timeout", trajectory_timeout_, 2.0)' in SOURCE
    assert "timeoutCallback" in SOURCE
    assert "ACTION_ABORT" in SOURCE


def test_launches_use_two_second_loss_timeout():
    uav0 = (PACKAGE / "launch/uav0_autotrans_controller.launch").read_text()
    uav1 = (PACKAGE / "launch/uav1_diff_autotrans.launch").read_text()
    assert '<param name="trajectory_timeout" value="2.0"/>' in uav0
    assert '<arg name="trajectory_timeout" default="2.0"/>' in uav1


def test_fuel_coefficients_match_autotrans_polynomial_column_order():
    assert "for (int i = DEGREE; i >= 0; i--)" in AUTOTRANS_POLYNOMIAL
    assert "for (int order = degree; order >= 0; --order)" in FUEL_SOURCE
    assert "output.data.push_back(coefficients(order, dim));" in FUEL_SOURCE
