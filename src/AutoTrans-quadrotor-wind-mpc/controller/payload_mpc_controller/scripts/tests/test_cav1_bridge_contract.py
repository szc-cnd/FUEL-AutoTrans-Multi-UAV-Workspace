#!/usr/bin/env python3
"""Static regression checks for the CAV1 Diff-Planner bridge contract."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT.parent.parent.parent / "autotrans_reference_bridge" / "src" / "reference_bridge_node.cpp"
LAUNCH = ROOT.parent.parent.parent / "autotrans_reference_bridge" / "launch" / "uav1_diff_autotrans.launch"


def test_bridge_does_not_abort_on_missing_replan():
    text = BRIDGE.read_text(encoding="utf-8")
    assert "trajectory_timeout" not in text
    assert "ACTION_ABORT" not in text


def test_bridge_output_is_not_latched():
    text = BRIDGE.read_text(encoding="utf-8")
    assert "advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, true)" not in text
    assert "advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, false)" in text


def test_bridge_preserves_matrix_column_major_layout():
    text = BRIDGE.read_text(encoding="utf-8")
    assert "Eigen::Map 按列主序恢复" in text
    assert "piece.data.push_back(input->coef_x[index])" in text
    assert "piece.data.push_back(input->coef_y[index])" in text
    assert "piece.data.push_back(input->coef_z[index])" in text


def test_uav1_launch_keeps_diff_planner_only_input():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "/drone_1_planning/autotrans_trajectory" in text
    assert "trajectory_timeout" not in text
