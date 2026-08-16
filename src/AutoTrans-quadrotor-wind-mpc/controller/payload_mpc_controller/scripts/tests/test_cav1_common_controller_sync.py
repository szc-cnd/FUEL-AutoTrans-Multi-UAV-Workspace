#!/usr/bin/env python3
"""CAV0 通用控制器改动同步到 CAV1 的静态回归检查。"""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[2]
SRC = PACKAGE / "src"
INCLUDE = PACKAGE / "include" / "payload_mpc_controller"
CONFIG = PACKAGE / "config" / "mpc.yaml"
MODEL_CONFIG = PACKAGE / "config" / "model.yaml"


def test_chinese_locale_and_startup_messages():
    node = (SRC / "mpc_controller_node.cpp").read_text(encoding="utf-8")
    assert node.index('std::setlocale(LC_ALL, "");') < node.index(
        'ros::init(argc, argv, "MPCctrl");'
    )
    assert "[启动] 等待遥控器数据" in node
    assert "Waiting for RC" not in node


def test_battery_first_sample_does_not_filter_from_zero():
    source = (SRC / "mpc_input.cpp").read_text(encoding="utf-8")
    assert "if (!std::isfinite(volt) || volt <= 0.0)" in source
    assert "volt = vlotage;" in source
    assert "volt = 0.8 * volt + 0.2 * vlotage;" in source


def test_normal_ground_rpm_does_not_spam_errors():
    source = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    assert "RPM below force_estimator/min_valid_rpm" not in source
    assert "Thrust-model gate status" not in source
    assert "门控状态：%s" in source


def test_takeoff_wait_reason_only_reports_when_changed():
    header = (INCLUDE / "mpc_fsm.h").read_text(encoding="utf-8")
    source = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    assert "last_takeoff_precondition_reason_" in header
    assert "reason != last_takeoff_precondition_reason_" in source
    assert "AUTO_TAKEOFF_WAIT_" not in source


def test_cav1_vehicle_parameters_match_latest_hover_calibration():
    config = CONFIG.read_text(encoding="utf-8")
    model_config = MODEL_CONFIG.read_text(encoding="utf-8")
    assert "mass_q: 1.817" in model_config
    for token in (
        "hover_percentage: 0.54",
        "target_z: 0.5",
        "climb_rate: 0.15",
        "Q_pos_xy:   220.0",
        "R_pitchroll:  6.0",
        "max_velocity_xy: 0.5",
        "max_velocity_z: 0.5",
        "kf: 1.790e-8",
        "force_axis_gain_x: 1.0",
        "force_axis_gain_y: 1.0",
        "force_axis_gain_z: 1.0",
        "max_force: 5.0",
        "max_applied_force: 5.0",
        "mpc_recovery_timeout: 1.0",
    ):
        assert token in config
