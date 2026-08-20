from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src/mpc_fsm.cpp").read_text()


def test_failed_nmpc_uses_level_attitude_instead_of_zero_body_rates():
    recovery = SOURCE.split("void MPCFSM::publishMpcRecoveryAttitude", 1)[1].split(
        "void MPCFSM::beginDirectAutoLand", 1
    )[0]
    assert "IGNORE_ROLL_RATE" in recovery
    assert "IGNORE_PITCH_RATE" in recovery
    assert "IGNORE_YAW_RATE" in recovery
    assert "msg.orientation.w" in recovery
    assert "msg.orientation.z" in recovery
    assert "IGNORE_ATTITUDE" not in recovery


def test_persistent_solver_failure_requests_auto_land_quickly():
    recovery = SOURCE.split("void MPCFSM::processMpcRecovery", 1)[1].split(
        "void MPCFSM::publishMpcRecoveryAttitude", 1
    )[0]
    assert 'beginDirectAutoLand(now, "NMPC 恢复超时")' in recovery
    assert "kMpcRecoveryAutoLandSeconds = 0.30" in SOURCE
