from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src/task_search_manager.cpp").read_text()
FSM_SOURCE = (PACKAGE / "src/fast_exploration_fsm.cpp").read_text()
MANAGER_SOURCE = (PACKAGE / "src/fast_exploration_manager.cpp").read_text()


def test_rc_trigger_requires_low_then_held_high():
    callback = SOURCE.split("void TaskSearchManager::rcSearchLandingCallback", 1)[1].split(
        "void TaskSearchManager::activateRcSearchLanding", 1
    )[0]
    assert "pwm <= rc_search_landing_low_pwm_" in callback
    assert "rc_search_landing_armed_ = true" in callback
    assert "if (!rc_search_landing_armed_)" in callback
    assert "pwm < rc_search_landing_high_pwm_" in callback
    assert "rc_search_landing_high_since_ = now" in callback
    assert "rc_search_landing_hold_sec_" in callback
    assert callback.index("if (!rc_search_landing_armed_)") < callback.index(
        "activateRcSearchLanding(now)"
    )


def test_rc_trigger_uses_current_pose_and_not_detected_exit():
    activation = SOURCE.split("void TaskSearchManager::activateRcSearchLanding", 1)[1].split(
        "void TaskSearchManager::setMap", 1
    )[0]
    assert "if (!latest_robot_pose_valid_)" in activation
    assert "mission_stage_ = SEARCH_OUTSIDE_LANDING" in activation
    assert "outside_search_anchor_ = latest_robot_pos_" in activation
    assert "exit_portal_center_ = latest_robot_pos_" in activation
    assert "exit_outward_direction_ = outward" in activation
    assert "final_exit_pose_pub_.publish(final_exit_pose)" in activation
    assert "publishSearchState()" in activation
    assert "rc_search_landing_triggered_ = true" in activation


def test_rc_trigger_defaults_to_uav0_ch9_and_is_fail_closed():
    header = (PACKAGE / "include/exploration_manager/task_search_manager.h").read_text()
    assert 'rc_search_landing_topic_{"/UAV0/mavros/rc/in"}' in header
    assert "rc_search_landing_channel_{8}" in header
    assert "rc_search_landing_enabled_{false}" in header
    assert "rc_search_landing_armed_{false}" in header
    assert "rc_search_landing_triggered_{false}" in header


def test_rc_trigger_can_start_fuel_directly_from_wait_trigger():
    activation = SOURCE.split("void TaskSearchManager::activateRcSearchLanding", 1)[1].split(
        "void TaskSearchManager::setMap", 1
    )[0]
    assert "rc_search_landing_start_requested_ = true" in activation
    assert "TaskSearchManager::consumeRcSearchLandingStartRequest" in SOURCE
    assert "task_search_manager_->consumeRcSearchLandingStartRequest()" in MANAGER_SOURCE

    wait_state = FSM_SOURCE.split("case WAIT_TRIGGER:", 1)[1].split(
        "case FINISH:", 1
    )[0]
    assert "consumeRcSearchLandingStartRequest()" in wait_state
    assert "fd_->trigger_ = true" in wait_state
    assert 'transitState(PLAN_TRAJ, "CH9 search-landing trigger")' in wait_state
