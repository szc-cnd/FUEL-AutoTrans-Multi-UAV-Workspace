#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[5]
PACKAGE = Path(__file__).resolve().parents[1]


class Cav0SyncContractStaticTest(unittest.TestCase):
    def test_controller_sources_are_split_as_required(self):
        launch = (PACKAGE / "launch/quad_wind_mpc_controller.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('/UAV0/fast_lio/Odom_high_freq', launch)
        self.assertIn('/UAV0/mavros/local_position/odom', launch)
        self.assertIn('/UAV0/mavros/imu/data', launch)
        self.assertIn('from="~force_attitude_odom"', launch)

    def test_recovery_has_no_automatic_land_transition(self):
        fsm = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        begin = fsm.index("void MPCFSM::beginMpcRecovery")
        end = fsm.index("void MPCFSM::processMpcRecovery", begin)
        recovery = fsm[begin:end]
        self.assertIn("MPC_RECOVERY_HOVER", recovery)
        self.assertNotIn("AUTO_LAND", recovery)
        process = fsm[end : fsm.index("void MPCFSM::", end + 10)]
        self.assertIn("planning_restart_pub_", process)
        self.assertIn("CH10", process)

    def test_recovery_levels_attitude_and_requires_converged_nominal_solution(self):
        fsm = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        controller = (PACKAGE / "src/mpc_controller.cpp").read_text(encoding="utf-8")
        header = (
            PACKAGE / "include/payload_mpc_controller/mpc_controller.h"
        ).read_text(encoding="utf-8")
        wrapper = (PACKAGE / "src/mpc_wrapper.cpp").read_text(encoding="utf-8")
        config = (PACKAGE / "config/mpc.yaml").read_text(encoding="utf-8")

        self.assertIn("mpc_recovery_success_cycles: 5", config)
        self.assertIn("mpc_recovery_last_valid_hold: 0.05", config)
        self.assertIn("publish_recovery_attitude_ctrl", fsm)
        self.assertIn("recoveryStateConverged", fsm)
        self.assertIn("restoreNominalVelocityLimits", fsm)
        self.assertIn("mpc_recovery_full_reset_period", fsm)
        self.assertIn("odom_spike_guard_.faultActive()", fsm)

        begin = fsm.split("void MPCFSM::beginMpcRecovery", 1)[1]
        begin = begin.split("void MPCFSM::processMpcRecovery", 1)[0]
        self.assertNotIn("clearLastValidControl", begin)

        attitude_publish = fsm.split("void MPCFSM::publish_recovery_attitude_ctrl", 1)[1]
        attitude_publish = attitude_publish.split("void MPCFSM::publish_bodyrate_ctrl", 1)[0]
        self.assertIn("IGNORE_ROLL_RATE", attitude_publish)
        self.assertIn("IGNORE_PITCH_RATE", attitude_publish)
        self.assertIn("IGNORE_YAW_RATE", attitude_publish)
        self.assertNotIn("IGNORE_ATTITUDE", attitude_publish)
        self.assertIn("controller_.currentHoverPercentage()", attitude_publish)
        self.assertNotIn("params_.thr_map_.hover_percentage", attitude_publish)
        self.assertIn("double currentHoverPercentage() const;", header)
        self.assertIn("weight / thrustscale_", controller)

        self.assertIn("std::mutex external_force_mutex_", header)
        setter = controller.split("void MpcController::setExternalForce", 1)[1]
        setter = setter.split("void MpcController::execMPC", 1)[0]
        self.assertNotIn("mpc_wrapper_.setExternalForce", setter)
        preparation = controller.split("void MpcController::preparationThread", 1)[1]
        preparation = preparation.split("double MpcController::angle_limit", 1)[0]
        self.assertIn("mpc_wrapper_.setExternalForce(external_force);", preparation)

        initialize = wrapper.split("void MpcWrapper::initialize", 1)[1]
        initialize = initialize.split("bool MpcWrapper::setCosts", 1)[0]
        self.assertNotIn("acado_preparationStep();", initialize)

    def test_bridge_uses_latched_output_and_post_trajectory_timeout_abort(self):
        bridge = (
            WORKSPACE / "src/autotrans_reference_bridge/src/fuel_autotrans_bridge_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, true)", bridge)
        self.assertIn('param("trajectory_timeout", trajectory_timeout_, 2.0)', bridge)
        self.assertIn("abort_deadline_ = active_until", bridge)
        self.assertIn("createTimer", bridge)
        self.assertIn("ACTION_ABORT", bridge)

    def test_seven_pane_entry_uses_uav0_high_frequency_chain(self):
        script = (WORKSPACE / "shfiles/start_uav0_six_terminator.sh").read_text(
            encoding="utf-8"
        )
        layout = (WORKSPACE / "shfiles/terminator_uav0_six.conf").read_text(
            encoding="utf-8"
        )
        self.assertEqual(layout.count("type = Terminal"), 7)
        self.assertIn("--pane detection", layout)
        for value in (
            "/UAV0/fast_lio/Odom_high_freq",
            "/UAV0/mavros/local_position/odom",
            "/fuel_traj_server",
            "uav0_autotrans_controller.launch",
        ):
            self.assertIn(value, script)
        self.assertNotIn("rosbag record", script)

        controller_launch = (
            PACKAGE / "launch/quad_wind_mpc_controller.launch"
        ).read_text(encoding="utf-8")
        logger = (PACKAGE / "scripts/autotrans_mpc_logger.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('name="enable_rosbag"', controller_launch)
        self.assertIn('param="rosbag_topics"', controller_launch)
        self.assertIn("start_rosbag_recording", logger)
        self.assertIn("stop_rosbag_recording", logger)

    def test_cav1_controller_behavior_is_synced_without_replacing_fuel(self):
        fsm = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        controller = (PACKAGE / "src/mpc_controller.cpp").read_text(encoding="utf-8")
        controller_header = (
            PACKAGE / "include/payload_mpc_controller/mpc_controller.h"
        ).read_text(encoding="utf-8")
        inputs = (PACKAGE / "src/mpc_input.cpp").read_text(encoding="utf-8")
        params = (
            PACKAGE / "include/payload_mpc_controller/mpc_params.h"
        ).read_text(encoding="utf-8")

        self.assertIn("beginDirectAutoLand", fsm)
        self.assertIn("MANUAL_CTRL -> AUTO_HOVER", fsm)
        self.assertIn("MANUAL_CTRL -> CMD_CTRL", fsm)
        self.assertIn("yaw_reference_initialized_", controller_header)
        self.assertIn("predicted_yaw", controller)
        self.assertIn("trajectory_id 重新计数", inputs)
        self.assertNotIn("trajectory_id 必须从 1 开始", inputs)
        self.assertIn("首个有效样本直接初始化", inputs)
        self.assertIn("force_axis_gain_x", params)

        recovery_start = fsm.index("void MPCFSM::beginMpcRecovery")
        recovery_end = fsm.index("void MPCFSM::processMpcRecovery", recovery_start)
        recovery = fsm[recovery_start:recovery_end]
        self.assertIn("if (!controller_.resetForHover", recovery)

        land_case = fsm.split("case AUTO_LAND:", 1)[1].split("default:", 1)[0]
        self.assertIn(
            "controller_.lastMpcSolveSuccessful() && (low_enough || timeout)",
            land_case,
        )

        prestream_start = fsm.index("void MPCFSM::publish_manual_ctrl")
        prestream_end = fsm.index("void MPCFSM::handleOffboardLoss", prestream_start)
        self.assertIn("msg.thrust = 0.01;", fsm[prestream_start:prestream_end])
        self.assertIn("suppress_manual_setpoint_", fsm[prestream_start:prestream_end])
        self.assertIn("manual_setpoint_published_", fsm[prestream_start:prestream_end])

        manual = fsm.split("case MANUAL_CTRL:", 1)[1].split("case AUTO_HOVER:", 1)[0]
        self.assertIn('state_data.current_state.mode != "OFFBOARD"', manual)
        self.assertIn("suppress_manual_setpoint_ = !prestream_allowed", manual)
        self.assertNotIn("takeoff_prestream_start_", manual)
        self.assertNotIn("至少持续 1.0 s", manual)
        self.assertIn("controller_.resetThrustMapping();", manual)

        config = (PACKAGE / "config/mpc.yaml").read_text(encoding="utf-8")
        self.assertIn("force_attitude_odom: 0.1", config)

    def test_disturbance_compensation_is_slew_limited_after_magnitude_cap(self):
        fsm = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        params = (
            PACKAGE / "include/payload_mpc_controller/mpc_params.h"
        ).read_text(encoding="utf-8")
        config = (PACKAGE / "config/mpc.yaml").read_text(encoding="utf-8")

        for name, value in (
            ("max_applied_force_rate_xy", "2.0"),
            ("max_applied_force_rate_z", "3.0"),
        ):
            self.assertIn('read_essential_param(nh, "force_estimator/%s"' % name, params)
            self.assertIn("%s: %s" % (name, value), config)

        update = fsm.split("void MPCFSM::setForceEstimation", 1)[1]
        update = update.split("void MPCFSM::", 1)[0]
        gain_position = update.index("force_axis_gain_x")
        magnitude_position = update.index("estimated_norm > max_applied_force")
        slew_position = update.index("disturbance_slew_limiter_.update")
        publish_position = update.index("controller_.setExternalForce")
        self.assertLess(gain_position, magnitude_position)
        self.assertLess(magnitude_position, slew_position)
        self.assertLess(slew_position, publish_position)
        self.assertIn("constexpr double initial_slew_dt = 0.01", update)
        self.assertIn("slew_dt = initial_slew_dt", update)
        self.assertIn("std::min(elapsed, 2.0 * nominal_dt)", update)

        clear = fsm.split("void MPCFSM::clearAppliedDisturbance", 1)[1]
        clear = clear.split("void MPCFSM::", 1)[0]
        self.assertIn("disturbance_slew_limiter_.reset()", clear)
        self.assertIn("last_disturbance_slew_update_time_ = ros::Time(0)", clear)
        self.assertEqual(fsm.count("fq_applied_.setZero()"), 1)

        observer_clear = fsm.split("void MPCFSM::clearForceObserverState", 1)[1]
        observer_clear = observer_clear.split("void MPCFSM::", 1)[0]
        self.assertIn("clearAppliedDisturbance()", observer_clear)

    def test_precision_landing_defaults_to_high_frequency_odometry(self):
        landing_launch = (
            WORKSPACE / "src/precision_landing/launch/precision_landing.launch"
        ).read_text(encoding="utf-8")
        competition_launch = (
            WORKSPACE
            / "src/uav0_competition_bringup/launch/uav0_detection_landing_stack.launch"
        ).read_text(encoding="utf-8")
        self.assertIn("/fast_lio/Odom_high_freq", landing_launch)
        self.assertNotIn("/fast_lio/Odometry", landing_launch)
        self.assertIn("/UAV0/fast_lio/Odom_high_freq", competition_launch)

    def test_controller_joins_preparation_thread_on_early_exit(self):
        header = (PACKAGE / "include/payload_mpc_controller/mpc_controller.h").read_text(
            encoding="utf-8"
        )
        controller = (PACKAGE / "src/mpc_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("~MpcController();", header)
        destructor_start = controller.index("MpcController::~MpcController()")
        next_method = controller.index("void MpcController::execMPC", destructor_start)
        destructor = controller[destructor_start:next_method]
        self.assertIn("waitForPreparation();", destructor)


if __name__ == "__main__":
    unittest.main()
