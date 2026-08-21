#include "mpc_fsm.h"
#include <uav_utils/converters.h>
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
using namespace std;
using namespace uav_utils;
#define USE_PX4_OR_ARDUPILOT 0 // 0: PX4, 1: ArduPilot. 本项目按 PX4 + MAVROS OFFBOARD 使用。

namespace
{
	constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
	constexpr double safe_output_hold_time = 0.3;
	constexpr double kLastValidMpcHoldSeconds = 0.2;

	std::string firstToken(const std::string &text)
	{
		const std::size_t end = text.find_first_of(" \t\r\n");
		return text.substr(0, end);
	}

	bool isLandingSearchYawState(const std::string &state)
	{
		return state == "FRONT_ARUCO_INITIAL_WAIT" ||
			state == "FRONT_ARUCO_YAW_SCAN_LEFT" ||
			state == "FRONT_ARUCO_YAW_SCAN_RIGHT" ||
			state == "FRONT_ARUCO_YAW_SCAN_RETURN";
	}

	double wrapYaw(double yaw)
	{
		return std::remainder(yaw, 2.0 * M_PI);
	}

	PayloadMPC::ForceAttitudeAlignmentConfig makeForceAttitudeAlignmentConfig(
		const PayloadMPC::MpcParams &params)
	{
		PayloadMPC::ForceAttitudeAlignmentConfig config;
		config.duration = params.force_estimator_param_.attitude_alignment_duration;
		config.min_samples = static_cast<std::size_t>(
			params.force_estimator_param_.attitude_alignment_min_samples);
		config.max_body_rate = params.force_estimator_param_.attitude_alignment_max_body_rate;
		config.max_speed = params.force_estimator_param_.attitude_alignment_max_speed;
		config.max_tilt_error =
			params.force_estimator_param_.attitude_alignment_max_tilt_error_deg * kDegToRad;
		config.max_yaw_std =
			params.force_estimator_param_.attitude_alignment_max_yaw_std_deg * kDegToRad;
		return config;
	}

	const char *forceAttitudeSampleResultName(PayloadMPC::ForceAttitudeSampleResult result)
	{
		switch (result)
		{
		case PayloadMPC::ForceAttitudeSampleResult::ACCUMULATING: return "正在采集";
		case PayloadMPC::ForceAttitudeSampleResult::ALIGNED: return "已对齐";
		case PayloadMPC::ForceAttitudeSampleResult::ALREADY_ALIGNED: return "已经对齐";
		case PayloadMPC::ForceAttitudeSampleResult::DUPLICATE: return "重复样本";
		case PayloadMPC::ForceAttitudeSampleResult::INVALID: return "数据无效";
		case PayloadMPC::ForceAttitudeSampleResult::MOVING: return "机体正在运动";
		case PayloadMPC::ForceAttitudeSampleResult::TILT_MISMATCH: return "横滚/俯仰不一致";
		case PayloadMPC::ForceAttitudeSampleResult::YAW_UNSTABLE: return "偏航不稳定";
		}
		return "未知原因";
	}
}

namespace PayloadMPC
{

	MPCFSM::MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller) : nh_(nh),
															  params_(params),
															  controller_(controller),
															  force_attitude_aligner_(makeForceAttitudeAlignmentConfig(params))
	{
		fsm_state = MANUAL_CTRL;
		exec_traj_state_ = HOVER;
		hover_pose_.setZero();
		hover_yaw_ = 0;
		land_start_time_ = ros::Time(0);
		auto_land_lockout_ = false;

		pub_predicted_trajectory_ =
			nh_.advertise<nav_msgs::Path>("mpc/trajectory_predicted", 1);
		pub_all_ref_data_ =
			nh_.advertise<nav_msgs::Path>("mpc/all_ref_data", 1);

		pub_reference_trajectory_ =
			nh_.advertise<nav_msgs::Path>("mpc/reference_trajectory", 1);
		controller_.resetThrustMapping();

		pub_force_marker_ = nh_.advertise<visualization_msgs::Marker>("mpc/force_marker", 1);
		pub_force_ = nh_.advertise<geometry_msgs::Accel>("mpc/force", 1);
		pub_force_applied_ = nh_.advertise<geometry_msgs::Accel>("mpc/force_applied", 1);
		pub_force_attitude_aligned_ =
			nh_.advertise<std_msgs::Bool>("mpc/force_attitude_aligned", 1, true);
		pub_force_attitude_yaw_offset_ =
			nh_.advertise<std_msgs::Float64>("mpc/force_attitude_yaw_offset", 1, true);

		pub_rmse_info_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/rmse_info", 1);

		force_estimator_.init(params_);
		ROS_INFO("[FORCE] 外力估计姿态来源：%s。",
			params_.force_estimator_param_.use_px4_imu_attitude
				? "MAVROS IMU 姿态"
				: "MAVROS local_position/odom 姿态");
		if (params_.force_estimator_param_.enable_force_estimation)
		{
			if (params_.force_estimator_param_.enable_disturbance_compensation)
				ROS_INFO("[FORCE] 外力补偿已启用。");
			else
				ROS_INFO("[FORCE] 外力估计已启用，当前仅记录，不参与补偿。");
		}
		else
		{
			ROS_INFO("[FORCE] 外力估计已关闭。");
		}
		publishForceAttitudeAlignmentDiagnostics();
		rc_data.set_mode_params(params_.rc_mode_.mode_channel,
								params_.rc_mode_.land_channel,
								params_.rc_mode_.low_threshold,
								params_.rc_mode_.mid_low_threshold,
								params_.rc_mode_.mid_high_threshold,
								params_.rc_mode_.high_threshold);
		nh_.param("landing_search_yaw_timeout", landing_search_yaw_timeout_, 0.5);
		if (!std::isfinite(landing_search_yaw_timeout_) ||
			landing_search_yaw_timeout_ <= 0.0)
		{
			ROS_WARN("[landing_search_yaw] invalid timeout; use 0.5 s.");
			landing_search_yaw_timeout_ = 0.5;
		}
	}

	void MPCFSM::landingSearchStateCallback(const std_msgs::String::ConstPtr &msg)
	{
		const std::string state = firstToken(msg->data);
		const bool active = isLandingSearchYawState(state);
		// WAIT_EXIT_SWITCH 和 LANDING_HANDOFF 之外都属于 CH9 搜索任务。
		// 前视结束后虽然恢复平移轨迹，但 yaw 仍必须固定为管理器持续发布的 CH9 锁定航向。
		landing_search_yaw_override_active_ =
			state != "WAIT_EXIT_SWITCH" && state != "LANDING_HANDOFF";
		if (active == landing_search_yaw_active_)
			return;

		landing_search_yaw_active_ = active;
		if (!active)
		{
			landing_search_hold_latched_ = false;
			ROS_WARN("[landing_search_yaw] 前视偏航阶段结束，允许 Diff 轨迹接管。");
			return;
		}

		// 实际锁点在下一个 NMPC 周期读取最新里程计后完成。
		landing_search_hold_latched_ = false;
		ROS_WARN("[landing_search_yaw] state=%s，准备锁定当前 XYZ 并执行前视偏航。",
			state.c_str());
	}

	void MPCFSM::landingSearchYawCallback(
		const quadrotor_msgs::PositionCommand::ConstPtr &msg)
	{
		if (!std::isfinite(msg->yaw))
		{
			ROS_WARN_THROTTLE(1.0, "[landing_search_yaw] reject non-finite yaw command.");
			return;
		}
		landing_search_yaw_ = wrapYaw(msg->yaw);
		have_landing_search_yaw_ = true;
		last_landing_search_yaw_time_ = ros::Time::now();
	}

	double MPCFSM::landingSearchYawReference(double fallback_yaw) const
	{
		if (landing_search_yaw_override_active_ && have_landing_search_yaw_)
			return landing_search_yaw_;
		return fallback_yaw;
	}

	/*
			Finite State Machine

			   system start
				   /
				  /
				 v
	----- > MANUAL_CTRL
	|         ^   |
	|         |   |
	|         |   |
	|         |   |
	|         |   |
	|         |   v
	|       AUTO_HOVER
	|         ^   |
	|         |   |
	|         |	  |
	|         |   |
	|         |   v
	-------- CMD_CTRL

	*/

	void MPCFSM::process()
	{
		ros::Time now_time = ros::Time::now();
		manual_setpoint_published_ = false;
		if (fsm_state != CMD_CTRL)
		{
			entry_command_active_ = false;
		}

		setEstimateState(odom_data, force_attitude_odom_data);
		if (mpc_recovery_active_ || direct_auto_land_active_)
		{
			clearAppliedDisturbance();
		}
		else
		{
			setForceEstimation();
		}
		if (params_.use_simulation_ && !mpc_recovery_active_ && !direct_auto_land_active_)
		{
			fsm_state = CMD_CTRL;
			rc_data.is_hover_mode = true;
			rc_data.is_command_mode = true;
			static bool is_first_time = true;
			if (is_first_time)
			{
				is_first_time = false;
				update_hover_pose();
			}
		}

		const bool state_fresh = !state_data.rcv_stamp.isZero() &&
			(now_time - state_data.rcv_stamp).toSec() < params_.msg_timeout_.state;
		const bool rc_mode_available = rc_is_received(now_time) && rc_data.mode_input_valid;
		const bool automatic_state = fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL ||
			fsm_state == AUTO_TAKEOFF || fsm_state == AUTO_LAND;
		if (direct_auto_land_active_ && state_fresh &&
			state_data.current_state.mode == "AUTO.LAND")
		{
			clearAutonomousState();
			fsm_state = MANUAL_CTRL;
			auto_land_lockout_ = true;
			suppress_manual_setpoint_ = true;
			ROS_WARN("[AUTO_LAND] PX4 已确认进入 AUTO.LAND，AutoTrans 停止 NMPC 和 setpoint。");
		}
		else if (!direct_auto_land_active_ && automatic_state && !odomControlStateValid(now_time))
			startOdomFailsafe(now_time);
		else if (!direct_auto_land_active_ && automatic_state && state_fresh &&
			state_data.current_state.mode != "OFFBOARD")
		{
			if (!(fsm_state == AUTO_LAND && state_data.current_state.mode == "AUTO.LAND"))
				handleOffboardLoss();
		}

		if (direct_auto_land_active_)
			processDirectAutoLand(now_time);
		else if (mpc_recovery_active_)
			processMpcRecovery(now_time);

		if (!mpc_recovery_active_ && !direct_auto_land_active_)
		switch (fsm_state)
		{
			case MANUAL_CTRL:
			{
				// 与 CAV1 保持一致：Manual 输出只用于进入 OFFBOARD 前的起飞预流。
				// 统一在本周期末尾发布，并通过抑制标志阻止失效状态或 OFFBOARD 内的低推力输出。
				const bool prestream_allowed = rc_mode_available && rc_data.is_takeoff_mode &&
					state_fresh && state_data.current_state.mode != "OFFBOARD";
				suppress_manual_setpoint_ = !prestream_allowed;
				if (odom_failsafe_active_)
			{
				if (state_fresh && state_data.current_state.mode != "OFFBOARD")
				{
					odom_failsafe_active_ = false;
					ROS_WARN("[安全] PX4 已退出 OFFBOARD，结束里程计失联输出保持。");
				}
				break;
			}
			if (!rc_mode_available)
			{
				break;
			}
			if (auto_land_lockout_)
			{
				if (!rc_data.is_takeoff_mode)
					break;
				auto_land_lockout_ = false;
				ROS_WARN("[AUTO_LAND] CH8 已回到低位，解除 AUTO.LAND 锁定。");
			}
			if (!rc_data.is_takeoff_mode)
			{
				takeoff_requested_ = false;
				takeoff_request_latched_ = false;
				last_takeoff_precondition_reason_.clear();
				if (!rc_data.is_hover_mode && !rc_data.is_command_mode)
					break;
				if (state_data.current_state.mode != "OFFBOARD")
				{
					ROS_INFO_ONCE("[自动控制] 等待通过 QGC/CH6 进入 PX4 OFFBOARD。");
					break;
				}
				if (!odomControlStateValid(now_time))
				{
					ROS_WARN_THROTTLE(1.0, "[自动控制] 拒绝进入：里程计或控制姿态无效。");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_WARN_THROTTLE(5.0,
						"[自动控制] 拒绝进入：定位速度 %.2f m/s 超过 3.0 m/s。",
						odom_data.v.norm());
					break;
				}

				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_mode_hover_pose();
				controller_.resetThrustMapping();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				if (rc_data.is_hover_mode)
				{
					fsm_state = AUTO_HOVER;
					ROS_INFO("[AUTO_HOVER] CH8 中位：MANUAL_CTRL -> AUTO_HOVER。");
				}
				else if (rc_data.is_command_mode)
				{
					fsm_state = CMD_CTRL;
					publish_trigger(odom_data.msg);
					ROS_INFO("[CMD_CTRL] CH8 高位：MANUAL_CTRL -> CMD_CTRL。");
				}
				break;
			}
			if (params_.takeoff_.enabled && !takeoff_request_latched_)
			{
				takeoff_request_latched_ = true;
				takeoff_requested_ = true;
				ROS_INFO("[AUTO_TAKEOFF] CH8 低位：持续发送安全预流，等待 PX4 OFFBOARD 和起飞条件满足。");
			}
				if (takeoff_requested_)
			{
				const char *reason = nullptr;
				if (!takeoffPreconditions(now_time, reason))
				{
					const char *reason_zh = "未知安全条件未满足";
					if (std::strcmp(reason, "OFFBOARD") == 0) reason_zh = "PX4 尚未进入 OFFBOARD";
					else if (std::strcmp(reason, "SENSOR_STALE") == 0) reason_zh = "定位、IMU 或 RPM 数据超时";
					else if (std::strcmp(reason, "SENSOR_INVALID") == 0) reason_zh = "传感器数据无效";
					else if (std::strcmp(reason, "DISARMED") == 0) reason_zh = "飞控尚未解锁";
					else if (std::strcmp(reason, "EXTENDED_STATE_STALE") == 0) reason_zh = "PX4 扩展状态数据超时";
					else if (std::strcmp(reason, "NOT_ON_GROUND") == 0) reason_zh = "PX4 尚未确认在地面";
					else if (std::strcmp(reason, "SPEED_UNSAFE") == 0) reason_zh = "定位速度超过安全阈值";
					else if (std::strcmp(reason, "RC_STALE") == 0) reason_zh = "遥控器数据超时";
					else if (std::strcmp(reason, "STATE_STALE") == 0) reason_zh = "PX4 状态数据超时";
					else if (std::strcmp(reason, "TARGET_NOT_ABOVE_UAV") == 0) reason_zh = "起飞目标高度未高于当前高度";
					else if (std::strcmp(reason, "INITIAL_XY_TOO_FAR") == 0) reason_zh = "当前位置距离固定悬停点过远";
					else if (std::strcmp(reason, "DISABLED") == 0) reason_zh = "起飞功能已关闭";
					if (reason != last_takeoff_precondition_reason_)
					{
						ROS_INFO("[AUTO_TAKEOFF] 条件未满足：%s。", reason_zh);
						last_takeoff_precondition_reason_ = reason;
					}
					break;
				}
				last_takeoff_precondition_reason_.clear();
				startAutoTakeoff(now_time);
			}
			break;
		}

		case AUTO_HOVER:
		{
			if (rc_mode_available && rc_data.enter_land_mode)
			{
				std_msgs::Empty stop_msg;
				planning_stop_pub_.publish(stop_msg);
				// AUTO_LAND 禁止使用风力补偿；先清零 OnlineData，再计算首个降落控制量。
				clearAppliedDisturbance();
				land_start_time_ = now_time;
				auto_land_request_sent_ = false;
				last_auto_land_request_time_ = ros::Time(0);
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[AUTO_LAND] CH10 上升沿：AUTO_HOVER -> AUTO_LAND。");
			}
			else if (landing_search_yaw_active_)
			{
				processLandingSearchYawHold(now_time);
			}
			else if (rc_mode_available && rc_data.is_command_mode)
			{
				if (((USE_PX4_OR_ARDUPILOT == 1) && (state_data.current_state.mode == "GUIDED_NOGPS")) || ((USE_PX4_OR_ARDUPILOT == 0) && (state_data.current_state.mode == "OFFBOARD")))
				{
					update_mode_hover_pose();
					controller_.setHoverReference(hover_pose_, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
					fsm_state = CMD_CTRL;
					exec_traj_state_ = HOVER;
					ROS_INFO("[CMD_CTRL] CH8 高位：AUTO_HOVER -> CMD_CTRL。");
					publish_trigger(odom_data.msg);
					ROS_INFO("[CMD_CTRL] 已发送规划触发信号，等待目标或轨迹。");
				}
			}
			else
			{
				if (params_.enable_rc_hover_adjust_ && !params_.fixed_hover_.enabled)
				{
					// 开启后 CH1~CH4 会按 max_manual_vel 积分移动悬停参考点；
					// CH3 表示高度参考速度，不是直接电机油门。非回中油门杆应保持关闭。
					update_hover_with_rc();
				}
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				if (rc_data.enter_command_mode)
				{
					publish_trigger(odom_data.msg);
					ROS_INFO("[CMD_CTRL] 已发送规划触发信号，等待目标或轨迹。");
				}

				// cout << "des.p=" << des.p.transpose() << endl;
			}

			break;
		}

		case CMD_CTRL:
		{
			if (rc_mode_available && rc_data.enter_land_mode)
			{
				std_msgs::Empty stop_msg;
				planning_stop_pub_.publish(stop_msg);
				// AUTO_LAND 禁止使用风力补偿；先清零 OnlineData，再计算首个降落控制量。
				clearAppliedDisturbance();
				land_start_time_ = now_time;
				auto_land_request_sent_ = false;
				last_auto_land_request_time_ = ros::Time(0);
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[AUTO_LAND] CH10 上升沿：CMD_CTRL -> AUTO_LAND。");
			}
			else if (landing_search_yaw_active_)
			{
				processLandingSearchYawHold(now_time);
			}
			else if (rc_mode_available && (rc_data.is_takeoff_mode || rc_data.is_hover_mode))
			{
				// 高位退出命令模式后，低位和中位都回到悬停；已在空中的低位不重复启动起飞。
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_mode_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_HOVER;
				ROS_INFO("[AUTO_HOVER] CH8 低位/中位：CMD_CTRL -> AUTO_HOVER。");
			}
			else
			{
				CMD_CTRL_process();
			}

			break;
		}
		case AUTO_TAKEOFF:
		{
			if (rc_mode_available && rc_data.enter_land_mode)
			{
				std_msgs::Empty stop_msg;
				planning_stop_pub_.publish(stop_msg);
				clearAppliedDisturbance();
				takeoff_requested_ = false;
				land_start_time_ = now_time;
				auto_land_request_sent_ = false;
				last_auto_land_request_time_ = ros::Time(0);
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[AUTO_LAND] CH10 上升沿：AUTO_TAKEOFF -> AUTO_LAND。");
				break;
			}
			if (rc_mode_available && rc_data.is_hover_mode)
			{
				clearAppliedDisturbance();
				takeoff_requested_ = false;
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_mode_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_HOVER;
				ROS_INFO("[AUTO_HOVER] CH8 中位：AUTO_TAKEOFF -> AUTO_HOVER。");
				break;
			}
			if (rc_mode_available && rc_data.is_command_mode)
			{
				takeoff_requested_ = false;
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_mode_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = CMD_CTRL;
				publish_trigger(odom_data.msg);
				ROS_INFO("[CMD_CTRL] CH8 高位：AUTO_TAKEOFF -> CMD_CTRL。");
				break;
			}

			const double elapsed = std::max((now_time - takeoff_start_time_).toSec(), 0.0);
			hover_pose_ = takeoff_start_pose_;
			hover_pose_(2) = std::min(takeoff_target_z_,
									 takeoff_start_pose_(2) + params_.takeoff_.climb_rate * elapsed);
			hover_yaw_ = takeoff_start_yaw_;
			controller_.setHoverReference(hover_pose_, hover_yaw_);
			controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
			break;
		}
		case AUTO_LAND:
		{
			if (state_fresh && state_data.current_state.mode == "AUTO.LAND")
			{
					clearAutonomousState();
					fsm_state = MANUAL_CTRL;
					auto_land_lockout_ = true;
					suppress_manual_setpoint_ = true;
					ROS_WARN("[AUTO_LAND] PX4 已确认进入 AUTO.LAND，AutoTrans 停止 NMPC 和 setpoint。");
				break;
			}
			const double delta_t = std::max((now_time - last_set_hover_pose_time).toSec(), 0.0);
			last_set_hover_pose_time = now_time;
			// AUTO_LAND 内部仍由 NMPC 控制，每周期降低世界系 z 方向悬停目标，单位 m。
			hover_pose_(2) = std::max(hover_pose_(2) - params_.land_.descent_rate * delta_t,
									  params_.land_.min_target_z);
			controller_.setHoverReference(hover_pose_, hover_yaw_);
			controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

			const bool low_enough = hover_pose_(2) <= params_.land_.min_target_z &&
									odom_data.p(2) <= params_.land_.switch_odom_z;
			const bool timeout = params_.land_.timeout > 0.0 &&
								 (now_time - land_start_time_).toSec() > params_.land_.timeout;
			if (controller_.lastMpcSolveSuccessful() && (low_enough || timeout))
			{
				auto_land_request_sent_ = request_px4_auto_land() || auto_land_request_sent_;
			}

			break;
		}
		default:
			break;
		}

		if (!direct_auto_land_active_ && (fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL ||
			fsm_state == AUTO_TAKEOFF || fsm_state == AUTO_LAND))
		{
			if (!mpc_recovery_active_ && !controller_.lastMpcSolveSuccessful())
				beginMpcRecovery(now_time);
			if (!controller_.lastMpcSolveSuccessful() &&
				controller_.hasRecentValidControl(now_time, kLastValidMpcHoldSeconds))
				publish_bodyrate_ctrl(controller_.lastValidControlInput(), now_time);
			else
				publish_bodyrate_ctrl(mpc_predicted_inputs_.col(0), now_time);
			publishPrediction(controller_.reference_states_, mpc_predicted_states_, now_time, controller_.getTimeStep());
		}
			else if (fsm_state == MANUAL_CTRL)
			{
				if (odom_failsafe_active_)
					publishFailsafeHold(now_time);
				else
					publish_manual_ctrl(now_time);
			}

		const ThrustModelGateReason thrust_model_gate = thrustModelGate(now_time);
		reportThrustModelGate(thrust_model_gate);
		if (thrust_model_gate == ThrustModelGateReason::ACTIVE)
		{
			// 发布后再匹配历史命令；门控恢复时队列只有新样本，因此至少等待 35 ms 才会更新。
			controller_.estimateThrustModel(
				imu_data.a, force_attitude_odom_data.q, rpm_data.rpm_vec, bat_data.volt, params_);
		}
		else
		{
			// 门控关闭周期结束前清空队列，保留最后有效 thrustscale/P。
			controller_.clearThrustCommandHistory();
		}

		// STEP6: Clear flags beyound their lifetime
		rc_data.enter_hover_mode = false;
		rc_data.enter_command_mode = false;
		rc_data.enter_land_mode = false;
		rc_data.toggle_reboot = false;
	}
	/*
		Finite State Machine

		   CMD_CTRL
			   /
			  /
			 v
		  HOVER
		  ^   |
		  |   |
		  |   |
		  |   |
		  |   |
		  |   v
		POLY_TRAJ


	*/
	void MPCFSM::CMD_CTRL_process()
	{
		ros::Time now_time = ros::Time::now();
		switch (exec_traj_state_)
		{
		case HOVER:
		{
			if (now_time >= trajectory_data.total_traj_start_time &&
				now_time <= trajectory_data.total_traj_end_time &&
				trajectory_data.exec_traj == 1 && (!trajectory_data.traj_queue.empty()))
			{
				// same as the below
				update_hover_pose();
				oneTraj_Data_t *traj_info = &trajectory_data.traj_queue.front();
				traj_info = &trajectory_data.traj_queue.front();
				trajectory_data.total_traj_start_time = traj_info->traj_start_time;

				double traj_time = (now_time - traj_info->traj_start_time).toSec();
				const double reference_yaw = landingSearchYawReference(hover_yaw_);
				controller_.setTrajectoyReference(
					traj_info->traj, traj_time, reference_yaw,
					traj_info->has_yaw ? &traj_info->yaw_traj : nullptr);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

				exec_traj_state_ = POLY_TRAJ;
				entry_command_active_ = false;
				last_reported_trajectory_id_ = trajectory_data.trajectory_id;
				last_reported_trajectory_piece_ = -1;
				ROS_INFO("[TRAJ] 收到完整轨迹，开始轨迹跟踪：HOVER -> POLY_TRAJ。");
			}
			else
			{
				const bool received_new_command = !cmd_data.rcv_stamp.isZero() &&
					cmd_data.rcv_stamp != last_entry_command_stamp_;
				if (received_new_command)
				{
					last_entry_command_stamp_ = cmd_data.rcv_stamp;
					const bool command_valid = cmd_data.p.allFinite() && cmd_data.v.allFinite() &&
						cmd_data.a.allFinite() && cmd_data.j.allFinite();
					const bool attitude_valid = mpcControlStateValid(now_time);

					if (command_valid && attitude_valid)
					{
						latched_entry_command_ = cmd_data;
						entry_command_yaw_ = landingSearchYawReference(
							get_yaw_from_quaternion(force_attitude_odom_data.q));
						entry_command_active_ = true;
						ROS_INFO("[CMD] 收到新目标：位置=(%.2f, %.2f, %.2f) m。",
							latched_entry_command_.p.x(), latched_entry_command_.p.y(),
							latched_entry_command_.p.z());
						ROS_INFO("[CMD] 收到 PositionCommand，开始跟踪目标。");
						ROS_INFO("[CMD] 忽略规划器 yaw，保持当前航向 %.2f rad。", entry_command_yaw_);
					}
					else
					{
						ROS_ERROR("[CMD] 收到无效目标，继续使用上一目标。");
					}
				}

				if (entry_command_active_)
				{
					if (!controller_.setPositionCommandReference(
						latched_entry_command_.p, latched_entry_command_.v,
						latched_entry_command_.a, latched_entry_command_.j,
						entry_command_yaw_, 0.0))
					{
						entry_command_active_ = false;
						ROS_ERROR_THROTTLE(1.0,
							"[CMD] 当前目标变为无效，回到悬停参考。");
					}
					else
					{
						controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
					}
				}
				if (!entry_command_active_)
				{
					hover_yaw_ = landingSearchYawReference(hover_yaw_);
					controller_.setHoverReference(hover_pose_, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
			}
		}

		break;

		case POLY_TRAJ:
		{
			if (now_time < (trajectory_data.total_traj_start_time) || now_time > trajectory_data.total_traj_end_time || trajectory_data.exec_traj != 1 || trajectory_data.traj_queue.empty())
			{
				if (params_.use_trajectory_ending_pos_ && trajectory_data.exec_traj != -1 &&
					!trajectory_data.traj_queue.empty())
				{
					// tracking the end point of the trajectory
					//  the hover pose is the end point of the trajectory
					auto &traj_info = trajectory_data.traj_queue.front().traj;
					hover_pose_ = traj_info.getJuncPos(traj_info.getPieceNum());
					hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
				}
				else
				{
					// 中止、异常或空队列时只能锁定当前实际位置，禁止访问空队列或恢复旧终点。
					update_hover_pose();
				}
				hover_yaw_ = landingSearchYawReference(hover_yaw_);
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				exec_traj_state_ = HOVER;
				ROS_INFO("[TRAJ] 轨迹结束，保持终点悬停：POLY_TRAJ -> HOVER。");
				trajectory_data.exec_traj = 0;
				printandresetRMSE();
			}
			else
			{
				update_hover_pose();
				oneTraj_Data_t *traj_info = &trajectory_data.traj_queue.front();
				if (now_time < (traj_info->traj_start_time))
				{ // the start time of first trajectory should be whole trajectory start time
					trajectory_data.total_traj_start_time = traj_info->traj_start_time;
					controller_.setHoverReference(hover_pose_, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
				else
				{
					if (trajectory_data.traj_queue.size() > 1)
					{
						oneTraj_Data_t *next_traj_info = &trajectory_data.traj_queue.at(1);
						while (now_time > next_traj_info->traj_start_time)
						{ // finish the first trajectory
							trajectory_data.traj_queue.pop_front();
							traj_info = &trajectory_data.traj_queue.front();
							trajectory_data.total_traj_start_time = traj_info->traj_start_time;
							trajectory_data.total_traj_end_time = trajectory_data.traj_queue.back().traj_end_time;
							if (trajectory_data.traj_queue.size() == 1)
							{
								break;
							}
							next_traj_info = &trajectory_data.traj_queue.at(1);
						}
					}

					 double traj_time = (now_time - traj_info->traj_start_time).toSec();
					double piece_time = traj_time;
					const int piece_index = traj_info->traj.locatePieceIdx(piece_time);
					if (piece_index != last_reported_trajectory_piece_)
					{
						last_reported_trajectory_piece_ = piece_index;
						ROS_INFO("[TRAJ] 当前执行第 %d/%d 段。",
							piece_index + 1, traj_info->traj.getPieceNum());
					}
					addRMSE();
					const double reference_yaw = landingSearchYawReference(hover_yaw_);
					controller_.setTrajectoyReference(
						traj_info->traj, traj_time, reference_yaw,
						traj_info->has_yaw ? &traj_info->yaw_traj : nullptr);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
			}
		}
		break;

		case POINTS:
		{
			exec_traj_state_ = HOVER;
			ROS_ERROR("[TRAJ] 轨迹状态未知，切换到悬停。");
		}
		break;

		default:
		{
			exec_traj_state_ = HOVER;
			ROS_ERROR("[TRAJ] 轨迹状态未知，切换到悬停。");
		}

		break;
		}
	}

	void MPCFSM::setEstimateState(const Odom_Data_t &translation_odom,
						  const Odom_Data_t &attitude_odom)
	{
		est_state_(kPosX) = translation_odom.p[0];
		est_state_(kPosY) = translation_odom.p[1];
		est_state_(kPosZ) = translation_odom.p[2];
		auto rot_q = attitude_odom.q;
		rot_q.normalize();
		est_state_(kOriW) = rot_q.w();
		est_state_(kOriX) = rot_q.x();
		est_state_(kOriY) = rot_q.y();
		est_state_(kOriZ) = rot_q.z();
		est_state_(kVelX) = translation_odom.v[0];
		est_state_(kVelY) = translation_odom.v[1];
		est_state_(kVelZ) = translation_odom.v[2];
	}

	void MPCFSM::publishForceAttitudeAlignmentDiagnostics()
	{
		Eigen::Quaterniond unused_attitude;
		const bool attitude_ready = getForceAttitude(ros::Time::now(), unused_attitude);
		if (force_attitude_diagnostic_published_ &&
			attitude_ready == last_force_attitude_ready_)
		{
			return;
		}
		force_attitude_diagnostic_published_ = true;
		last_force_attitude_ready_ = attitude_ready;

		std_msgs::Bool aligned_msg;
		aligned_msg.data = attitude_ready;
		pub_force_attitude_aligned_.publish(aligned_msg);

		std_msgs::Float64 yaw_offset_msg;
		if (!attitude_ready)
		{
			yaw_offset_msg.data = std::numeric_limits<double>::quiet_NaN();
		}
		else if (params_.force_estimator_param_.use_px4_imu_attitude)
		{
			yaw_offset_msg.data = force_attitude_aligner_.yawOffset();
		}
		else
		{
			// MAVROS local_position/odom 已是 PX4 EKF 融合世界系，不再叠加控制器侧 yaw 偏移。
			yaw_offset_msg.data = 0.0;
		}
		pub_force_attitude_yaw_offset_.publish(yaw_offset_msg);
	}

	bool MPCFSM::getForceAttitude(const ros::Time &now, Eigen::Quaterniond &attitude) const
	{
		if (params_.force_estimator_param_.use_px4_imu_attitude)
		{
			return force_attitude_aligner_.aligned() &&
				force_attitude_aligner_.transformPx4Attitude(imu_data.q, attitude);
		}

		const bool fresh = !force_attitude_odom_data.rcv_stamp.isZero() &&
			(now - force_attitude_odom_data.rcv_stamp).toSec() <
				params_.msg_timeout_.force_attitude_odom;
		const bool finite = force_attitude_odom_data.q.coeffs().allFinite() &&
			force_attitude_odom_data.q.norm() > 1.0e-6;
		if (!fresh || !finite)
		{
			return false;
		}

		attitude = force_attitude_odom_data.q.normalized();
		return true;
	}

	void MPCFSM::resetForceAttitudeAlignment(const char *reason)
	{
		(void)reason;
		const bool had_alignment = force_attitude_aligner_.aligned();
		const std::size_t discarded_samples = force_attitude_aligner_.sampleCount();
		force_attitude_aligner_.reset();
		clearForceObserverState();
		publishForceAttitudeAlignmentDiagnostics();
		if (had_alignment || discarded_samples > 0)
		{
			ROS_WARN("[FORCE] 外力估计姿态对齐已重置：需要重新静止标定。");
		}
	}

	void MPCFSM::clearForceObserverState()
	{
		force_estimator_.reset();
		force_observer_input_valid_ = false;
		fq_estimate_valid_ = false;
		fq_estimated_.setZero();
		fq_applied_.setZero();
		controller_.setExternalForce(fq_applied_);
	}

	void MPCFSM::updateForceAttitudeAlignment(const ros::Time &now)
	{
		if (!params_.force_estimator_param_.use_px4_imu_attitude)
		{
			return;
		}

		const bool odom_fresh = !odom_data.rcv_stamp.isZero() &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom;
		const bool imu_fresh = !imu_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu;
		if (!odom_fresh || !imu_fresh)
		{
			if (have_force_alignment_odom_ || have_force_alignment_imu_ ||
				force_attitude_aligner_.aligned() || force_attitude_aligner_.sampleCount() > 0)
			{
				resetForceAttitudeAlignment(
					!odom_fresh ? "FAST-LIO odom timed out" : "MAVROS IMU timed out");
			}
			have_force_alignment_odom_ = false;
			have_force_alignment_imu_ = false;
			ROS_WARN_THROTTLE(
				1.0,
				"[FORCE] 姿态对齐等待有效数据：FAST-LIO 里程计=%d，MAVROS IMU=%d。",
				odom_fresh, imu_fresh);
			return;
		}

		const ros::Time odom_stamp = odom_data.msg.header.stamp.isZero()
			? odom_data.rcv_stamp
			: odom_data.msg.header.stamp;
		const ros::Time imu_stamp = imu_data.msg.header.stamp.isZero()
			? imu_data.rcv_stamp
			: imu_data.msg.header.stamp;
		const uint32_t odom_seq = odom_data.msg.header.seq;
		const uint32_t imu_seq = imu_data.msg.header.seq;
		const std::string &odom_frame_id = odom_data.msg.header.frame_id;
		const std::string &odom_child_frame_id = odom_data.msg.child_frame_id;
		const std::string &imu_frame_id = imu_data.msg.header.frame_id;
		if (odom_stamp.isZero() || imu_stamp.isZero())
		{
			ROS_WARN_THROTTLE(1.0, "[FORCE] 姿态对齐等待有效时间戳。");
			return;
		}

		bool source_restarted = false;
		const char *restart_reason = nullptr;
		if (have_force_alignment_odom_)
		{
			const bool stamp_reversed = odom_stamp + ros::Duration(1.0e-6) <
				last_force_alignment_odom_stamp_;
			const bool stamp_gap = odom_stamp > last_force_alignment_odom_stamp_ &&
				(odom_stamp - last_force_alignment_odom_stamp_).toSec() >= params_.msg_timeout_.odom;
			const bool sequence_restarted = last_force_alignment_odom_seq_ > 0 &&
				odom_seq < last_force_alignment_odom_seq_;
			const bool frame_changed = odom_frame_id != force_alignment_odom_frame_id_ ||
				odom_child_frame_id != force_alignment_odom_child_frame_id_;
			if (stamp_reversed || stamp_gap || sequence_restarted || frame_changed)
			{
				source_restarted = true;
				restart_reason = "FAST-LIO odom source interrupted, restarted, or frame changed";
			}
		}

		if (have_force_alignment_imu_)
		{
			const bool stamp_reversed = imu_stamp + ros::Duration(1.0e-6) <
				last_force_alignment_imu_stamp_;
			const bool stamp_gap = imu_stamp > last_force_alignment_imu_stamp_ &&
				(imu_stamp - last_force_alignment_imu_stamp_).toSec() >= params_.msg_timeout_.imu;
			const bool sequence_restarted = last_force_alignment_imu_seq_ > 0 &&
				imu_seq < last_force_alignment_imu_seq_;
			const bool frame_changed = imu_frame_id != force_alignment_imu_frame_id_;
			if (stamp_reversed || stamp_gap || sequence_restarted || frame_changed)
			{
				source_restarted = true;
				restart_reason = "MAVROS IMU source interrupted, restarted, or frame changed";
			}
		}

		const bool new_odom_sample = !have_force_alignment_odom_ ||
			odom_stamp != last_force_alignment_odom_stamp_;
		last_force_alignment_odom_stamp_ = odom_stamp;
		last_force_alignment_odom_seq_ = odom_seq;
		force_alignment_odom_frame_id_ = odom_frame_id;
		force_alignment_odom_child_frame_id_ = odom_child_frame_id;
		have_force_alignment_odom_ = true;
		last_force_alignment_imu_stamp_ = imu_stamp;
		last_force_alignment_imu_seq_ = imu_seq;
		force_alignment_imu_frame_id_ = imu_frame_id;
		have_force_alignment_imu_ = true;

		if (source_restarted)
		{
			resetForceAttitudeAlignment(restart_reason);
			return;
		}

		const bool odom_attitude_valid = odom_data.q.coeffs().allFinite() &&
			odom_data.q.norm() > 1.0e-6;
		const bool imu_attitude_valid = imu_data.q.coeffs().allFinite() &&
			imu_data.q.norm() > 1.0e-6 && imu_data.msg.orientation_covariance[0] >= 0.0;
		if (!odom_attitude_valid || !imu_attitude_valid)
		{
			if (force_attitude_aligner_.aligned() || force_attitude_aligner_.sampleCount() > 0)
			{
				resetForceAttitudeAlignment("odom or MAVROS IMU attitude became invalid");
			}
			else
			{
				clearForceObserverState();
			}
			ROS_WARN_THROTTLE(1.0, "[FORCE] 姿态对齐拒绝无效四元数或姿态消息。");
			return;
		}

		if (force_attitude_aligner_.aligned())
		{
			return;
		}
		if (!new_odom_sample)
		{
			return;
		}

		const bool state_fresh = !state_data.rcv_stamp.isZero() &&
			(now - state_data.rcv_stamp).toSec() < params_.msg_timeout_.state;
		const bool extended_state_fresh = !extended_state_data.rcv_stamp.isZero() &&
			(now - extended_state_data.rcv_stamp).toSec() < params_.msg_timeout_.extended_state;
		const bool landed = extended_state_fresh &&
			extended_state_data.current_extended_state.landed_state !=
				mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
		const bool calibration_allowed = state_fresh && extended_state_fresh &&
			odom_fresh && imu_fresh && state_data.current_state.connected &&
			!state_data.current_state.armed && landed;
		if (!calibration_allowed)
		{
			if (force_attitude_aligner_.sampleCount() > 0)
			{
				force_attitude_aligner_.reset();
				publishForceAttitudeAlignmentDiagnostics();
			}
			ROS_WARN_THROTTLE(
				1.0,
				"[FORCE] 姿态对齐等待静止条件：已连接=%d，已解锁=%d，在地面=%d，里程计=%d，IMU=%d，姿态有效=%d。",
				state_data.current_state.connected,
				state_data.current_state.armed,
				landed,
				odom_fresh,
				imu_fresh,
				imu_attitude_valid);
			return;
		}

		const ForceAttitudeSampleResult result = force_attitude_aligner_.addSample(
			odom_stamp.toSec(),
			odom_data.q,
			imu_data.q,
			imu_data.filtered_w,
			odom_data.v);
		if (result == ForceAttitudeSampleResult::ALIGNED)
		{
			force_estimator_.reset();
			force_observer_input_valid_ = false;
			publishForceAttitudeAlignmentDiagnostics();
			ROS_INFO(
				"[FORCE] 外力估计姿态对齐完成：偏航差=%.3f deg，标准差=%.3f deg，样本数=%zu。",
				force_attitude_aligner_.yawOffset() / kDegToRad,
				force_attitude_aligner_.yawStd() / kDegToRad,
				force_attitude_aligner_.sampleCount());
		}
		else if (result == ForceAttitudeSampleResult::ACCUMULATING)
		{
			ROS_INFO_THROTTLE(
				1.0,
				"[FORCE] 姿态对齐采集中：%zu/%d 个样本。",
				force_attitude_aligner_.sampleCount(),
				params_.force_estimator_param_.attitude_alignment_min_samples);
		}
		else if (result != ForceAttitudeSampleResult::DUPLICATE &&
			result != ForceAttitudeSampleResult::ALREADY_ALIGNED)
		{
			publishForceAttitudeAlignmentDiagnostics();
			ROS_WARN_THROTTLE(
				1.0,
				"[FORCE] 姿态对齐样本被拒绝：%s。",
				forceAttitudeSampleResultName(result));
		}
	}

	void MPCFSM::addNewForceObseverState()
	{
		// 起飞阶段的加速度主要来自起飞瞬态，不作为外力 f_Q 估计；
		// 基础 NMPC 仍继续运行，进入悬停/命令模式后再重新收集样本。
		if (fsm_state == AUTO_TAKEOFF)
		{
			clearForceObserverState();
			return;
		}

		force_observer_input_valid_ = false;
		if (!params_.force_estimator_param_.enable_force_estimation)
		{
			clearForceObserverState();
			return;
		}

		const ros::Time now = ros::Time::now();
		updateForceAttitudeAlignment(now);
		Eigen::Quaterniond force_attitude;
		if (!getForceAttitude(now, force_attitude))
		{
			publishForceAttitudeAlignmentDiagnostics();
			ROS_ERROR_THROTTLE(1.0, "[FORCE] 姿态来源超时或包含非有限值，外力估计已清零。");
			clearForceObserverState();
			return;
		}
		publishForceAttitudeAlignmentDiagnostics();
		const bool extended_state_fresh = !extended_state_data.rcv_stamp.isZero() &&
			(now - extended_state_data.rcv_stamp).toSec() < params_.msg_timeout_.extended_state;
		if (extended_state_fresh)
		{
			const bool in_air = extended_state_data.current_extended_state.landed_state ==
				mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
			if (in_air && !was_in_air_)
			{
				// 地面支持力不属于自由飞行动力学；离地时清空地面窗口，仅保留新的空中样本。
				force_estimator_.reset();
				airborne_since_ = now;
				ROS_INFO("[FORCE] 检测到已离地，清空外力估计窗口。");
			}
			else if (!in_air)
			{
				airborne_since_ = ros::Time(0);
			}
			was_in_air_ = in_air;
		}

		const bool input_fresh = !imu_data.rcv_stamp.isZero() && !odom_data.rcv_stamp.isZero() &&
			!rpm_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom &&
			(now - rpm_data.rcv_stamp).toSec() < params_.msg_timeout_.rpm;
		const bool input_finite = imu_data.filtered_a.allFinite() && force_attitude.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && force_attitude.norm() > 1.0e-6;
		if (!input_fresh || !input_finite)
		{
			ROS_ERROR_THROTTLE(1.0, "[FORCE] 外力估计传感器数据超时或无效，暂不更新估计。");
			clearForceObserverState();
			return;
		}

		if (rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
		{
			clearForceObserverState();
			return;
		}
		// 加速度来自 MAVROS IMU 机体系；姿态来自 PX4 EKF 融合 odom，并已处于 MAVROS ENU 世界系。
		force_estimator_.setSystemState(imu_data.filtered_a, force_attitude, rpm_data.filtered_rpm);
		force_observer_input_valid_ = true;
	}

	MPCFSM::ThrustModelGateReason MPCFSM::thrustModelGate(const ros::Time &now) const
	{
		if (params_.thr_map_.accurate_thrust_model != 1)
			return ThrustModelGateReason::DISABLED_BY_PARAM;
		if (fsm_state != AUTO_HOVER && fsm_state != CMD_CTRL)
			return ThrustModelGateReason::MODE_BLOCKED;
		if (state_data.rcv_stamp.isZero() ||
			(now - state_data.rcv_stamp).toSec() >= params_.msg_timeout_.state)
			return ThrustModelGateReason::STATE_STALE;
		if (!state_data.current_state.armed)
			return ThrustModelGateReason::DISARMED;
		if (state_data.current_state.mode != "OFFBOARD")
			return ThrustModelGateReason::NOT_OFFBOARD;
		if (extended_state_data.rcv_stamp.isZero() ||
			(now - extended_state_data.rcv_stamp).toSec() >= params_.msg_timeout_.extended_state)
			return ThrustModelGateReason::EXTENDED_STATE_STALE;
		if (extended_state_data.current_extended_state.landed_state !=
			mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
			return ThrustModelGateReason::LANDED;
		if (rpm_data.rcv_stamp.isZero() ||
			(now - rpm_data.rcv_stamp).toSec() >= params_.msg_timeout_.rpm)
			return ThrustModelGateReason::RPM_STALE;
		if (!rpm_data.rpm_vec.allFinite() ||
			rpm_data.rpm_vec.minCoeff() < params_.thr_map_.min_learning_rpm)
			return ThrustModelGateReason::RPM_INVALID;
		return ThrustModelGateReason::ACTIVE;
	}

	void MPCFSM::reportThrustModelGate(ThrustModelGateReason reason)
	{
		const char *name = "未知状态";
		switch (reason)
		{
		case ThrustModelGateReason::DISABLED_BY_PARAM: name = "参数关闭"; break;
		case ThrustModelGateReason::MODE_BLOCKED: name = "当前模式禁止学习"; break;
		case ThrustModelGateReason::STATE_STALE: name = "PX4 状态超时"; break;
		case ThrustModelGateReason::DISARMED: name = "飞控未解锁"; break;
		case ThrustModelGateReason::NOT_OFFBOARD: name = "尚未进入 OFFBOARD"; break;
		case ThrustModelGateReason::EXTENDED_STATE_STALE: name = "飞行状态超时"; break;
		case ThrustModelGateReason::LANDED: name = "尚未确认离地"; break;
		case ThrustModelGateReason::RPM_STALE: name = "RPM 消息超时"; break;
		case ThrustModelGateReason::RPM_INVALID: name = "RPM 无效"; break;
		case ThrustModelGateReason::ACTIVE: name = "在线学习已生效"; break;
		}

		if (reason == last_thrust_model_gate_reason_)
			return;

		// 门控状态只在发生变化时记录；持续数值由 thrustscale/RPM 的 1 Hz 诊断承担。
		ROS_INFO("[推力映射] 门控状态：%s。", name);
		last_thrust_model_gate_reason_ = reason;
	}

	MPCFSM::DisturbanceGateReason MPCFSM::disturbanceCompensationGate(const ros::Time &now) const
	{
		if (!params_.force_estimator_param_.enable_force_estimation ||
			!params_.force_estimator_param_.enable_disturbance_compensation)
			return DisturbanceGateReason::DISABLED_BY_PARAM;
		if (fsm_state != AUTO_HOVER && fsm_state != CMD_CTRL)
			return DisturbanceGateReason::MODE_BLOCKED;
		if (state_data.rcv_stamp.isZero() ||
			(now - state_data.rcv_stamp).toSec() >= params_.msg_timeout_.state)
			return DisturbanceGateReason::STATE_STALE;
		if (!state_data.current_state.armed)
			return DisturbanceGateReason::DISARMED;
		if (state_data.current_state.mode != "OFFBOARD")
			return DisturbanceGateReason::NOT_OFFBOARD;
		if (extended_state_data.rcv_stamp.isZero() ||
			(now - extended_state_data.rcv_stamp).toSec() >= params_.msg_timeout_.extended_state)
			return DisturbanceGateReason::EXTENDED_STATE_STALE;
		if (extended_state_data.current_extended_state.landed_state !=
			mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
			return DisturbanceGateReason::LANDED;
		if (airborne_since_.isZero() ||
			(now - airborne_since_).toSec() < params_.force_estimator_param_.compensation_airborne_delay)
			return DisturbanceGateReason::AIRBORNE_DELAY;
		if (params_.force_estimator_param_.use_px4_imu_attitude &&
			!force_attitude_aligner_.aligned())
			return DisturbanceGateReason::ATTITUDE_UNALIGNED;
		Eigen::Quaterniond attitude_source;
		if (!getForceAttitude(now, attitude_source))
			return DisturbanceGateReason::SENSOR_INVALID;
		if (!rpm_data.filtered_rpm.allFinite() ||
			rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
			return DisturbanceGateReason::RPM_INVALID;
		const bool sensor_fresh = !imu_data.rcv_stamp.isZero() && !odom_data.rcv_stamp.isZero() &&
			!rpm_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom &&
			(now - rpm_data.rcv_stamp).toSec() < params_.msg_timeout_.rpm;
		const bool sensor_finite = imu_data.filtered_a.allFinite() && attitude_source.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && attitude_source.norm() > 1.0e-6;
		if (!force_observer_input_valid_ || !sensor_fresh || !sensor_finite)
			return DisturbanceGateReason::SENSOR_INVALID;
		if (!force_estimator_.windowFull())
			return DisturbanceGateReason::WINDOW_NOT_FULL;
		if (!fq_estimate_valid_ || !fq_estimated_.allFinite())
			return DisturbanceGateReason::ESTIMATOR_FAILED;
		return DisturbanceGateReason::ACTIVE;
	}

	void MPCFSM::reportDisturbanceGate(DisturbanceGateReason reason)
	{
		if (reason == last_gate_reason_)
			return;

		const char *name = "未知状态";
		switch (reason)
		{
		case DisturbanceGateReason::DISABLED_BY_PARAM: name = "参数关闭"; break;
		case DisturbanceGateReason::MODE_BLOCKED: name = "当前模式不允许"; break;
		case DisturbanceGateReason::STATE_STALE: name = "PX4 状态超时"; break;
		case DisturbanceGateReason::DISARMED: name = "飞控未解锁"; break;
		case DisturbanceGateReason::NOT_OFFBOARD: name = "尚未进入 OFFBOARD"; break;
		case DisturbanceGateReason::EXTENDED_STATE_STALE: name = "飞行状态超时"; break;
		case DisturbanceGateReason::LANDED: name = "尚未确认离地"; break;
		case DisturbanceGateReason::AIRBORNE_DELAY: name = "尚未满足空中稳定等待时间"; break;
		case DisturbanceGateReason::SENSOR_INVALID: name = "传感器数据无效"; break;
		case DisturbanceGateReason::RPM_INVALID: name = "RPM 无效"; break;
		case DisturbanceGateReason::ATTITUDE_UNALIGNED: name = "姿态尚未对齐"; break;
		case DisturbanceGateReason::WINDOW_NOT_FULL: name = "估计窗口未填满"; break;
		case DisturbanceGateReason::ESTIMATOR_FAILED: name = "外力估计失败"; break;
		case DisturbanceGateReason::ACTIVE: name = "补偿已生效"; break;
		}
		if (reason == DisturbanceGateReason::DISABLED_BY_PARAM &&
			params_.force_estimator_param_.enable_force_estimation &&
			!params_.force_estimator_param_.enable_disturbance_compensation)
			ROS_INFO("[FORCE] 外力估计已启用，当前仅记录，不参与补偿。");
		else if (reason == DisturbanceGateReason::MODE_BLOCKED)
			ROS_WARN("[FORCE] 外力补偿被禁止：当前模式不允许。");
		else if (reason == DisturbanceGateReason::AIRBORNE_DELAY)
			ROS_INFO("[FORCE] 外力补偿被禁止：尚未满足空中稳定条件。");
		else if (reason == DisturbanceGateReason::WINDOW_NOT_FULL)
			ROS_INFO("[FORCE] 外力补偿被禁止：估计窗口未填满。");
		else if (reason == DisturbanceGateReason::SENSOR_INVALID)
			ROS_WARN("[FORCE] 外力补偿被禁止：传感器数据无效。");
		else if (reason == DisturbanceGateReason::ACTIVE)
			ROS_INFO("[FORCE] 外力补偿已启用。");
		else
			ROS_INFO("[FORCE] 外力补偿状态：%s。", name);
		last_gate_reason_ = reason;
	}

	void MPCFSM::clearAppliedDisturbance()
	{
		fq_applied_.setZero();
		controller_.setExternalForce(fq_applied_);
	}

	void MPCFSM::processLandingSearchYawHold(const ros::Time &now)
	{
		if (!landing_search_hold_latched_)
		{
			// CH9 后禁止旧 FUEL/入口命令在前视扫描期间继续平移。锁存当前实际位置，
			// 清除旧轨迹；扫描结束后新的 Diff 轨迹仍可正常进入。
			update_hover_pose();
			trajectory_data.exec_traj = 0;
			trajectory_data.traj_queue.clear();
			trajectory_data.total_traj_start_time = ros::Time(0);
			trajectory_data.total_traj_end_time = ros::Time(0);
			exec_traj_state_ = HOVER;
			entry_command_active_ = false;
			cmd_data.rcv_stamp = ros::Time(0);
			landing_search_hold_latched_ = true;
			ROS_ERROR("[landing_search_yaw] 已锁定前视扫描位置 (%.3f, %.3f, %.3f)。",
				hover_pose_.x(), hover_pose_.y(), hover_pose_.z());
		}

		const bool yaw_fresh = have_landing_search_yaw_ &&
			(now - last_landing_search_yaw_time_).toSec() <= landing_search_yaw_timeout_;
		if (yaw_fresh)
			hover_yaw_ = landing_search_yaw_;
		else
			ROS_WARN_THROTTLE(1.0,
				"[landing_search_yaw] 航向指令超时，保持最后航向且不恢复水平轨迹。");

		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
	}

	void MPCFSM::clearAutonomousState()
	{
		controller_.waitForPreparation();
		trajectory_data.traj_queue.clear();
		trajectory_data.total_traj_start_time = ros::Time(0);
		trajectory_data.total_traj_end_time = ros::Time(0);
		trajectory_data.exec_traj = 0;
		exec_traj_state_ = HOVER;
		cmd_data.rcv_stamp = ros::Time(0);
		latched_entry_command_.rcv_stamp = ros::Time(0);
		entry_command_active_ = false;
		takeoff_requested_ = false;
		takeoff_settle_start_ = ros::Time(0);
		clearForceObserverState();
		clearAppliedDisturbance();
		mpc_recovery_active_ = false;
		direct_auto_land_active_ = false;
		planning_stop_sent_ = false;
		mpc_recovery_success_count_ = 0;
		controller_.clearLastValidControl();
		trajectory_data.allowTrajectoryAcceptanceAfter(ros::Time::now());
	}

	bool MPCFSM::odomControlStateValid(const ros::Time &now) const
	{
		return mpcControlStateValid(now);
	}

	bool MPCFSM::mpcControlStateValid(const ros::Time &now) const
	{
		const bool attitude_fresh = !force_attitude_odom_data.rcv_stamp.isZero() &&
			(now - force_attitude_odom_data.rcv_stamp).toSec() <
				params_.msg_timeout_.force_attitude_odom;
		const double attitude_norm = force_attitude_odom_data.q.norm();
		return odom_is_received(now) && attitude_fresh && odom_data.p.allFinite() &&
			odom_data.v.allFinite() && force_attitude_odom_data.q.coeffs().allFinite() &&
			std::isfinite(attitude_norm) && attitude_norm > 1.0e-6;
	}

	void MPCFSM::beginMpcRecovery(const ros::Time &now)
	{
		if (mpc_recovery_active_)
			return;
		if (!mpcControlStateValid(now) || !est_state_.allFinite())
		{
			startOdomFailsafe(now);
			return;
		}

		hover_pose_ = odom_data.p;
		hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
		last_set_hover_pose_time = now;
		trajectory_data.blockTrajectoryAcceptance();
		cmd_data.rcv_stamp = ros::Time(0);
		entry_command_active_ = false;
		exec_traj_state_ = MPC_RECOVERY_HOVER;
		mpc_recovery_start_time_ = now;
		mpc_recovery_success_count_ = 0;
		mpc_recovery_active_ = true;
		clearForceObserverState();
		controller_.clearLastValidControl();
		if (fsm_state == AUTO_TAKEOFF)
			fsm_state = AUTO_HOVER;

		if (!controller_.resetForHover(est_state_, hover_pose_, hover_yaw_))
		{
			ROS_ERROR_THROTTLE(5.0,
				"[安全] NMPC 求解器重置未完成，继续锁存悬停位置并等待下次恢复重试。");
			return;
		}
		ROS_ERROR("[安全] NMPC 求解失败：锁存悬停点 (%.3f, %.3f, %.3f)，进入 MPC_RECOVERY_HOVER；不自动降落。",
			hover_pose_.x(), hover_pose_.y(), hover_pose_.z());
	}

	void MPCFSM::processMpcRecovery(const ros::Time &now)
	{
		if (rc_data.enter_land_mode)
		{
			beginDirectAutoLand(now, "NMPC 恢复期间收到 CH10 人工降落请求");
			return;
		}

		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
		if (controller_.lastMpcSolveSuccessful())
			++mpc_recovery_success_count_;
		else
			mpc_recovery_success_count_ = 0;

		if (mpc_recovery_success_count_ >= params_.safety_.mpc_recovery_success_cycles)
		{
			mpc_recovery_active_ = false;
			exec_traj_state_ = HOVER;
			cmd_data.rcv_stamp = ros::Time(0);
			last_entry_command_stamp_ = ros::Time(0);
			trajectory_data.allowTrajectoryAcceptanceAfter(now);
			std_msgs::Empty restart_msg;
			planning_restart_pub_.publish(restart_msg);
			ROS_INFO("[安全] NMPC 已连续成功恢复 %d 次；保持悬停并请求 FUEL 从最新高频里程计重新规划。",
				params_.safety_.mpc_recovery_success_cycles);
			return;
		}

		const double elapsed = std::max((now - mpc_recovery_start_time_).toSec(), 0.0);
		if (elapsed >= params_.safety_.mpc_recovery_timeout)
		{
			ROS_WARN_THROTTLE(5.0,
				"[安全] NMPC 恢复超过 %.2f s，继续锁点重试，不因求解失败请求 AUTO.LAND。",
				params_.safety_.mpc_recovery_timeout);
		}
	}

	void MPCFSM::beginDirectAutoLand(const ros::Time &now, const char *reason)
	{
		if (!direct_auto_land_active_)
		{
			mpc_recovery_active_ = false;
			direct_auto_land_active_ = true;
			fsm_state = AUTO_LAND;
			exec_traj_state_ = MPC_RECOVERY_HOVER;
			trajectory_data.blockTrajectoryAcceptance();
			mpc_recovery_success_count_ = 0;
			clearAppliedDisturbance();
			controller_.waitForPreparation();
			if (!planning_stop_sent_)
			{
				std_msgs::Empty stop_msg;
				planning_stop_pub_.publish(stop_msg);
				planning_stop_sent_ = true;
			}
			auto_land_request_sent_ = false;
			last_auto_land_request_time_ = ros::Time(0);
			ROS_ERROR("[安全] %s：停止规划和 NMPC，直接请求 PX4 AUTO.LAND。",
				reason == nullptr ? "人工降落请求" : reason);
		}
		processDirectAutoLand(now);
	}

	void MPCFSM::processDirectAutoLand(const ros::Time &now)
	{
		const bool retry_allowed = last_auto_land_request_time_.isZero() ||
			(now - last_auto_land_request_time_).toSec() >= params_.safety_.auto_land_retry_period;
		if (!retry_allowed)
			return;
		auto_land_request_sent_ = request_px4_auto_land();
	}

	void MPCFSM::startOdomFailsafe(const ros::Time &now)
	{
		if (odom_failsafe_active_)
			return;
		clearAutonomousState();
		takeoff_request_latched_ = true;
		suppress_manual_setpoint_ = true;
		odom_failsafe_active_ = true;
		odom_failsafe_start_ = now;
		fsm_state = MANUAL_CTRL;
		ROS_ERROR("[安全] FAST-LIO 里程计超时或状态无效：停止 NMPC，最多保持最后安全输出 %.1f s，随后停止 setpoint，等待 PX4 OFFBOARD 失联保护进入 Land。",
			safe_output_hold_time);
	}

	void MPCFSM::publishFailsafeHold(const ros::Time &now)
	{
		if (!have_last_safe_setpoint_ ||
			(now - odom_failsafe_start_).toSec() > safe_output_hold_time)
			return;
		last_safe_setpoint_.header.stamp = now;
		ctrl_FCU_pub.publish(last_safe_setpoint_);
	}

	void MPCFSM::publish_manual_ctrl(const ros::Time &stamp)
	{
		if (suppress_manual_setpoint_ || !rc_data.mode_input_valid ||
			!rc_data.is_takeoff_mode || state_data.current_state.mode == "OFFBOARD" ||
			manual_setpoint_published_)
		{
			return;
		}
		manual_setpoint_published_ = true;
		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = "FCU";
		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
		// 预流只建立 OFFBOARD setpoint 流；body_rate 为机体系角速度命令，单位 rad/s。
		msg.body_rate.x = 0.0;
		msg.body_rate.y = 0.0;
		msg.body_rate.z = 0.0;
		// 与 CAV1 一致：0.01 仅用于 PX4 进入 OFFBOARD 前建立安全预流，不是悬停推力。
		// PX4 进入 OFFBOARD 后由 AUTO_TAKEOFF 的实际 NMPC 输出立即接管。
		msg.thrust = 0.01;
		ctrl_FCU_pub.publish(msg);
	}

	void MPCFSM::handleOffboardLoss()
	{
		clearAutonomousState();
		// CH8 仍在低位时不自动重新起飞，必须先离开低位再重新进入。
		takeoff_request_latched_ = true;
		suppress_manual_setpoint_ = true;
		fsm_state = MANUAL_CTRL;
		ROS_WARN("[安全] PX4 已退出 OFFBOARD：已清空自动目标并停止 NMPC 和 setpoint。");
	}

	void MPCFSM::setForceEstimation()
	{

		fq_estimated_.setZero();
		fq_estimate_valid_ = force_observer_input_valid_ &&
			force_estimator_.caculate_force(fl_, fq_estimated_);

		const DisturbanceGateReason gate_reason = disturbanceCompensationGate(ros::Time::now());
		reportDisturbanceGate(gate_reason);
		if (gate_reason == DisturbanceGateReason::ACTIVE)
		{
			fq_applied_ = fq_estimated_;
			fq_applied_(0) *= params_.force_estimator_param_.force_axis_gain_x;
			fq_applied_(1) *= params_.force_estimator_param_.force_axis_gain_y;
			fq_applied_(2) *= params_.force_estimator_param_.force_axis_gain_z;
			const double estimated_norm = fq_applied_.norm();
			const double max_applied_force = params_.force_estimator_param_.max_applied_force;
			if (estimated_norm > max_applied_force)
			{
				fq_applied_ *= max_applied_force / estimated_norm;
				ROS_WARN_THROTTLE(
					1.0,
					"[FORCE] 外力补偿达到上限：估计=%.3f N，实际应用=%.3f N。",
					estimated_norm,
					max_applied_force);
			}
		}
		else
		{
			fq_applied_.setZero();
		}
		controller_.setExternalForce(fq_applied_);

		// Publish the force
		if (pub_force_.getNumSubscribers() > 0 || pub_force_applied_.getNumSubscribers() > 0 ||
			pub_force_marker_.getNumSubscribers() > 0)
		{
			geometry_msgs::Accel force_msg;
			// linear 为世界系无人机外力估计 f_Q，单位 N；angular 保留为 0。
			force_msg.linear.x = fq_estimated_(0);
			force_msg.linear.y = fq_estimated_(1);
			force_msg.linear.z = fq_estimated_(2);
			force_msg.angular.x = 0.0;
			force_msg.angular.y = 0.0;
			force_msg.angular.z = 0.0;
			pub_force_.publish(force_msg);

			geometry_msgs::Accel applied_force_msg;
			// linear 为实际写入 NMPC OnlineData 的世界系补偿力，单位 N；补偿关闭或估计无效时为零。
			applied_force_msg.linear.x = fq_applied_(0);
			applied_force_msg.linear.y = fq_applied_(1);
			applied_force_msg.linear.z = fq_applied_(2);
			applied_force_msg.angular.x = 0.0;
			applied_force_msg.angular.y = 0.0;
			applied_force_msg.angular.z = 0.0;
			pub_force_applied_.publish(applied_force_msg);

			visualization_msgs::Marker force_marker;
			force_marker.header.frame_id = "world";
			force_marker.header.stamp = ros::Time::now();
			force_marker.ns = "force";
			force_marker.id = 0;
			force_marker.type = visualization_msgs::Marker::ARROW;
			force_marker.action = visualization_msgs::Marker::ADD;
			force_marker.pose.position.x = est_state_(kPosX);
			force_marker.pose.position.y = est_state_(kPosY);
			force_marker.pose.position.z = est_state_(kPosZ);
			Eigen::Quaterniond q_fq = Eigen::Quaterniond::Identity();
			if (fq_estimated_.norm() > 1.0e-6)
			{
				q_fq = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), fq_estimated_);
			}
			q_fq.normalize();
			force_marker.pose.orientation.x = q_fq.x();
			force_marker.pose.orientation.y = q_fq.y();
			force_marker.pose.orientation.z = q_fq.z();
			force_marker.pose.orientation.w = q_fq.w();
			force_marker.scale.x = fq_estimated_.norm();
			force_marker.scale.y = 0.05;
			force_marker.scale.z = 0.05;
			force_marker.color.a = 1.0;
			force_marker.color.r = 1.0;
			force_marker.color.g = 0.0;
			force_marker.color.b = 0.0;
			pub_force_marker_.publish(force_marker);

		}
	}

	void MPCFSM::printandresetRMSE()
	{
		double drone_rmse = sqrt(rmse_sum_ / rmse_cnt_);
		double drone_rmse_xy = sqrt(rmse_xy_sum_ / rmse_cnt_);
		double drone_max = sqrt(drone_max_);
		double drone_max_xy = sqrt(drone_max_xy_);
		if (params_.print_info_)
		{
			ROS_INFO("[误差] 三维位置 RMSE=%.4f m。", drone_rmse);
			ROS_INFO("[误差] 水平位置 RMSE=%.4f m。", drone_rmse_xy);
			ROS_INFO("[误差] 三维位置最大误差=%.4f m。", drone_max);
			ROS_INFO("[误差] 水平位置最大误差=%.4f m。", drone_max_xy);
		}

		std_msgs::Float64MultiArray msg;
		msg.data.resize(4);
		msg.data[0] = drone_rmse;
		msg.data[1] = drone_rmse_xy;
		msg.data[2] = drone_max;
		msg.data[3] = drone_max_xy;
		pub_rmse_info_.publish(msg);

		rmse_cnt_ = 0;
		rmse_sum_ = 0;
		rmse_xy_sum_ = 0;
		drone_max_ = 0;
		drone_max_xy_ = 0;
	}

	void MPCFSM::addRMSE()
	{
		rmse_cnt_++;
		double pos_err = pow((odom_data.p[0] - controller_.reference_states_(kPosX, 0)), 2) +
						 pow((odom_data.p[1] - controller_.reference_states_(kPosY, 0)), 2) +
						 pow((odom_data.p[2] - controller_.reference_states_(kPosZ, 0)), 2);

		double pos_xy_err = pow((odom_data.p[0] - controller_.reference_states_(kPosX, 0)), 2) +
							pow((odom_data.p[1] - controller_.reference_states_(kPosY, 0)), 2);
		rmse_sum_ += pos_err;
		rmse_xy_sum_ += pos_xy_err;

		if (drone_max_ < pos_err)
		{
			drone_max_ = pos_err;
		}
		if (drone_max_xy_ < pos_xy_err)
		{
			drone_max_xy_ = pos_xy_err;
		}
	}

	bool MPCFSM::takeoffPreconditions(const ros::Time &now, const char *&reason) const
	{
		if (!params_.takeoff_.enabled)
		{
			reason = "DISABLED";
			return false;
		}
		if (rc_data.rcv_stamp.isZero() || (now - rc_data.rcv_stamp).toSec() >= params_.msg_timeout_.rc)
		{
			reason = "RC_STALE";
			return false;
		}
		if (state_data.rcv_stamp.isZero() || (now - state_data.rcv_stamp).toSec() >= params_.msg_timeout_.state)
		{
			reason = "STATE_STALE";
			return false;
		}
		if (state_data.current_state.mode != "OFFBOARD")
		{
			reason = "OFFBOARD";
			return false;
		}
		if (!state_data.current_state.armed)
		{
			reason = "DISARMED";
			return false;
		}
		if (extended_state_data.rcv_stamp.isZero() ||
			(now - extended_state_data.rcv_stamp).toSec() >= params_.msg_timeout_.extended_state)
		{
			reason = "EXTENDED_STATE_STALE";
			return false;
		}
		if (extended_state_data.current_extended_state.landed_state !=
			mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
		{
			reason = "NOT_ON_GROUND";
			return false;
		}
		if (!odom_is_received(now) || !imu_is_received(now) ||
			 rpm_data.rcv_stamp.isZero() || (now - rpm_data.rcv_stamp).toSec() >= params_.msg_timeout_.rpm)
		{
			reason = "SENSOR_STALE";
			return false;
		}
		if (!mpcControlStateValid(now) ||
			!imu_data.filtered_a.allFinite() || !rpm_data.rpm_vec.allFinite() ||
			rpm_data.rpm_vec.minCoeff() < 0.0)
		{
			reason = "SENSOR_INVALID";
			return false;
		}
		if (odom_data.v.norm() > 0.5)
		{
			reason = "SPEED_UNSAFE";
			return false;
		}
		if (params_.takeoff_.target_z <= odom_data.p(2))
		{
			reason = "TARGET_NOT_ABOVE_UAV";
			return false;
		}
		if (params_.fixed_hover_.enabled)
		{
			const double dx = odom_data.p(0) - params_.fixed_hover_.x;
			const double dy = odom_data.p(1) - params_.fixed_hover_.y;
			if (std::hypot(dx, dy) > params_.takeoff_.max_initial_xy_error)
			{
				reason = "INITIAL_XY_TOO_FAR";
				return false;
			}
		}
		return true;
	}

	void MPCFSM::startAutoTakeoff(const ros::Time &now)
	{
		takeoff_requested_ = false;
		takeoff_start_pose_ = odom_data.p;
		takeoff_target_z_ = params_.takeoff_.target_z;
		takeoff_start_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
		takeoff_start_time_ = now;
		last_set_hover_pose_time = now;
		hover_pose_ = takeoff_start_pose_;
		hover_yaw_ = takeoff_start_yaw_;
		trajectory_data.exec_traj = 0;
		exec_traj_state_ = HOVER;
		clearAppliedDisturbance();
		controller_.resetThrustMapping();
		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
		fsm_state = AUTO_TAKEOFF;
		ROS_INFO("[AUTO_TAKEOFF] 开始起飞，目标高度=%.2f m，上升速度=%.2f m/s。",
				takeoff_target_z_, params_.takeoff_.climb_rate);
	}

	void MPCFSM::update_hover_pose()
	{
		last_set_hover_pose_time = ros::Time::now();
		hover_pose_ = odom_data.p;
		// hover_pose_(0) = params_.pos_x_;
		// hover_pose_(1) = params_.pos_y_;
		// hover_pose_(2) = params_.takeoff_height_;

		hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
	}

	void MPCFSM::update_mode_hover_pose()
	{
		// 固定悬停点只用于 AUTO_HOVER 和 CMD_CTRL/HOVER 的模式入口。
		// AUTO_LAND 与轨迹衔接继续调用 update_hover_pose()，避免目标位置或高度跳变。
		if (!params_.fixed_hover_.enabled)
		{
			update_hover_pose();
			return;
		}

		last_set_hover_pose_time = ros::Time::now();
		hover_pose_ << params_.fixed_hover_.x,
			params_.fixed_hover_.y,
			params_.fixed_hover_.z;
		// 只固定 ENU 世界系位置；期望 yaw 保持切入时的当前航向，避免同时产生偏航阶跃。
		hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
		ROS_INFO("[AUTO_HOVER] 悬停参考：位置=(%.2f, %.2f, %.2f) m，yaw=%.2f rad。",
			 hover_pose_.x(), hover_pose_.y(), hover_pose_.z(), hover_yaw_);
	}
	void MPCFSM::update_hover_with_rc()
	{
		ros::Time now = ros::Time::now();
		double delta_t = (now - last_set_hover_pose_time).toSec();
		last_set_hover_pose_time = now;

		hover_pose_(0) += rc_data.ch[1] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.pitch ? 1 : -1);
		hover_pose_(1) += rc_data.ch[0] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.roll ? 1 : -1);
		hover_pose_(2) += rc_data.ch[2] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.throttle ? 1 : -1);
		hover_yaw_ += rc_data.ch[3] * params_.max_manual_vel_ * delta_t * (params_.rc_reverse_.yaw ? 1 : -1);

		if (hover_pose_(2) < -0.3)
			hover_pose_(2) = -0.3;
	}

	void MPCFSM::publishPrediction(
		const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
		const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> predicted_traj,
		ros::Time &time, double dt)
	{
		nav_msgs::Path path_msg;
		nav_msgs::Path reference_path_msg;
		path_msg.header.stamp = time;
		path_msg.header.frame_id = "world";
		reference_path_msg.header = path_msg.header;
		geometry_msgs::PoseStamped pose;
		geometry_msgs::PoseStamped reference_pose;

		for (int i = 0; i < kSamples + 1; i++)
		{
			pose.header.stamp = time + ros::Duration(i * dt);
			pose.header.seq = i;
			pose.pose.position.x = predicted_traj(kPosX, i);
			pose.pose.position.y = predicted_traj(kPosY, i);
			pose.pose.position.z = predicted_traj(kPosZ, i);
			pose.pose.orientation.w = predicted_traj(kOriW, i);
			pose.pose.orientation.x = predicted_traj(kOriX, i);
			pose.pose.orientation.y = predicted_traj(kOriY, i);
			pose.pose.orientation.z = predicted_traj(kOriZ, i);

			path_msg.poses.push_back(pose);

			reference_pose.header.stamp = time + ros::Duration(i * dt);
			reference_pose.header.seq = i;
			reference_pose.pose.position.x = reference_states(kPosX, i);
			reference_pose.pose.position.y = reference_states(kPosY, i);
			reference_pose.pose.position.z = reference_states(kPosZ, i);
			reference_pose.pose.orientation.w = reference_states(kOriW, i);
			reference_pose.pose.orientation.x = reference_states(kOriX, i);
			reference_pose.pose.orientation.y = reference_states(kOriY, i);
			reference_pose.pose.orientation.z = reference_states(kOriZ, i);

			reference_path_msg.poses.push_back(reference_pose);
		}

		pub_predicted_trajectory_.publish(path_msg);
		pub_reference_trajectory_.publish(reference_path_msg);

		nav_msgs::Path all_ref;
		all_ref.header = path_msg.header;
		geometry_msgs::PoseStamped data_point;
		data_point.header = path_msg.header;

		// Position & attitude 0
		data_point.pose.position.x = reference_states(kPosX, 0);
		data_point.pose.position.y = reference_states(kPosY, 0);
		data_point.pose.position.z = reference_states(kPosZ, 0);
		data_point.pose.orientation.w = reference_states(kOriW, 0);
		data_point.pose.orientation.x = reference_states(kOriX, 0);
		data_point.pose.orientation.y = reference_states(kOriY, 0);
		data_point.pose.orientation.z = reference_states(kOriZ, 0);
		all_ref.poses.push_back(data_point);
		// Velocity 1
		data_point.pose.position.x = reference_states(kVelX, 0);
		data_point.pose.position.y = reference_states(kVelY, 0);
		data_point.pose.position.z = reference_states(kVelZ, 0);
		data_point.pose.orientation.w = 1.0;
		data_point.pose.orientation.x = 0.0;
		data_point.pose.orientation.y = 0.0;
		data_point.pose.orientation.z = 0.0;
		all_ref.poses.push_back(data_point);

		// angular velocity 2
		data_point.pose.position.x = controller_.reference_inputs_(kRateX, 0);
		data_point.pose.position.y = controller_.reference_inputs_(kRateY, 0);
		data_point.pose.position.z = controller_.reference_inputs_(kRateZ, 0);
		data_point.pose.orientation.w = 1.0;
		data_point.pose.orientation.x = 0.0;
		data_point.pose.orientation.y = 0.0;
		data_point.pose.orientation.z = 0.0;
		all_ref.poses.push_back(data_point);
		// Thrust 3，单位 N；发布到 MAVROS 前会转成归一化 thrust。
		data_point.pose.position.x = controller_.reference_inputs_(kThrust, 0);
		data_point.pose.position.y = 0.0;
		data_point.pose.position.z = 0.0;
		all_ref.poses.push_back(data_point);

		pub_all_ref_data_.publish(all_ref);
	}

	bool MPCFSM::rc_is_received(const ros::Time &now_time) const
	{
		return (now_time - rc_data.rcv_stamp).toSec() < params_.msg_timeout_.rc;
	}

	bool MPCFSM::odom_is_received(const ros::Time &now_time) const
	{
		return (now_time - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom;
	}

	bool MPCFSM::imu_is_received(const ros::Time &now_time) const
	{
		return (now_time - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu;
	}

	bool MPCFSM::recv_new_odom()
	{
		if (odom_data.rcv_new_msg)
		{
			odom_data.rcv_new_msg = false;
			return true;
		}
		return false;
	}

	void MPCFSM::publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
									   const ros::Time &stamp)
	{
		if (!predicted_input.allFinite())
		{
			ROS_ERROR_THROTTLE(1.0, "[OUTPUT] NMPC 输出包含非有限值，本周期不发布控制量。");
			return;
		}
		mavros_msgs::AttitudeTarget msg;

		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");

		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		// double collective_thrust;
		Eigen::Vector3d bodyrates;

		// collective_thrust = input_bounded(INPUT_BODYRATE::kThrust);
		bodyrates << predicted_input(INPUT_BODYRATE::kRateX), predicted_input(INPUT_BODYRATE::kRateY),
			predicted_input(INPUT_BODYRATE::kRateZ);

		const double max_bodyrate_xy = static_cast<double>(params_.max_bodyrate_xy_);
		const double max_bodyrate_z = static_cast<double>(params_.max_bodyrate_z_);
		msg.body_rate.x = std::max(-max_bodyrate_xy, std::min(bodyrates[0], max_bodyrate_xy));
		msg.body_rate.y = std::max(-max_bodyrate_xy, std::min(bodyrates[1], max_bodyrate_xy));
		msg.body_rate.z = std::max(-max_bodyrate_z, std::min(bodyrates[2], max_bodyrate_z));

		// body_rate.x/y/z 是发送给 MAVROS/PX4 的机体系角速度命令，单位通常为 rad/s。
		// AttitudeTarget.thrust 是 PX4 归一化推力命令，不是 NMPC 内部的牛顿推力 T。
		if (params_.use_simulation_)
		{
			msg.thrust = predicted_input(INPUT_BODYRATE::kThrust);
		}
		else
		{
			msg.thrust = controller_.convertThrust(predicted_input(INPUT_BODYRATE::kThrust), bat_data.volt);
		}
		if (!std::isfinite(msg.thrust))
		{
			ROS_ERROR_THROTTLE(1.0, "[OUTPUT] 归一化推力为非有限值，本周期不发布控制量。");
			return;
		}

		ROS_INFO_THROTTLE(1.0,
			"[OUTPUT] 推力=%.3f（MAVROS 归一化值），机体系角速度=(%.2f, %.2f, %.2f) rad/s。",
			msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z);

		ctrl_FCU_pub.publish(msg);
		last_safe_setpoint_ = msg;
		have_last_safe_setpoint_ = true;
	}

	void MPCFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
	{
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "world";
		msg.pose = odom_msg.pose.pose;

		traj_start_trigger_pub.publish(msg);
	}

	bool MPCFSM::request_px4_auto_land()
	{
		const ros::Time now = ros::Time::now();
		if (!last_auto_land_request_time_.isZero() &&
			(now - last_auto_land_request_time_).toSec() < params_.safety_.auto_land_retry_period)
			return false;
		last_auto_land_request_time_ = now;
		mavros_msgs::SetMode land_set_mode;
		// PX4 AUTO.LAND 由飞控接管最终降落；本函数不负责解锁或起飞。
		land_set_mode.request.custom_mode = "AUTO.LAND";
		if (!(set_FCU_mode_srv.call(land_set_mode) && land_set_mode.response.mode_sent))
		{
			ROS_ERROR_THROTTLE(5.0, "[AUTO_LAND] PX4 未接受 AUTO.LAND，继续重试。");
			return false;
		}
		ROS_INFO_THROTTLE(1.0, "[AUTO_LAND] PX4 已接受 AUTO.LAND 请求，等待状态确认。");
		return true;
	}

	void MPCFSM::reboot_FCU()
	{
		// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
		mavros_msgs::CommandLong reboot_srv;
		reboot_srv.request.broadcast = false;
		reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
		reboot_srv.request.param1 = 1;	  // Reboot autopilot
		reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
		reboot_srv.request.confirmation = true;

		reboot_FCU_srv.call(reboot_srv);

		ROS_INFO("[系统] 正在请求重启飞控。");

		// if (params_.print_dbg)
		// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
	}

}
