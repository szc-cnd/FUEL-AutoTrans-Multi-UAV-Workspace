#include "mpc_fsm.h"
#include <uav_utils/converters.h>
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <cmath>
#include <cstring>
using namespace std;
using namespace uav_utils;
#define USE_PX4_OR_ARDUPILOT 0 // 0: PX4, 1: ArduPilot. 本项目按 PX4 + MAVROS OFFBOARD 使用。
namespace
{
	PayloadMPC::OdomSpikeGuardConfig makeOdomSpikeGuardConfig(
		const PayloadMPC::MpcParams &params)
	{
		PayloadMPC::OdomSpikeGuardConfig config;
		config.enabled = params.odom_spike_guard_.enabled;
		config.max_sample_interval = params.odom_spike_guard_.max_sample_interval;
		config.max_position_residual_xy = params.odom_spike_guard_.max_position_residual_xy;
		config.max_position_residual_z = params.odom_spike_guard_.max_position_residual_z;
		config.max_velocity_jump_xy = params.odom_spike_guard_.max_velocity_jump_xy;
		config.max_velocity_jump_z = params.odom_spike_guard_.max_velocity_jump_z;
		config.fault_duration = params.odom_spike_guard_.fault_duration;
		config.recovery_good_samples = params.odom_spike_guard_.recovery_good_samples;
		return config;
	}
}
namespace PayloadMPC
{
	MPCFSM::MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller) : nh_(nh),
																							  params_(params),
																							  controller_(controller),
																							  odom_spike_guard_(makeOdomSpikeGuardConfig(params))
	{
		fsm_state = MANUAL_CTRL;
		exec_traj_state_ = HOVER;
		hover_pose_.setZero();
		hover_yaw_ = 0;
		land_start_time_ = ros::Time(0);

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
		ROS_INFO("[MPC CTRL] Disturbance compensation axis gains: x=%.3f, y=%.3f, z=%.3f",
			params_.force_estimator_param_.force_axis_gain_x,
			params_.force_estimator_param_.force_axis_gain_y,
			params_.force_estimator_param_.force_axis_gain_z);

		pub_rmse_info_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/rmse_info", 1);

		force_estimator_.init(params_);
		rc_data.set_mode_params(params_.rc_mode_.mode_channel,
								params_.rc_mode_.land_channel,
								params_.rc_mode_.low_threshold,
								params_.rc_mode_.mid_low_threshold,
								params_.rc_mode_.mid_high_threshold,
								params_.rc_mode_.high_threshold);
	}

	void MPCFSM::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
	{
		Odom_Data_t candidate;
		candidate.feed(msg);
		const OdomSpikeGuardResult result = odom_spike_guard_.evaluate(
			candidate.rcv_stamp.toSec(), candidate.p, candidate.v);

		if (result == OdomSpikeGuardResult::ACCEPTED ||
			result == OdomSpikeGuardResult::RECOVERED)
		{
			odom_data = candidate;
			if (result == OdomSpikeGuardResult::RECOVERED)
			{
				ROS_WARN("[ODOM_SPIKE] FAST-LIO 已连续恢复 %d 个正常样本，解除尖峰故障锁存。",
					params_.odom_spike_guard_.recovery_good_samples);
			}
			return;
		}

		ROS_WARN_THROTTLE(
			0.5,
			"[ODOM_SPIKE] 拒绝 FAST-LIO 异常样本：位置残差 xy=%.3f z=%.3f m，速度突变 xy=%.3f z=%.3f m/s；暂用上一可信状态。",
			odom_spike_guard_.lastPositionResidualXY(),
			odom_spike_guard_.lastPositionResidualZ(),
			odom_spike_guard_.lastVelocityJumpXY(),
			odom_spike_guard_.lastVelocityJumpZ());
		if (result == OdomSpikeGuardResult::FAULT_LATCHED)
		{
			ROS_ERROR("[ODOM_SPIKE] FAST-LIO 异常已连续 %.2f s，锁存故障并进入 NMPC 原有恢复流程。",
				params_.odom_spike_guard_.fault_duration);
		}
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
		const bool auto_state = fsm_state == AUTO_TAKEOFF || fsm_state == AUTO_HOVER ||
			fsm_state == CMD_CTRL || fsm_state == AUTO_LAND;
		const bool state_fresh = !state_data.rcv_stamp.isZero() &&
			(now_time - state_data.rcv_stamp).toSec() < params_.msg_timeout_.state;

		// 里程计失效时不再重新求解或恢复旧任务，交给 PX4 OFFBOARD 失联保护。
		if (odom_failsafe_active_)
		{
			if (state_fresh && state_data.current_state.mode != "OFFBOARD")
			{
				odom_failsafe_active_ = false;
				ROS_WARN("[安全] PX4 已退出 OFFBOARD，停止 AutoTrans setpoint。");
			}
			else if (now_time <= odom_failsafe_deadline_)
			{
				publish_failsafe_hold(now_time);
			}
			return;
		}

		if (auto_state && (!odom_is_received(now_time) || !odom_state_valid()))
		{
			enter_odom_failsafe("FAST-LIO 平移状态或 MAVROS 姿态未接收/无效");
			publish_failsafe_hold(now_time);
			return;
		}

		if (fsm_state == AUTO_LAND && state_fresh && state_data.current_state.mode == "AUTO.LAND")
		{
			clear_autonomous_inputs();
			fsm_state = MANUAL_CTRL;
			auto_land_lockout_ = true;
			suppress_manual_setpoint_ = true;
			ROS_INFO("[AUTO_LAND] PX4 已确认 AUTO.LAND，AutoTrans 停止 NMPC 和 setpoint。");
			return;
		}

		if (auto_state && state_fresh && state_data.current_state.mode != "OFFBOARD")
		{
			clear_autonomous_inputs();
			fsm_state = MANUAL_CTRL;
			suppress_manual_setpoint_ = true;
			ROS_WARN_THROTTLE(1.0, "[安全] PX4 已退出 OFFBOARD，AutoTrans 同步停止控制输出。");
			return;
		}

		setEstimateState(odom_data, force_attitude_odom_data);
		if (!direct_auto_land_active_ && auto_state && odom_spike_guard_.faultActive())
			beginMpcRecovery(now_time, "FAST-LIO 里程计连续尖峰");
		if (mpc_recovery_active_ || direct_auto_land_active_)
		{
			fq_estimated_.setZero();
			fq_applied_.setZero();
			controller_.setExternalForce(fq_applied_);
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

		bool mpc_result_handled = false;
		if (direct_auto_land_active_)
		{
			processDirectAutoLand(now_time);
			mpc_result_handled = true;
		}
		else if (mpc_recovery_active_)
		{
			processMpcRecovery(now_time);
			mpc_result_handled = true;
		}
		else
		{
			switch (fsm_state)
			{
		case MANUAL_CTRL:
		{
			// 预流仅用于 PX4 进入 OFFBOARD 前维持外部 setpoint 数据流。
			// body_rate 为机体系角速度命令，单位 rad/s；thrust 为归一化推力，不是牛顿力。
			const bool prestream_allowed = rc_data.mode_valid && rc_data.is_takeoff_mode &&
				state_fresh && state_data.current_state.mode != "OFFBOARD";
			suppress_manual_setpoint_ = !prestream_allowed;
			if (auto_land_lockout_)
			{
				if (rc_data.mode_valid && rc_data.is_takeoff_mode)
				{
					// PX4 AUTO.LAND 请求后要求 CH8 回低位，确认操作者已退出自动降落请求。
					auto_land_lockout_ = false;
					ROS_WARN("[AUTO_LAND] CH8 已回到低位，解除 AUTO.LAND 锁定。");
				}
				else
				{
					ROS_WARN_ONCE("[AUTO_LAND] 等待 CH8 回到低位以解除 AUTO.LAND 锁定。");
					break;
				}
			}

			// CH8 低位请求 AUTO_TAKEOFF；CH6 只由 QGC/PX4 负责切换 OFFBOARD。
			if (!rc_data.is_takeoff_mode)
			{
				takeoff_requested_ = false;
				takeoff_request_latched_ = false;
				last_takeoff_precondition_reason_.clear();
			}
			else if (params_.takeoff_.enabled && !takeoff_request_latched_)
			{
				takeoff_request_latched_ = true;
				takeoff_requested_ = true;
				ROS_INFO("[AUTO_TAKEOFF] CH8 低位：持续发送安全预流，等待 PX4 OFFBOARD 和起飞条件满足。");
			}

			// AUTO_TAKEOFF 只在 PX4 已经进入 OFFBOARD 后启动；CH6 由 PX4/QGC 的 RC_MAP_OFFB_SW 处理。
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
				break;
			}

			if (rc_data.is_hover_mode)
			{
				if (state_data.current_state.mode != "OFFBOARD")
				{
					ROS_INFO_ONCE("[AUTO_HOVER] 等待通过 QGC/CH6 进入 PX4 OFFBOARD。");
					break;
				}
				if (!odom_is_received(now_time) || !odom_state_valid())
				{
					ROS_WARN_THROTTLE(1.0, "[AUTO_HOVER] 拒绝进入悬停：里程计无效。");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_WARN_THROTTLE(5.0, "[AUTO_HOVER] 拒绝进入悬停：定位速度 %.2f m/s 超过 3.0 m/s。", odom_data.v.norm());
					break;
				}

				trajectory_data.exec_traj = 0;
				update_mode_hover_pose();
				controller_.resetThrustMapping();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

				fsm_state = AUTO_HOVER;
				exec_traj_state_ = HOVER;
				ROS_INFO("[AUTO_HOVER] CH8 中位：MANUAL_CTRL -> AUTO_HOVER。");
			}
			else if (rc_data.is_command_mode)
			{
				if (state_data.current_state.mode != "OFFBOARD")
				{
					ROS_INFO_ONCE("[CMD_CTRL] 等待通过 QGC/CH6 进入 PX4 OFFBOARD。");
					break;
				}
				if (!odom_is_received(now_time) || !odom_state_valid())
				{
					ROS_WARN_THROTTLE(1.0, "[CMD_CTRL] 拒绝进入命令模式：里程计无效。");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_WARN_THROTTLE(5.0, "[CMD_CTRL] 拒绝进入命令模式：定位速度 %.2f m/s 超过 3.0 m/s。", odom_data.v.norm());
					break;
				}

				trajectory_data.exec_traj = 0; // clean the trajectory data
				update_mode_hover_pose();
				controller_.resetThrustMapping();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

				fsm_state = CMD_CTRL;
				exec_traj_state_ = HOVER;
				publish_trigger(odom_data.msg);
				ROS_INFO("[CMD_CTRL] CH8 高位：MANUAL_CTRL -> CMD_CTRL。");
			}

			if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
			{
				if (state_data.current_state.armed)
				{
					ROS_ERROR("[安全] 拒绝重启飞控：请先解除解锁。");
					break;
				}
				reboot_FCU();
			}

			break;
		}

		case AUTO_HOVER:
		{
			if (!odom_is_received(now_time) || !odom_state_valid())
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;

				ROS_WARN("[安全] AUTO_HOVER 因里程计失效退出自动控制。");
			}
			else if (rc_data.enter_land_mode)
			{
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
				ROS_WARN("[AUTO_LAND] CH10 触发：AUTO_HOVER -> AUTO_LAND。");
			}
			else if (rc_data.is_command_mode)
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
					ROS_INFO("[CMD_CTRL] 已发送规划触发信号，等待有效轨迹。");
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
					ROS_INFO("[CMD_CTRL] 已发送规划触发信号，等待有效轨迹。");
				}

				// cout << "des.p=" << des.p.transpose() << endl;
			}

			break;
		}

		case CMD_CTRL:
		{
			if (!odom_is_received(now_time) || !odom_state_valid())
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;
				exec_traj_state_ = HOVER;

				ROS_WARN("[安全] CMD_CTRL 因里程计失效退出自动控制。");
			}
			else if (rc_data.enter_land_mode)
			{
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
				ROS_WARN("[AUTO_LAND] CH10 触发：CMD_CTRL -> AUTO_LAND。");
			}
			else if (rc_data.mode_valid && !rc_data.is_command_mode)
			{
				// 高位退出命令模式后，低位和中位都回到悬停；已在空中的低位不重复启动起飞。
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_mode_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_HOVER;
				ROS_INFO("[AUTO_HOVER] CH8 低位/中位：CMD_CTRL -> AUTO_HOVER，并锁定当前位置。");
			}
			else
			{
				CMD_CTRL_process();
			}

			break;
		}
		case AUTO_TAKEOFF:
		{
			if (rc_data.mode_valid && rc_data.enter_land_mode)
			{
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
				ROS_WARN("[AUTO_LAND] CH10 触发：AUTO_TAKEOFF -> AUTO_LAND。");
				break;
			}
			if (rc_data.mode_valid && rc_data.is_hover_mode)
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
			if (rc_data.mode_valid && rc_data.is_command_mode)
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
			if (!odom_is_received(now_time) || !odom_state_valid())
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;
				exec_traj_state_ = HOVER;
				ROS_WARN("[安全] AUTO_LAND 因里程计失效停止 NMPC，等待 PX4 OFFBOARD 失联保护。");
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
				const bool retry_allowed = !auto_land_request_sent_ ||
					last_auto_land_request_time_.isZero() ||
					(now_time - last_auto_land_request_time_).toSec() >= 1.0;
				if (retry_allowed && request_px4_auto_land())
				{
					auto_land_request_sent_ = true;
					last_auto_land_request_time_ = now_time;
					ROS_INFO("[AUTO_LAND] 已请求 PX4 AUTO.LAND，等待 PX4 状态确认。");
				}
			}

			break;
		}
			default:
				break;
			}
		}

		if (fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL ||
			fsm_state == AUTO_TAKEOFF || fsm_state == AUTO_LAND)
		{
			if (!mpc_result_handled && !controller_.lastMpcSolveSuccessful())
				beginMpcRecovery(now_time);

			const bool use_last_valid_mpc = !direct_auto_land_active_ &&
				!controller_.lastMpcSolveSuccessful() && canReuseLastValidMpc(now_time);
			if (use_last_valid_mpc)
				publish_bodyrate_ctrl(controller_.lastValidControlInput(), now_time);
			else if (mpc_recovery_active_ && !controller_.lastMpcSolveSuccessful())
				publish_recovery_attitude_ctrl(now_time);
			else
				publish_bodyrate_ctrl(mpc_predicted_inputs_.col(0), now_time);
			if (!direct_auto_land_active_)
				publishPrediction(controller_.reference_states_, mpc_predicted_states_, now_time, controller_.getTimeStep());
		}
		else if (fsm_state == MANUAL_CTRL)
		{
			// 对齐 IPC：Manual 下不求解 NMPC、不请求 OFFBOARD，但持续发布小推力 setpoint，
			// 避免 /mavros/setpoint_raw/attitude 保留上一帧自动控制输出。
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
				controller_.setTrajectoyReference(traj_info->traj, traj_time, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

				exec_traj_state_ = POLY_TRAJ;
				ROS_INFO("[轨迹] 收到有效轨迹：HOVER -> POLY_TRAJ。");
			}
			else
			{
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
			}
		}

		break;

		case POLY_TRAJ:
		{
			const bool queue_empty = trajectory_data.traj_queue.empty();
			const bool normal_end = !queue_empty &&
				trajectory_data.exec_traj == 1 &&
				now_time > trajectory_data.total_traj_end_time;
			const bool should_stop = queue_empty ||
				now_time < trajectory_data.total_traj_start_time ||
				normal_end || trajectory_data.exec_traj != 1;

			if (should_stop)
			{
				if (params_.use_trajectory_ending_pos_ && normal_end &&
					trajectory_data.last_end_position_valid)
				{
					// 只有完整轨迹正常结束时才使用缓存的轨迹终点；中止、空队列和异常均停在当前位置。
					hover_pose_ = trajectory_data.last_end_position;
					hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
				}
				else
				{
					update_hover_pose();
				}
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				exec_traj_state_ = HOVER;
				ROS_INFO("[轨迹] 轨迹执行结束：POLY_TRAJ -> HOVER。");
				trajectory_data.exec_traj = 0;
				trajectory_data.last_end_position_valid = false;
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
							if (trajectory_data.traj_queue.empty())
							{
								update_hover_pose();
								controller_.setHoverReference(hover_pose_, hover_yaw_);
								controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
								exec_traj_state_ = HOVER;
								trajectory_data.exec_traj = 0;
								trajectory_data.last_end_position_valid = false;
								return;
							}
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
					addRMSE();
					controller_.setTrajectoyReference(traj_info->traj, traj_time, hover_yaw_);
					controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				}
			}
		}
		break;

		case POINTS:
		{
			exec_traj_state_ = HOVER;
			ROS_ERROR("[轨迹] 未知轨迹执行状态，切换为悬停。");
		}
		break;

		default:
		{
			exec_traj_state_ = HOVER;
			ROS_ERROR("[轨迹] 未知轨迹执行状态，切换为悬停。");
		}

		break;
		}
	}

	void MPCFSM::setEstimateState(const Odom_Data_t &translation_odom,
							  const Odom_Data_t &attitude_odom)
	{
		const double q_norm = attitude_odom.q.norm();
		if (!translation_odom.p.allFinite() || !translation_odom.v.allFinite() ||
			!attitude_odom.q.coeffs().allFinite() || !std::isfinite(q_norm) || q_norm <= 1.0e-6)
		{
			// 无效组合状态不进入求解器；自动状态会在 process() 开头触发安全保护。
			est_state_.setZero();
			est_state_(kOriW) = 1.0;
			return;
		}
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

	void MPCFSM::addNewForceObseverState()
	{
		force_observer_input_valid_ = false;
		if (!params_.force_estimator_param_.enable_force_estimation)
		{
			clearForceObserverState();
			return;
		}

		// 起飞阶段的加速度主要来自起飞瞬态，不作为外力 f_Q 估计；
		// 基础 NMPC 仍继续运行，进入悬停/命令模式后再重新收集样本。
		if (fsm_state == AUTO_TAKEOFF)
		{
			clearForceObserverState();
			return;
		}

		const ros::Time now = ros::Time::now();
		const bool extended_state_fresh = !extended_state_data.rcv_stamp.isZero() &&
			(now - extended_state_data.rcv_stamp).toSec() < params_.msg_timeout_.extended_state;
		if (extended_state_fresh)
		{
			const bool in_air = extended_state_data.current_extended_state.landed_state ==
				mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
			if (in_air && !was_in_air_)
			{
				// 地面支持力不属于自由飞行动力学；离地时清空地面窗口，仅保留新的空中样本。
				clearForceObserverState();
				airborne_since_ = now;
				ROS_INFO("[外力] 检测到飞行状态，已重置估计窗口并重新采样。");
			}
			else if (!in_air)
			{
				airborne_since_ = ros::Time(0);
			}
			was_in_air_ = in_air;
		}

		const bool input_fresh = !imu_data.rcv_stamp.isZero() && !odom_data.rcv_stamp.isZero() &&
			!force_attitude_odom_data.rcv_stamp.isZero() &&
			!rpm_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom &&
			(now - force_attitude_odom_data.rcv_stamp).toSec() < params_.msg_timeout_.force_attitude_odom &&
			(now - rpm_data.rcv_stamp).toSec() < params_.msg_timeout_.rpm;
		const bool input_finite = imu_data.filtered_a.allFinite() &&
			force_attitude_odom_data.q.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && force_attitude_odom_data.q.norm() > 1.0e-6;
		if (!input_fresh || !input_finite)
		{
			ROS_WARN_THROTTLE(5.0, "[外力] 估计输入超时或包含非有限值，当前估计与补偿已清零。");
			clearForceObserverState();
			return;
		}

		if (rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
		{
			// 地面未解锁时低 RPM 属于正常状态，仅通过门控状态变化提示，不重复报错。
			clearForceObserverState();
			return;
		}
		// 加速度来自 MAVROS IMU 机体系；姿态与 NMPC 共用 MAVROS local odom。
		// 两个传感器的机体系安装偏差必须在标定中消除，否则会形成虚假外力分量。
		force_estimator_.setSystemState(
			imu_data.filtered_a, force_attitude_odom_data.q, rpm_data.filtered_rpm);
		force_observer_input_valid_ = true;
	}

	MPCFSM::ThrustModelGateReason MPCFSM::thrustModelGate(const ros::Time &now) const
	{
		if (params_.thr_map_.accurate_thrust_model != 1)
			return ThrustModelGateReason::DISABLED_BY_PARAM;
		if (mpc_recovery_active_ || direct_auto_land_active_)
			return ThrustModelGateReason::MODE_BLOCKED;
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
		case ThrustModelGateReason::DISABLED_BY_PARAM: name = "参数已关闭"; break;
		case ThrustModelGateReason::MODE_BLOCKED: name = "当前模式禁止学习"; break;
		case ThrustModelGateReason::STATE_STALE: name = "PX4 状态超时"; break;
		case ThrustModelGateReason::DISARMED: name = "飞控未解锁"; break;
		case ThrustModelGateReason::NOT_OFFBOARD: name = "尚未进入 OFFBOARD"; break;
		case ThrustModelGateReason::EXTENDED_STATE_STALE: name = "PX4 扩展状态超时"; break;
		case ThrustModelGateReason::LANDED: name = "当前处于地面"; break;
		case ThrustModelGateReason::RPM_STALE: name = "RPM 数据超时"; break;
		case ThrustModelGateReason::RPM_INVALID: name = "RPM 低于学习门限"; break;
		case ThrustModelGateReason::ACTIVE: name = "在线学习已生效"; break;
		}

		if (reason != last_thrust_model_gate_reason_)
		{
			ROS_INFO("[推力映射] 门控状态：%s。", name);
			last_thrust_model_gate_reason_ = reason;
		}
	}

	MPCFSM::DisturbanceGateReason MPCFSM::disturbanceCompensationGate(const ros::Time &now) const
	{
		if (!params_.force_estimator_param_.enable_force_estimation ||
			!params_.force_estimator_param_.enable_disturbance_compensation)
			return DisturbanceGateReason::DISABLED_BY_PARAM;
		if (mpc_recovery_active_ || direct_auto_land_active_)
			return DisturbanceGateReason::MODE_BLOCKED;
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
		if (!rpm_data.filtered_rpm.allFinite() ||
			rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
			return DisturbanceGateReason::RPM_INVALID;
		const bool sensor_fresh = !imu_data.rcv_stamp.isZero() && !odom_data.rcv_stamp.isZero() &&
			!force_attitude_odom_data.rcv_stamp.isZero() &&
			!rpm_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom &&
			(now - force_attitude_odom_data.rcv_stamp).toSec() < params_.msg_timeout_.force_attitude_odom &&
			(now - rpm_data.rcv_stamp).toSec() < params_.msg_timeout_.rpm;
		const bool sensor_finite = imu_data.filtered_a.allFinite() &&
			force_attitude_odom_data.q.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && force_attitude_odom_data.q.norm() > 1.0e-6;
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
		case DisturbanceGateReason::DISABLED_BY_PARAM: name = "参数已关闭"; break;
		case DisturbanceGateReason::MODE_BLOCKED: name = "当前模式禁止补偿"; break;
		case DisturbanceGateReason::STATE_STALE: name = "PX4 状态超时"; break;
		case DisturbanceGateReason::DISARMED: name = "飞控未解锁"; break;
		case DisturbanceGateReason::NOT_OFFBOARD: name = "尚未进入 OFFBOARD"; break;
		case DisturbanceGateReason::EXTENDED_STATE_STALE: name = "PX4 扩展状态超时"; break;
		case DisturbanceGateReason::LANDED: name = "当前处于地面"; break;
		case DisturbanceGateReason::AIRBORNE_DELAY: name = "等待空中稳定时间"; break;
		case DisturbanceGateReason::SENSOR_INVALID: name = "传感器数据无效"; break;
		case DisturbanceGateReason::RPM_INVALID: name = "RPM 低于有效门限"; break;
		case DisturbanceGateReason::WINDOW_NOT_FULL: name = "估计窗口尚未填满"; break;
		case DisturbanceGateReason::ESTIMATOR_FAILED: name = "外力估计失败"; break;
		case DisturbanceGateReason::ACTIVE: name = "外力补偿已生效"; break;
		}
		ROS_INFO("[外力] 补偿门控状态：%s。", name);
		last_gate_reason_ = reason;
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

	void MPCFSM::clearAppliedDisturbance()
	{
		fq_applied_.setZero();
		controller_.setExternalForce(fq_applied_);
	}

	bool MPCFSM::odom_state_valid() const
	{
		const double q_norm = force_attitude_odom_data.q.norm();
		return odom_data.p.allFinite() && odom_data.v.allFinite() &&
			force_attitude_odom_data.q.coeffs().allFinite() &&
			std::isfinite(q_norm) && q_norm > 1.0e-6;
	}

	void MPCFSM::clear_autonomous_inputs()
	{
		controller_.waitForPreparation();
		trajectory_data.total_traj_start_time = ros::Time(0);
		trajectory_data.total_traj_end_time = ros::Time(0);
		trajectory_data.traj_queue.clear();
		trajectory_data.exec_traj = 0;
		trajectory_data.last_end_position_valid = false;
		exec_traj_state_ = HOVER;
		mpc_recovery_active_ = false;
		direct_auto_land_active_ = false;
		mpc_recovery_success_count_ = 0;
		last_mpc_recovery_reset_time_ = ros::Time(0);

		cmd_data.rcv_stamp = ros::Time(0);
		cmd_data.p.setZero();
		cmd_data.v.setZero();
		cmd_data.a.setZero();
		cmd_data.j.setZero();
		cmd_data.yaw = 0.0;
		cmd_data.yaw_rate = 0.0;
		takeoff_requested_ = false;
		auto_land_request_sent_ = false;
		last_auto_land_request_time_ = ros::Time(0);
		clearForceObserverState();
		controller_.clearThrustCommandHistory();
		controller_.clearLastValidControl();
		// 所有 ACADO 在线数据清理完成后再恢复正常约束并启动下一轮准备线程。
		controller_.restoreNominalVelocityLimits();
	}

	void MPCFSM::beginMpcRecovery(const ros::Time &now, const char *reason)
	{
		if (mpc_recovery_active_ || direct_auto_land_active_)
			return;

		if (!odom_state_valid() || !est_state_.allFinite())
		{
			enter_odom_failsafe("NMPC 求解失败且无法锁存有效里程计位置");
			return;
		}

		hover_pose_ = odom_data.p;
		hover_yaw_ = get_yaw_from_quaternion(force_attitude_odom_data.q);
		last_set_hover_pose_time = now;
		trajectory_data.blockTrajectoryAcceptance();
		exec_traj_state_ = MPC_RECOVERY_HOVER;
		mpc_recovery_start_time_ = now;
		last_mpc_recovery_reset_time_ = now;
		mpc_recovery_success_count_ = 0;
		mpc_recovery_active_ = true;
		fq_estimated_.setZero();
		fq_applied_.setZero();

		// 起飞阶段失败后不能重新跳回基于时间推进的爬升参考。
		// 轨迹模式保持 CMD_CTRL，恢复后由规划器的新轨迹继续任务。
		if (fsm_state == AUTO_TAKEOFF)
			fsm_state = AUTO_HOVER;

		if (!controller_.resetForHover(est_state_, hover_pose_, hover_yaw_))
		{
			ROS_ERROR_THROTTLE(5.0, "[安全] NMPC 求解器重置未完成，继续锁存悬停位置并重试。");
			return;
		}

		ROS_ERROR("[安全] %s：锁存悬停点 (%.3f, %.3f, %.3f)，"
				  "进入 MPC_RECOVERY_HOVER；故障期间轨迹将被丢弃。",
				  reason == nullptr ? "NMPC 求解失败" : reason,
				  hover_pose_.x(), hover_pose_.y(), hover_pose_.z());
	}

	void MPCFSM::processMpcRecovery(const ros::Time &now)
	{
		if (rc_data.enter_land_mode)
		{
			beginDirectAutoLand(now, "安全恢复期间收到人工降落请求");
			return;
		}

		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

		// 等速度明显收敛后再恢复名义硬约束，避免在约束边界附近因噪声反复重置。
		const bool velocity_ready_for_nominal =
			odom_data.v.head<2>().norm() <= params_.safety_.mpc_recovery_exit_speed_xy &&
			std::abs(odom_data.v.z()) <= params_.safety_.mpc_recovery_exit_speed_z;
		if (controller_.lastMpcSolveSuccessful() &&
			controller_.recoveryVelocityLimitsRelaxed() && velocity_ready_for_nominal)
		{
			if (!controller_.restoreNominalVelocityLimits())
				ROS_ERROR_THROTTLE(5.0, "[NMPC恢复] 无法恢复配置的速度硬约束，继续留在恢复状态。");
		}

		const double deg_to_rad = M_PI / 180.0;
		const bool state_converged = recoveryStateConverged(
			odom_data.p, hover_pose_, odom_data.v, force_attitude_odom_data.q,
			params_.safety_.mpc_recovery_exit_position_error,
			params_.safety_.mpc_recovery_exit_speed_xy,
			params_.safety_.mpc_recovery_exit_speed_z,
			params_.safety_.mpc_recovery_exit_tilt_deg * deg_to_rad);
		if (controller_.lastMpcSolveSuccessful() && !odom_spike_guard_.faultActive() &&
			!controller_.recoveryVelocityLimitsRelaxed() && state_converged)
			++mpc_recovery_success_count_;
		else
			mpc_recovery_success_count_ = 0;

		const double elapsed = std::max((now - mpc_recovery_start_time_).toSec(), 0.0);
		if (mpc_recovery_success_count_ >= params_.safety_.mpc_recovery_success_cycles &&
			 controller_.lastMpcSolveSuccessful())
		{
			mpc_recovery_active_ = false;
			exec_traj_state_ = HOVER;
			trajectory_data.allowTrajectoryAcceptanceAfter(now);
			std_msgs::Empty restart_msg;
			planning_restart_pub_.publish(restart_msg);
			ROS_INFO("[安全] NMPC 已成功恢复 %d 次；保持锁存悬停并请求 Diff 从最新里程计重新规划，"
					 "仅接受 %.6f 之后生成的新轨迹。",
					 params_.safety_.mpc_recovery_success_cycles, now.toSec());
			return;
		}

		if (elapsed >= params_.safety_.mpc_recovery_timeout)
		{
			ROS_WARN_THROTTLE(5.0,
				"[安全] NMPC 恢复已超过 %.2f s，继续锁存悬停并后台重试，不因求解失败自动降落。",
				params_.safety_.mpc_recovery_timeout);
		}

		if (!controller_.lastMpcSolveSuccessful() &&
			(now - last_mpc_recovery_reset_time_).toSec() >=
				params_.safety_.mpc_recovery_full_reset_period)
		{
			last_mpc_recovery_reset_time_ = now;
			if (!controller_.resetForHover(est_state_, hover_pose_, hover_yaw_))
				ROS_ERROR_THROTTLE(5.0, "[NMPC恢复] 周期性完整重置失败，继续发布拉平姿态并重试。");
		}
	}

	void MPCFSM::beginDirectAutoLand(const ros::Time &now, const char *reason)
	{
		if (!direct_auto_land_active_)
		{
			mpc_recovery_active_ = false;
			direct_auto_land_active_ = true;
			last_mpc_recovery_reset_time_ = ros::Time(0);
			fsm_state = AUTO_LAND;
			exec_traj_state_ = MPC_RECOVERY_HOVER;
			trajectory_data.blockTrajectoryAcceptance();
			mpc_recovery_success_count_ = 0;
			fq_estimated_.setZero();
			fq_applied_.setZero();
			controller_.waitForPreparation();

			if (!planning_stop_sent_)
			{
				std_msgs::Empty stop_msg;
				planning_stop_pub_.publish(stop_msg);
				planning_stop_sent_ = true;
			}

			auto_land_request_sent_ = false;
			last_auto_land_request_time_ = ros::Time(0);
			ROS_ERROR("[安全] %s：停止规划并直接请求 PX4 AUTO.LAND。", reason);
		}
		processDirectAutoLand(now);
	}

	void MPCFSM::processDirectAutoLand(const ros::Time &now)
	{
		const bool retry_allowed = last_auto_land_request_time_.isZero() ||
			(now - last_auto_land_request_time_).toSec() >=
				params_.safety_.auto_land_retry_period;
		if (!retry_allowed)
			return;

		// Record every attempt, including rejected calls, so a failed service does
		// not get hammered at the 100 Hz controller rate.
		last_auto_land_request_time_ = now;
		auto_land_request_sent_ = request_px4_auto_land();
		if (auto_land_request_sent_)
			ROS_WARN("[AUTO_LAND] PX4 已接受 AUTO.LAND 请求，等待状态确认。");
	}

	void MPCFSM::enter_odom_failsafe(const char *reason)
	{
		clear_autonomous_inputs();
		fsm_state = MANUAL_CTRL;
		suppress_manual_setpoint_ = true;
		odom_failsafe_active_ = true;
		odom_failsafe_deadline_ = ros::Time::now() + ros::Duration(0.3);
		ROS_ERROR_THROTTLE(1.0,
			"[安全] 混合状态失效（%s），停止 NMPC；最多保持最后安全 setpoint 0.3 s，随后停止发送，等待 PX4 OFFBOARD 失联保护。",
			reason == nullptr ? "unknown" : reason);
	}

	void MPCFSM::publish_failsafe_hold(const ros::Time &stamp)
	{
		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");
		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		Eigen::Vector3d bodyrates = last_safe_body_rate_;
		if (!last_safe_setpoint_valid_ || !bodyrates.allFinite())
		{
			bodyrates.setZero();
		}
		msg.body_rate.x = bodyrates.x();
		msg.body_rate.y = bodyrates.y();
		msg.body_rate.z = bodyrates.z();

		// thrust 是 MAVROS/PX4 的归一化推力命令，不是牛顿力；失效保护只保持最后有限值。
		double normalized_thrust = last_safe_normalized_thrust_;
		if (!last_safe_setpoint_valid_ || !std::isfinite(normalized_thrust))
		{
			normalized_thrust = params_.thr_map_.hover_percentage;
		}
		normalized_thrust = std::max(0.0, std::min(normalized_thrust,
			params_.thr_map_.max_normalized_thrust));
		msg.thrust = normalized_thrust;
		ctrl_FCU_pub.publish(msg);
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
			const auto &axis_gain = params_.force_estimator_param_;
			fq_applied_(0) *= axis_gain.force_axis_gain_x;
			fq_applied_(1) *= axis_gain.force_axis_gain_y;
			fq_applied_(2) *= axis_gain.force_axis_gain_z;
			const double scaled_norm = fq_applied_.norm();
			const double max_applied_force = params_.force_estimator_param_.max_applied_force;
			if (scaled_norm > max_applied_force)
			{
				fq_applied_ *= max_applied_force / scaled_norm;
				ROS_WARN_THROTTLE(
					1.0,
					"[MPC CTRL] Disturbance compensation saturated: scaled_norm=%.3f N, applied_norm=%.3f N",
					scaled_norm,
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
			ROS_INFO("[跟踪] 三维位置 RMSE=%.4f m，水平 RMSE=%.4f m。", drone_rmse, drone_rmse_xy);
			ROS_INFO("[跟踪] 三维最大位置误差=%.4f m，水平最大误差=%.4f m。", drone_max, drone_max_xy);
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
		if (!odom_data.p.allFinite() || !odom_data.v.allFinite() ||
			!force_attitude_odom_data.q.coeffs().allFinite() ||
			force_attitude_odom_data.q.norm() <= 1.0e-6 ||
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
		clearForceObserverState();
		controller_.resetThrustMapping();
		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
		fsm_state = AUTO_TAKEOFF;
		ROS_INFO("[AUTO_TAKEOFF] 开始起飞：目标高度=%.3f m，上升速度=%.3f m/s。",
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
		ROS_INFO("[AUTO_HOVER] 固定悬停参考：位置=(%.3f, %.3f, %.3f) m，yaw=%.3f rad。",
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
		(void)now_time;
		return !odom_data.rcv_stamp.isZero() &&
			!force_attitude_odom_data.rcv_stamp.isZero();
	}

	bool MPCFSM::imu_is_received(const ros::Time &now_time) const
	{
		return (now_time - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu;
	}

	bool MPCFSM::bat_is_received(const ros::Time &now_time) const
	{
		return (now_time - bat_data.rcv_stamp).toSec() < params_.msg_timeout_.bat;
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

	bool MPCFSM::canReuseLastValidMpc(const ros::Time &now) const
	{
		if (odom_spike_guard_.faultActive())
			return false;

		if (!controller_.hasRecentValidControl(
				now, params_.safety_.mpc_recovery_last_valid_hold))
		{
			return false;
		}

		const Eigen::Vector4d cached_input =
			controller_.lastValidControlInput().cast<double>();
		return conservativeLastValidInput(
			odom_data.v, force_attitude_odom_data.q, cached_input,
			params_.safety_.mpc_recovery_exit_speed_xy,
			params_.safety_.mpc_recovery_exit_speed_z,
			params_.safety_.mpc_recovery_exit_tilt_deg * M_PI / 180.0,
			params_.safety_.mpc_recovery_last_valid_max_bodyrate,
			params_.min_thrust_, params_.max_thrust_);
	}

	void MPCFSM::publish_recovery_attitude_ctrl(const ros::Time &stamp)
	{
		const RecoveryAttitudeCommand command = makeRecoveryAttitudeCommand(
			force_attitude_odom_data.q, hover_yaw_,
			controller_.currentHoverPercentage(),
			params_.thr_map_.max_normalized_thrust,
			params_.safety_.mpc_recovery_max_thrust_comp_tilt_deg * M_PI / 180.0);
		if (!command.valid)
		{
			ROS_ERROR_THROTTLE(1.0,
				"[NMPC恢复] 无法生成有限的拉平姿态目标，退回零角速度和悬停推力。");
			Eigen::Matrix<real_t, kInputSize, 1> hover_input;
			hover_input << params_.dyn_params_.mass_q * params_.gravity_, 0.0, 0.0, 0.0;
			publish_bodyrate_ctrl(hover_input, stamp);
			return;
		}

		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");
		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
			mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
			mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
		msg.orientation.x = command.orientation.x();
		msg.orientation.y = command.orientation.y();
		msg.orientation.z = command.orientation.z();
		msg.orientation.w = command.orientation.w();
		msg.body_rate.x = 0.0;
		msg.body_rate.y = 0.0;
		msg.body_rate.z = 0.0;
		msg.thrust = command.normalized_thrust;

		last_safe_body_rate_.setZero();
		last_safe_normalized_thrust_ = command.normalized_thrust;
		last_safe_setpoint_valid_ = true;
		ctrl_FCU_pub.publish(msg);
		ROS_WARN_THROTTLE(1.0,
			"[NMPC恢复] 当前无有效 MPC 输出，发布水平姿态目标并保持锁存 yaw；"
			"当前倾角 %.1f deg，归一化推力 %.3f。",
			command.tilt_rad * 180.0 / M_PI, command.normalized_thrust);
	}

	void MPCFSM::publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
									   const ros::Time &stamp)
	{
		mavros_msgs::AttitudeTarget msg;

		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");

		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		const bool input_finite = predicted_input.allFinite();
		Eigen::Vector3d bodyrates = Eigen::Vector3d::Zero();
		if (input_finite)
		{
			bodyrates << predicted_input(INPUT_BODYRATE::kRateX),
				predicted_input(INPUT_BODYRATE::kRateY),
				predicted_input(INPUT_BODYRATE::kRateZ);
		}
		const double max_bodyrate_xy = std::max(0.0, static_cast<double>(params_.max_bodyrate_xy_));
		const double max_bodyrate_z = std::max(0.0, static_cast<double>(params_.max_bodyrate_z_));
		bodyrates.x() = std::max(-max_bodyrate_xy, std::min(bodyrates.x(), max_bodyrate_xy));
		bodyrates.y() = std::max(-max_bodyrate_xy, std::min(bodyrates.y(), max_bodyrate_xy));
		bodyrates.z() = std::max(-max_bodyrate_z, std::min(bodyrates.z(), max_bodyrate_z));

		msg.body_rate.x = bodyrates[0];
		msg.body_rate.y = bodyrates[1];
		msg.body_rate.z = bodyrates[2];

		// body_rate.x/y/z 是发送给 MAVROS/PX4 的机体系角速度命令，单位通常为 rad/s。
		// AttitudeTarget.thrust 是 PX4 归一化推力命令，不是 NMPC 内部的牛顿推力 T。
		// body_rate.x/y/z 是发送给 MAVROS/PX4 的机体系角速度命令，单位 rad/s。
		// AttitudeTarget.thrust 是 MAVROS/PX4 归一化推力命令，不是 NMPC 内部的牛顿推力。
		double normalized_thrust = params_.thr_map_.hover_percentage;
		if (input_finite)
		{
			const double thrust_input = predicted_input(INPUT_BODYRATE::kThrust);
			normalized_thrust = params_.use_simulation_
				? thrust_input
				: controller_.convertThrust(thrust_input, bat_data.volt);
		}
		if (!std::isfinite(normalized_thrust))
		{
			normalized_thrust = params_.thr_map_.hover_percentage;
		}
		normalized_thrust = std::max(0.0, std::min(normalized_thrust,
			params_.thr_map_.max_normalized_thrust));

		last_safe_body_rate_ = bodyrates;
		last_safe_normalized_thrust_ = normalized_thrust;
		last_safe_setpoint_valid_ = true;
		msg.thrust = normalized_thrust;

		ctrl_FCU_pub.publish(msg);
	}

	void MPCFSM::publish_manual_ctrl(const ros::Time &stamp)
	{
		if (suppress_manual_setpoint_ || !rc_data.mode_valid || !rc_data.is_takeoff_mode ||
			state_data.current_state.mode == "OFFBOARD" || manual_setpoint_published_)
		{
			return;
		}
		manual_setpoint_published_ = true;
		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");
		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		// Manual 输出只用于清空上一帧自动控制 setpoint：机体系角速度命令为 0 rad/s。
		msg.body_rate.x = 0.0;
		msg.body_rate.y = 0.0;
		msg.body_rate.z = 0.0;
		// 0.01 只用于 PX4 进入 OFFBOARD 前的安全预流；这是 MAVROS/PX4 归一化推力，不是牛顿力。
		// 一旦 PX4 已进入 OFFBOARD，本函数不得发布该低推力，必须由实际自动控制输出接管。
		msg.thrust = 0.01;

		ctrl_FCU_pub.publish(msg);
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
		mavros_msgs::SetMode land_set_mode;
		// PX4 AUTO.LAND 由飞控接管最终降落；本函数不负责解锁或起飞。
		land_set_mode.request.custom_mode = "AUTO.LAND";
		if (!(set_FCU_mode_srv.call(land_set_mode) && land_set_mode.response.mode_sent))
		{
			ROS_ERROR_THROTTLE(5.0, "[AUTO_LAND] PX4 未接受 AUTO.LAND，请求将继续重试。");
			return false;
		}
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

		ROS_INFO("[飞控] 正在请求重启 PX4。");

		// if (params_.print_dbg)
		// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
	}

}
