#include "mpc_fsm.h"
#include <uav_utils/converters.h>
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <cstring>
using namespace std;
using namespace uav_utils;
#define USE_PX4_OR_ARDUPILOT 0 // 0: PX4, 1: ArduPilot. 本项目按 PX4 + MAVROS OFFBOARD 使用。
namespace PayloadMPC
{

	MPCFSM::MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller) : nh_(nh),
																							  params_(params),
																							  controller_(controller)
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

		pub_rmse_info_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/rmse_info", 1);

		force_estimator_.init(params_);
		rc_data.set_mode_params(params_.rc_mode_.mode_channel,
								params_.rc_mode_.land_channel,
								params_.rc_mode_.low_threshold,
								params_.rc_mode_.mid_low_threshold,
								params_.rc_mode_.mid_high_threshold,
								params_.rc_mode_.high_threshold);
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

		setEstimateState(odom_data);
		setForceEstimation();
		if (params_.use_simulation_)
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

		switch (fsm_state)
		{
		case MANUAL_CTRL:
		{
			// CH8 中位请求 AUTO_TAKEOFF；CH6 只由 QGC/PX4 负责切换 OFFBOARD。
			if (!rc_data.is_takeoff_mode)
			{
				takeoff_requested_ = false;
				takeoff_request_latched_ = false;
			}
			else if (params_.takeoff_.enabled && !takeoff_request_latched_)
			{
				takeoff_request_latched_ = true;
				takeoff_requested_ = true;
				ROS_INFO("[MPCctrl] CH8 middle: AUTO_TAKEOFF requested. Waiting for PX4 OFFBOARD and safety checks.");
			}

			// AUTO_TAKEOFF 只在 PX4 已经进入 OFFBOARD 后启动；CH6 由 PX4/QGC 的 RC_MAP_OFFB_SW 处理。
			if (takeoff_requested_)
			{
				const char *reason = nullptr;
				if (!takeoffPreconditions(now_time, reason))
				{
					if (std::strcmp(reason, "OFFBOARD") == 0)
					{
						ROS_INFO_THROTTLE(1.0, "[MPCctrl] AUTO_TAKEOFF_WAIT_OFFBOARD.");
					}
					else
					{
						ROS_INFO_THROTTLE(1.0, "[MPCctrl] AUTO_TAKEOFF_WAIT_%s.", reason);
					}
					break;
				}
				startAutoTakeoff(now_time);
				break;
			}

			if (rc_data.is_command_mode)
			{
				if (state_data.current_state.mode != "OFFBOARD")
				{
					ROS_INFO_THROTTLE(1.0, "[MPCctrl] CMD_CTRL waits for PX4 OFFBOARD selected by QGC/CH6.");
					break;
				}
				if (!odom_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] Reject entering control mode. No odom!");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_ERROR("[MPCctrl] Reject entering control mode. Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
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
				ROS_INFO("\033[32m[MPCctrl] MANUAL_CTRL --> CMD_CTRL by CH8 high.\033[32m");
			}

			if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
			{
				if (state_data.current_state.armed)
				{
					ROS_ERROR("[MPCctrl] Reject reboot! Disarm the drone first!");
					break;
				}
				reboot_FCU();
			}

			break;
		}

		case AUTO_HOVER:
		{
			if (rc_data.is_manual_mode || !odom_is_received(now_time))
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;

				ROS_WARN("[MPCctrl] AUTO_HOVER --> MANUAL_CTRL by CH8 low or odom timeout.");
			}
			else if (rc_data.enter_land_mode)
			{
				// AUTO_LAND 禁止使用风力补偿；先清零 OnlineData，再计算首个降落控制量。
				clearAppliedDisturbance();
				land_start_time_ = now_time;
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[MPCctrl] AUTO_HOVER --> AUTO_LAND by CH10 rising edge.");
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
					ROS_INFO("\033[32m[MPCctrl] AUTO_HOVER --> CMD_CTRL by CH8 high.\033[32m");
					publish_trigger(odom_data.msg);
					ROS_INFO("\033[32m[MPCctrl] TRIGGER sent, allow user command.\033[32m");
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
					ROS_INFO("\033[32m[MPCctrl] TRIGGER sent, allow user command.\033[32m");
				}

				// cout << "des.p=" << des.p.transpose() << endl;
			}

			break;
		}

		case CMD_CTRL:
		{
			if (rc_data.is_manual_mode || !odom_is_received(now_time))
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;
				exec_traj_state_ = HOVER;

				ROS_WARN("[MPCctrl] CMD_CTRL --> MANUAL_CTRL by CH8 low or odom timeout.");
			}
			else if (rc_data.enter_land_mode)
			{
				// AUTO_LAND 禁止使用风力补偿；先清零 OnlineData，再计算首个降落控制量。
				clearAppliedDisturbance();
				land_start_time_ = now_time;
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[MPCctrl] CMD_CTRL --> AUTO_LAND by CH10 rising edge.");
			}
			else
			{
				CMD_CTRL_process();
			}

			break;
		}
		case AUTO_TAKEOFF:
		{
			if (rc_data.is_manual_mode)
			{
				abortAutoTakeoff("CH8_LOW");
				break;
			}
			if (!rc_data.is_takeoff_mode && !rc_data.is_command_mode)
			{
				abortAutoTakeoff("CH8_INVALID");
				break;
			}

			const char *reason = nullptr;
			if (!takeoffRunningSafe(now_time, reason))
			{
				if (std::strcmp(reason, "OFFBOARD") == 0)
				{
					// 日志标识：AUTO_TAKEOFF_ABORTED_OFFBOARD_LOST。
					abortAutoTakeoff("OFFBOARD_LOST");
				}
				else
				{
					abortAutoTakeoff(reason);
				}
				break;
			}

			if (rc_data.enter_land_mode)
			{
				clearAppliedDisturbance();
				takeoff_requested_ = false;
				if (extended_state_data.current_extended_state.landed_state ==
					mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
				{
					abortAutoTakeoff("CH10_ON_GROUND");
					break;
				}
				land_start_time_ = now_time;
				trajectory_data.exec_traj = 0;
				exec_traj_state_ = HOVER;
				update_hover_pose();
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				fsm_state = AUTO_LAND;
				ROS_WARN("[MPCctrl] AUTO_TAKEOFF --> AUTO_LAND by CH10 rising edge.");
				break;
			}

			const double elapsed = std::max((now_time - takeoff_start_time_).toSec(), 0.0);
			hover_pose_ = takeoff_start_pose_;
			hover_pose_(2) = std::min(takeoff_target_z_,
									 takeoff_start_pose_(2) + params_.takeoff_.climb_rate * elapsed);
			hover_yaw_ = takeoff_start_yaw_;
			controller_.setHoverReference(hover_pose_, hover_yaw_);
			controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);

			const bool height_ok = std::abs(odom_data.p(2) - takeoff_target_z_) <=
				params_.takeoff_.position_tolerance;
			const bool vertical_speed_ok = std::abs(odom_data.v(2)) <=
				params_.takeoff_.velocity_tolerance;
			if (height_ok && vertical_speed_ok)
			{
				if (takeoff_settle_start_.isZero())
				{
					takeoff_settle_start_ = now_time;
				}
				else if ((now_time - takeoff_settle_start_).toSec() >= params_.takeoff_.settle_time)
				{
					completeAutoTakeoff();
				}
			}
			else
			{
				takeoff_settle_start_ = ros::Time(0);
			}

			if (fsm_state == AUTO_TAKEOFF && elapsed > params_.takeoff_.timeout)
			{
				abortAutoTakeoff("TIMEOUT");
			}
			break;
		}
		case AUTO_LAND:
		{
			if (rc_data.is_manual_mode || !odom_is_received(now_time))
			{
				clearAppliedDisturbance();
				fsm_state = MANUAL_CTRL;
				exec_traj_state_ = HOVER;
				ROS_WARN("[MPCctrl] AUTO_LAND --> MANUAL_CTRL by CH8 low or odom timeout.");
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
			if (low_enough || timeout)
			{
				// 方案1：CH10 只控制 AutoTrans 的 AUTO_LAND；不调用 PX4 AUTO.LAND 服务。
				// 目标降到最低高度后保持该状态，操作者通过 CH8/PX4 安全流程退出 OFFBOARD。
				ROS_WARN_THROTTLE(1.0,
					"[MPCctrl] AUTO_LAND reached minimum/timeout; waiting for CH8 low or PX4/QGC landing.");
			}

			break;
		}
		default:
			break;
		}

		if (fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL ||
			fsm_state == AUTO_TAKEOFF || fsm_state == AUTO_LAND)
		{
			publish_bodyrate_ctrl(mpc_predicted_inputs_.col(0), now_time);
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
				imu_data.a, odom_data.q, rpm_data.rpm_vec, bat_data.volt, params_);
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
				ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> POLY_TRAJ");
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
			if (now_time < (trajectory_data.total_traj_start_time) || now_time > trajectory_data.total_traj_end_time || trajectory_data.exec_traj != 1 || trajectory_data.traj_queue.empty())
			{
				if (params_.use_trajectory_ending_pos_ && trajectory_data.exec_traj != -1)
				{
					// tracking the end point of the trajectory
					//  the hover pose is the end point of the trajectory
					auto &traj_info = trajectory_data.traj_queue.front().traj;
					hover_pose_ = traj_info.getJuncPos(traj_info.getPieceNum());
					hover_yaw_ = get_yaw_from_quaternion(odom_data.q);
				}
				else
				{
					update_hover_pose();
				}
				controller_.setHoverReference(hover_pose_, hover_yaw_);
				controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
				exec_traj_state_ = HOVER;
				ROS_INFO("[MPCctrl] Stop execute the trajectory. POLY_TRAJ --> HOVER");
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
			ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to hover");
		}
		break;

		default:
		{
			exec_traj_state_ = HOVER;
			ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to hover");
		}

		break;
		}
	}

	void MPCFSM::setEstimateState(const Odom_Data_t &odom_est_state)
	{
		est_state_(kPosX) = odom_est_state.p[0];
		est_state_(kPosY) = odom_est_state.p[1];
		est_state_(kPosZ) = odom_est_state.p[2];
		auto rot_q = odom_est_state.q;
		rot_q.normalize();
		est_state_(kOriW) = rot_q.w();
		est_state_(kOriX) = rot_q.x();
		est_state_(kOriY) = rot_q.y();
		est_state_(kOriZ) = rot_q.z();
		est_state_(kVelX) = odom_est_state.v[0];
		est_state_(kVelY) = odom_est_state.v[1];
		est_state_(kVelZ) = odom_est_state.v[2];
	}

	void MPCFSM::addNewForceObseverState()
	{
		force_observer_input_valid_ = false;
		if (!params_.force_estimator_param_.enable_force_estimation)
		{
			force_estimator_.reset();
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
				force_estimator_.reset();
				airborne_since_ = now;
				ROS_INFO("[MPC CTRL] IN_AIR detected. Reset force-estimator window before compensation.");
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
		const bool input_finite = imu_data.filtered_a.allFinite() && odom_data.q.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && odom_data.q.norm() > 1.0e-6;
		if (!input_fresh || !input_finite)
		{
			ROS_ERROR_THROTTLE(1.0, "[MPC CTRL] Force-estimator input stale or non-finite.");
			force_estimator_.reset();
			return;
		}

		if (rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
		{
			ROS_ERROR_THROTTLE(1.0, "[MPC CTRL] RPM below force_estimator/min_valid_rpm.");
			force_estimator_.reset();
			return;
		}
		force_estimator_.setSystemState(imu_data.filtered_a, odom_data.q, rpm_data.filtered_rpm);
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
		const char *name = "UNKNOWN";
		switch (reason)
		{
		case ThrustModelGateReason::DISABLED_BY_PARAM: name = "DISABLED_BY_PARAM"; break;
		case ThrustModelGateReason::MODE_BLOCKED: name = "MODE_BLOCKED"; break;
		case ThrustModelGateReason::STATE_STALE: name = "STATE_STALE"; break;
		case ThrustModelGateReason::DISARMED: name = "DISARMED"; break;
		case ThrustModelGateReason::NOT_OFFBOARD: name = "NOT_OFFBOARD"; break;
		case ThrustModelGateReason::EXTENDED_STATE_STALE: name = "EXTENDED_STATE_STALE"; break;
		case ThrustModelGateReason::LANDED: name = "LANDED"; break;
		case ThrustModelGateReason::RPM_STALE: name = "RPM_STALE"; break;
		case ThrustModelGateReason::RPM_INVALID: name = "RPM_INVALID"; break;
		case ThrustModelGateReason::ACTIVE: name = "ACTIVE"; break;
		}

		if (reason != last_thrust_model_gate_reason_)
		{
			ROS_INFO("[MPC CTRL] Thrust-model gate: %s", name);
			last_thrust_model_gate_reason_ = reason;
		}

		if (reason == ThrustModelGateReason::ACTIVE ||
			reason == ThrustModelGateReason::MODE_BLOCKED ||
			reason == ThrustModelGateReason::DISABLED_BY_PARAM)
		{
			// ACTIVE 和正常模式阻断按 1 Hz 输出，便于飞行日志持续记录当前学习状态。
			ROS_INFO_THROTTLE(1.0, "[MPC CTRL] Thrust-model gate status: %s", name);
		}
		else
		{
			ROS_WARN_THROTTLE(1.0, "[MPC CTRL] Thrust-model gate remains blocked: %s", name);
		}
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
		if (!rpm_data.filtered_rpm.allFinite() ||
			rpm_data.filtered_rpm.minCoeff() < params_.force_estimator_param_.min_valid_rpm)
			return DisturbanceGateReason::RPM_INVALID;
		const bool sensor_fresh = !imu_data.rcv_stamp.isZero() && !odom_data.rcv_stamp.isZero() &&
			!rpm_data.rcv_stamp.isZero() &&
			(now - imu_data.rcv_stamp).toSec() < params_.msg_timeout_.imu &&
			(now - odom_data.rcv_stamp).toSec() < params_.msg_timeout_.odom &&
			(now - rpm_data.rcv_stamp).toSec() < params_.msg_timeout_.rpm;
		const bool sensor_finite = imu_data.filtered_a.allFinite() && odom_data.q.coeffs().allFinite() &&
			rpm_data.filtered_rpm.allFinite() && odom_data.q.norm() > 1.0e-6;
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

		const char *name = "UNKNOWN";
		switch (reason)
		{
		case DisturbanceGateReason::DISABLED_BY_PARAM: name = "DISABLED_BY_PARAM"; break;
		case DisturbanceGateReason::MODE_BLOCKED: name = "MODE_BLOCKED"; break;
		case DisturbanceGateReason::STATE_STALE: name = "STATE_STALE"; break;
		case DisturbanceGateReason::DISARMED: name = "DISARMED"; break;
		case DisturbanceGateReason::NOT_OFFBOARD: name = "NOT_OFFBOARD"; break;
		case DisturbanceGateReason::EXTENDED_STATE_STALE: name = "EXTENDED_STATE_STALE"; break;
		case DisturbanceGateReason::LANDED: name = "LANDED"; break;
		case DisturbanceGateReason::AIRBORNE_DELAY: name = "AIRBORNE_DELAY"; break;
		case DisturbanceGateReason::SENSOR_INVALID: name = "SENSOR_INVALID"; break;
		case DisturbanceGateReason::RPM_INVALID: name = "RPM_INVALID"; break;
		case DisturbanceGateReason::WINDOW_NOT_FULL: name = "WINDOW_NOT_FULL"; break;
		case DisturbanceGateReason::ESTIMATOR_FAILED: name = "ESTIMATOR_FAILED"; break;
		case DisturbanceGateReason::ACTIVE: name = "ACTIVE"; break;
		}
		ROS_INFO("[MPC CTRL] Disturbance compensation gate: %s", name);
		last_gate_reason_ = reason;
	}

	void MPCFSM::clearAppliedDisturbance()
	{
		fq_applied_.setZero();
		controller_.setExternalForce(fq_applied_);
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
			const double estimated_norm = fq_applied_.norm();
			const double max_applied_force = params_.force_estimator_param_.max_applied_force;
			if (estimated_norm > max_applied_force)
			{
				fq_applied_ *= max_applied_force / estimated_norm;
				ROS_WARN_THROTTLE(
					1.0,
					"[MPC CTRL] Disturbance compensation saturated: estimated_norm=%.3f N, applied_norm=%.3f N",
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
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone RMSE = %lf m.\033[32m", drone_rmse);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone RMSE_XY = %lf m.\033[32m", drone_rmse_xy);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone MAX = %lf m.\033[32m", drone_max);
			ROS_INFO("\033[32m[MPCctrl] Tracking Drone MAX_XY = %lf m.\033[32m", drone_max_xy);
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
		if (!odom_is_received(now) || !imu_is_received(now) || !bat_is_received(now) ||
			 rpm_data.rcv_stamp.isZero() || (now - rpm_data.rcv_stamp).toSec() >= params_.msg_timeout_.rpm)
		{
			reason = "SENSOR_STALE";
			return false;
		}
		if (!odom_data.p.allFinite() || !odom_data.v.allFinite() ||
			!odom_data.q.coeffs().allFinite() || odom_data.q.norm() <= 1.0e-6 ||
			!imu_data.filtered_a.allFinite() || !rpm_data.rpm_vec.allFinite() ||
			rpm_data.rpm_vec.minCoeff() < 0.0 || !std::isfinite(bat_data.volt) || bat_data.volt <= 0.0)
		{
			reason = "SENSOR_INVALID";
			return false;
		}
		if (odom_data.v.norm() > 0.5)
		{
			reason = "SPEED_UNSAFE";
			return false;
		}
		if (params_.takeoff_.target_z <= odom_data.p(2) + params_.takeoff_.position_tolerance)
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

	bool MPCFSM::takeoffRunningSafe(const ros::Time &now, const char *&reason) const
	{
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
		const auto landed_state = extended_state_data.current_extended_state.landed_state;
		if (landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
			landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
		{
			reason = "LANDED_STATE_UNKNOWN";
			return false;
		}
		if (!odom_is_received(now) || !imu_is_received(now) || !bat_is_received(now) ||
			rpm_data.rcv_stamp.isZero() || (now - rpm_data.rcv_stamp).toSec() >= params_.msg_timeout_.rpm)
		{
			reason = "SENSOR_STALE";
			return false;
		}
		if (!odom_data.p.allFinite() || !odom_data.v.allFinite() ||
			!odom_data.q.coeffs().allFinite() || odom_data.q.norm() <= 1.0e-6 ||
			!imu_data.filtered_a.allFinite() || !rpm_data.rpm_vec.allFinite() ||
			rpm_data.rpm_vec.minCoeff() < 0.0 || !std::isfinite(bat_data.volt) || bat_data.volt <= 0.0)
		{
			reason = "SENSOR_INVALID";
			return false;
		}
		return true;
	}

	void MPCFSM::startAutoTakeoff(const ros::Time &now)
	{
		takeoff_requested_ = false;
		takeoff_start_pose_ = odom_data.p;
		takeoff_target_z_ = params_.takeoff_.target_z;
		takeoff_start_yaw_ = get_yaw_from_quaternion(odom_data.q);
		takeoff_start_time_ = now;
		takeoff_settle_start_ = ros::Time(0);
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
		ROS_INFO("[MPCctrl] AUTO_TAKEOFF_STARTED: target_z=%.3f m, climb_rate=%.3f m/s.",
				takeoff_target_z_, params_.takeoff_.climb_rate);
	}

	void MPCFSM::abortAutoTakeoff(const char *reason)
	{
		clearAppliedDisturbance();
		fsm_state = MANUAL_CTRL;
		exec_traj_state_ = HOVER;
		takeoff_requested_ = false;
		takeoff_settle_start_ = ros::Time(0);
		ROS_WARN("[MPCctrl] AUTO_TAKEOFF_ABORTED_%s.", reason == nullptr ? "UNKNOWN" : reason);
	}

	void MPCFSM::completeAutoTakeoff()
	{
		if (rc_data.is_manual_mode)
		{
			abortAutoTakeoff("CH8_LOW");
			return;
		}

		if (params_.fixed_hover_.enabled)
		{
			update_mode_hover_pose();
			hover_yaw_ = takeoff_start_yaw_;
		}
		else
		{
			hover_pose_ = takeoff_start_pose_;
			hover_pose_(2) = takeoff_target_z_;
			hover_yaw_ = takeoff_start_yaw_;
			last_set_hover_pose_time = ros::Time::now();
		}

		controller_.setHoverReference(hover_pose_, hover_yaw_);
		controller_.execMPC(est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
		takeoff_requested_ = false;
		takeoff_settle_start_ = ros::Time(0);
		fsm_state = AUTO_HOVER;
		exec_traj_state_ = HOVER;
		ROS_INFO("[MPCctrl] AUTO_TAKEOFF_COMPLETED --> AUTO_HOVER.");
	}

	void MPCFSM::update_hover_pose()
	{
		last_set_hover_pose_time = ros::Time::now();
		hover_pose_ = odom_data.p;
		// hover_pose_(0) = params_.pos_x_;
		// hover_pose_(1) = params_.pos_y_;
		// hover_pose_(2) = params_.takeoff_height_;

		hover_yaw_ = get_yaw_from_quaternion(odom_data.q);
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
		hover_yaw_ = get_yaw_from_quaternion(odom_data.q);
		ROS_INFO("[MPCctrl] Fixed hover reference: p=(%.3f, %.3f, %.3f) m, yaw=%.3f rad.",
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

	void MPCFSM::publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
									   const ros::Time &stamp)
	{
		mavros_msgs::AttitudeTarget msg;

		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");

		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		// double collective_thrust;
		Eigen::Vector3d bodyrates;

		// collective_thrust = input_bounded(INPUT_BODYRATE::kThrust);
		bodyrates << predicted_input(INPUT_BODYRATE::kRateX), predicted_input(INPUT_BODYRATE::kRateY),
			predicted_input(INPUT_BODYRATE::kRateZ);

		msg.body_rate.x = bodyrates[0];
		msg.body_rate.y = bodyrates[1];
		msg.body_rate.z = bodyrates[2];

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

		ctrl_FCU_pub.publish(msg);
	}

	void MPCFSM::publish_manual_ctrl(const ros::Time &stamp)
	{
		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = std::string("FCU");
		msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

		// Manual 输出只用于清空上一帧自动控制 setpoint：机体系角速度命令为 0 rad/s。
		msg.body_rate.x = 0.0;
		msg.body_rate.y = 0.0;
		msg.body_rate.z = 0.0;
		// 与 IPC 保持一致：这里是 MAVROS/PX4 归一化 thrust，不是牛顿推力。
		msg.thrust = 0.05;

		ctrl_FCU_pub.publish(msg);
	}

	void MPCFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
	{
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "world";
		msg.pose = odom_msg.pose.pose;

		traj_start_trigger_pub.publish(msg);
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

		ROS_INFO("Reboot FCU");

		// if (params_.print_dbg)
		// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
	}

}
