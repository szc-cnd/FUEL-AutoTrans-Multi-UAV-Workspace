#ifndef __MPCFSM_H
#define __MPCFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <cstdint>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "mpc_params.h"
#include "mpc_input.h"
#include "mpc_controller.h"
#include "force_attitude_aligner.h"
#include "odom_spike_guard.h"
#include "recovery_control.h"
#include "polynomial_trajectory.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ESCStatus.h>
#include "multi_optimization_based_force_estimator.hpp"

namespace PayloadMPC
{

	class MPCFSM
	{
	public:
		//***************PX4CTRL Real *****************

		RC_Data_t rc_data;
		State_Data_t state_data;
		ExtendedState_Data_t extended_state_data;
		Odom_Data_t odom_data;
		// PX4 EKF 融合里程计提供 NMPC 控制姿态和外力估计姿态；
		// odom_data 只向 NMPC 提供 FAST-LIO 高频位置和线速度。
		Odom_Data_t force_attitude_odom_data;
		Imu_Data_t imu_data;
		Command_Data_t cmd_data;
		Battery_Data_t bat_data;
		Rpm_Data_t rpm_data;

		Cmd_Trigger_Data_t cmd_trigger_data;
		Trajectory_Data_t trajectory_data;

		ros::Publisher traj_start_trigger_pub;
		ros::Publisher ctrl_FCU_pub;
		ros::Publisher pub_force_marker_, pub_force_, pub_force_applied_;
		ros::Publisher pub_force_attitude_aligned_, pub_force_attitude_yaw_offset_;

		ros::Publisher debug_pub; // debug
		ros::ServiceClient set_FCU_mode_srv;
		ros::ServiceClient reboot_FCU_srv;

		ros::Publisher des_yaw_pub;
		ros::Publisher planning_stop_pub_;
		ros::Publisher planning_restart_pub_;

		Eigen::Vector3d hover_pose_;
		double hover_yaw_;
		ros::Time last_set_hover_pose_time;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		enum State_t
		{
				MANUAL_CTRL = 1, // 手动状态：不求解 NMPC；仅在起飞低位且进入 OFFBOARD 前发布安全预流。
			AUTO_HOVER,		 // 自动悬停：发布 body_rate(rad/s) + MAVROS 归一化 thrust。
			CMD_CTRL,		 // 指令/轨迹控制：跟踪轨迹或悬停参考，并持续发布 MAVROS setpoint。
			AUTO_TAKEOFF,	 // 自动起飞：等待 PX4 已进入 OFFBOARD 后平滑爬升，不自动解锁或切 OFFBOARD。
			AUTO_LAND,		 // 自动降落：内部逐步降低悬停高度，末端请求 PX4 AUTO.LAND。
		};

		enum Exec_Traj_State_t
		{
			HOVER = 10,		// 轨迹子状态：没有有效轨迹时保持悬停参考。
			POLY_TRAJ = 11, // 轨迹子状态：执行 PolynomialTraj 多项式轨迹。
			POINTS = 12,	// 轨迹子状态：预留点序列轨迹；当前流程遇到后回到 HOVER。
			MPC_RECOVERY_HOVER = 13, // NMPC 故障期间锁存固定位置并后台恢复。
		};

		MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller);

		void process();
		void CMD_CTRL_process();

		bool rc_is_received(const ros::Time &now_time) const;
		bool odom_is_received(const ros::Time &now_time) const;
		bool imu_is_received(const ros::Time &now_time) const;
		bool recv_new_odom();
		void addNewForceObseverState();
		void landingSearchStateCallback(const std_msgs::String::ConstPtr &msg);
		void landingSearchYawCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
		void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

	private:
		// Subscribers and publisher.
		ros::Publisher pub_control_command, pub_predicted_trajectory_, pub_reference_trajectory_;
		ros::Publisher pub_all_ref_data_, pub_rmse_info_;
		State_t fsm_state; // 只在 MPCFSM::process() 中修改主模式，避免状态跳转分散。

		Exec_Traj_State_t exec_traj_state_;

		// Handles
		ros::NodeHandle nh_;

		MpcParams &params_;
		MpcController &controller_;
		MultiOptForceEstimator force_estimator_;
		ForceAttitudeAligner force_attitude_aligner_;
		OdomSpikeGuard odom_spike_guard_;
		ros::Time land_start_time_;
		bool auto_land_lockout_{false};
		bool auto_land_request_sent_{false};
		ros::Time last_auto_land_request_time_{0};
		bool takeoff_requested_{false};
		// 前视扫描由搜索管理器给出航向，AutoTrans 锁定进入扫描时的 XYZ，仅执行偏航。
		bool landing_search_yaw_active_{false};
		// CH9 搜索任务全程的航向覆盖：前视阶段扫描，下视/接近阶段固定为 CH9 航向。
		bool landing_search_yaw_override_active_{false};
		bool landing_search_hold_latched_{false};
		bool have_landing_search_yaw_{false};
		double landing_search_yaw_{0.0};
		double landing_search_yaw_timeout_{0.5};
		ros::Time last_landing_search_yaw_time_{0};
		// CH8 低位触发一次起飞；失败后必须离开低位再重新进入，避免循环反复重启。
		bool takeoff_request_latched_{false};
		// 仅在起飞前置条件的失败原因变化时输出提示，避免控制周期反复刷屏。
		std::string last_takeoff_precondition_reason_;
		bool hover_offboard_wait_reported_{false};
		bool cmd_offboard_wait_reported_{false};
			bool odom_failsafe_active_{false};
			ros::Time odom_failsafe_start_{0};
			bool suppress_manual_setpoint_{false};
			bool manual_setpoint_published_{false};
		// 最近一次经过有限值检查和限幅的 MAVROS/PX4 控制量；里程计失效后最多保持 0.3 s。
		mavros_msgs::AttitudeTarget last_safe_setpoint_;
		bool have_last_safe_setpoint_{false};
		bool mpc_recovery_active_{false};
		bool direct_auto_land_active_{false};
		bool planning_stop_sent_{false};
		ros::Time mpc_recovery_start_time_{0};
		ros::Time last_mpc_recovery_reset_time_{0};
		int mpc_recovery_success_count_{0};
		// 最后一条有效入口 PositionCommand 持续作为 NMPC 世界系参考，直到新命令或完整轨迹接管。
		Command_Data_t latched_entry_command_;
		ros::Time last_entry_command_stamp_{0};
		// 入口点飞行忽略规划器 yaw；入口参考首次激活或 trajectory_id 变化时锁定
		// 无人机当前世界系偏航角，后续同任务高频 PositionCommand 只更新平移参考，单位 rad。
		double entry_command_yaw_{0.0};
		bool entry_command_active_{false};
		uint32_t last_reported_trajectory_id_{0};
		int last_reported_trajectory_piece_{-1};
		Eigen::Vector3d takeoff_start_pose_{Eigen::Vector3d::Zero()};
		double takeoff_target_z_{0.0};
		double takeoff_start_yaw_{0.0};
		ros::Time takeoff_start_time_{0};
		ros::Time takeoff_settle_start_{0};

		long int rmse_cnt_ = 0;
		double rmse_sum_ = 0;
		double rmse_xy_sum_ = 0;
		double drone_max_ = 0;
		double drone_max_xy_ = 0;

		Eigen::Matrix<real_t, kStateSize, 1> est_state_;
		Eigen::Matrix<real_t, kStateSize, kSamples + 1> mpc_predicted_states_;
		Eigen::Matrix<real_t, kInputSize, kSamples> mpc_predicted_inputs_;
		// 世界系外力估计与实际传入 NMPC 的补偿力，单位 N；两者分离以支持只估计模式。
		Eigen::Vector3d fq_estimated_{Eigen::Vector3d::Zero()};
		Eigen::Vector3d fq_applied_{Eigen::Vector3d::Zero()};
		Eigen::Vector3d fl_{Eigen::Vector3d::Zero()};
		bool force_observer_input_valid_{false};
		bool fq_estimate_valid_{false};
		ros::Time last_force_alignment_odom_stamp_{0};
		uint32_t last_force_alignment_odom_seq_{0};
		bool have_force_alignment_odom_{false};
		std::string force_alignment_odom_frame_id_;
		std::string force_alignment_odom_child_frame_id_;
		ros::Time last_force_alignment_imu_stamp_{0};
		uint32_t last_force_alignment_imu_seq_{0};
		bool have_force_alignment_imu_{false};
		std::string force_alignment_imu_frame_id_;
		bool was_in_air_{false};
		ros::Time airborne_since_{0};
		bool force_attitude_diagnostic_published_{false};
		bool last_force_attitude_ready_{false};

		enum class DisturbanceGateReason
		{
			DISABLED_BY_PARAM,
			MODE_BLOCKED,
			STATE_STALE,
			DISARMED,
			NOT_OFFBOARD,
			EXTENDED_STATE_STALE,
			LANDED,
			AIRBORNE_DELAY,
			SENSOR_INVALID,
			RPM_INVALID,
			ATTITUDE_UNALIGNED,
			WINDOW_NOT_FULL,
			ESTIMATOR_FAILED,
			ACTIVE
		};
		DisturbanceGateReason last_gate_reason_{DisturbanceGateReason::DISABLED_BY_PARAM};

		enum class ThrustModelGateReason
		{
			DISABLED_BY_PARAM,
			MODE_BLOCKED,
			STATE_STALE,
			DISARMED,
			NOT_OFFBOARD,
			EXTENDED_STATE_STALE,
			LANDED,
			RPM_STALE,
			RPM_INVALID,
			ACTIVE
		};
		ThrustModelGateReason last_thrust_model_gate_reason_{
			ThrustModelGateReason::DISABLED_BY_PARAM};

		void setEstimateState(const Odom_Data_t &translation_odom,
						  const Odom_Data_t &attitude_odom);
		void setForceEstimation();
		void updateForceAttitudeAlignment(const ros::Time &now);
		void resetForceAttitudeAlignment(const char *reason);
		void clearForceObserverState();
		void publishForceAttitudeAlignmentDiagnostics();
		bool getForceAttitude(const ros::Time &now, Eigen::Quaterniond &attitude) const;
		DisturbanceGateReason disturbanceCompensationGate(const ros::Time &now) const;
		void reportDisturbanceGate(DisturbanceGateReason reason);
		void clearAppliedDisturbance();
		void processLandingSearchYawHold(const ros::Time &now);
		double landingSearchYawReference(double fallback_yaw) const;
		void clearAutonomousState();
		bool odomControlStateValid(const ros::Time &now) const;
		void startOdomFailsafe(const ros::Time &now);
		void publishFailsafeHold(const ros::Time &now);
			void publish_manual_ctrl(const ros::Time &stamp);
		// PX4 退出 OFFBOARD 后清除旧轨迹和外力补偿，防止重新进入自动模式时恢复旧控制目标。
		void handleOffboardLoss();
		bool mpcControlStateValid(const ros::Time &now) const;
		void beginMpcRecovery(const ros::Time &now,
			const char *reason = "NMPC 求解失败");
		void processMpcRecovery(const ros::Time &now);
		void beginDirectAutoLand(const ros::Time &now, const char *reason);
		void processDirectAutoLand(const ros::Time &now);
		ThrustModelGateReason thrustModelGate(const ros::Time &now) const;
		void reportThrustModelGate(ThrustModelGateReason reason);

		void publishPrediction(const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
							   const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> predicted_traj,
							   ros::Time &time, double dt);

		void publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
								   const ros::Time &stamp);
		void publish_recovery_attitude_ctrl(const ros::Time &stamp);
		bool canReuseLastValidMpc(const ros::Time &now) const;

		// ---- tools ----
		void printandresetRMSE();
		void addRMSE();
		void update_hover_pose();
		void update_mode_hover_pose();
		void update_hover_with_rc();
		bool takeoffPreconditions(const ros::Time &now, const char *&reason) const;
		void startAutoTakeoff(const ros::Time &now);
		void publish_trigger(const nav_msgs::Odometry &odom_msg);
		bool request_px4_auto_land();
		void reboot_FCU();
	};

}
#endif
