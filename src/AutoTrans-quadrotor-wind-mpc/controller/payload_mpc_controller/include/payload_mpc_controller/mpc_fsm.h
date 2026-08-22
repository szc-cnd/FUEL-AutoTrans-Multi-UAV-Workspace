#ifndef __MPCFSM_H
#define __MPCFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "mpc_params.h"
#include "mpc_input.h"
#include "mpc_controller.h"
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
		// NMPC、推力模型和外力估计共用该姿态；平移状态仍来自 odom_data。
		Odom_Data_t force_attitude_odom_data;
		Imu_Data_t imu_data;
		Command_Data_t cmd_data;
		Battery_Data_t bat_data;
		Rpm_Data_t rpm_data;

		Start_Trigger_Data_t start_trigger_data;
		Cmd_Trigger_Data_t cmd_trigger_data;
		Trajectory_Data_t trajectory_data;

		ros::Publisher traj_start_trigger_pub;
		ros::Publisher ctrl_FCU_pub;
		ros::Publisher pub_force_marker_, pub_force_, pub_force_applied_;

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
			MANUAL_CTRL = 1, // 手动模式：不求解自动控制；发布低值占位 setpoint，避免残留上一帧命令。
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
			MPC_RECOVERY_HOVER = 13, // NMPC 故障期间锁存固定位置，禁止执行规划轨迹。
		};

		MPCFSM(const ros::NodeHandle &nh, MpcParams &params, MpcController &controller);

		void process();
		void CMD_CTRL_process();

		bool rc_is_received(const ros::Time &now_time) const;
		bool odom_is_received(const ros::Time &now_time) const;
		bool imu_is_received(const ros::Time &now_time) const;
		bool bat_is_received(const ros::Time &now_time) const;
		bool recv_new_odom();
		void addNewForceObseverState();
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
		OdomSpikeGuard odom_spike_guard_;
		ros::Time land_start_time_;
		bool auto_land_lockout_{false};
		bool takeoff_requested_{false};
		// CH8 低位触发一次起飞；失败后必须离开低位再重新进入，避免循环反复重启。
		bool takeoff_request_latched_{false};
		std::string last_takeoff_precondition_reason_;
		Eigen::Vector3d takeoff_start_pose_{Eigen::Vector3d::Zero()};
		double takeoff_target_z_{0.0};
		double takeoff_start_yaw_{0.0};
		ros::Time takeoff_start_time_{0};
		// 里程计失效后的 OFFBOARD 失联保护：最多继续发 0.3 s 的最后安全 setpoint。
		bool odom_failsafe_active_{false};
		ros::Time odom_failsafe_deadline_{0};
		bool suppress_manual_setpoint_{false};
		bool auto_land_request_sent_{false};
		ros::Time last_auto_land_request_time_{0};
		Eigen::Vector3d last_safe_body_rate_{Eigen::Vector3d::Zero()};
		double last_safe_normalized_thrust_{0.0};
		bool last_safe_setpoint_valid_{false};
		bool manual_setpoint_published_{false};
		bool mpc_recovery_active_{false};
		bool direct_auto_land_active_{false};
		bool planning_stop_sent_{false};
		ros::Time mpc_recovery_start_time_{0};
		ros::Time last_mpc_recovery_reset_time_{0};
		int mpc_recovery_success_count_{0};

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
		bool was_in_air_{false};
		ros::Time airborne_since_{0};

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
		void clearForceObserverState();
		DisturbanceGateReason disturbanceCompensationGate(const ros::Time &now) const;
		void reportDisturbanceGate(DisturbanceGateReason reason);
		void clearAppliedDisturbance();
		ThrustModelGateReason thrustModelGate(const ros::Time &now) const;
		void reportThrustModelGate(ThrustModelGateReason reason);

		void publishPrediction(const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
							   const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> predicted_traj,
							   ros::Time &time, double dt);

		void publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> predicted_input,
									   const ros::Time &stamp);
		void publish_recovery_attitude_ctrl(const ros::Time &stamp);
		bool canReuseLastValidMpc(const ros::Time &now) const;
		void publish_manual_ctrl(const ros::Time &stamp);
		void publish_failsafe_hold(const ros::Time &stamp);
		void enter_odom_failsafe(const char *reason);
		void clear_autonomous_inputs();
		bool odom_state_valid() const;
		void beginMpcRecovery(const ros::Time &now,
			const char *reason = "NMPC 求解失败");
		void processMpcRecovery(const ros::Time &now);
		void beginDirectAutoLand(const ros::Time &now, const char *reason);
		void processDirectAutoLand(const ros::Time &now);

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
