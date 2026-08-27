#pragma once

#include <cmath>
#include <ros/ros.h>
#include <mpc_wrapper.h>

namespace PayloadMPC
{

	class MpcParams
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		struct Q_Gain
		{
			real_t Q_pos_xy;
			real_t Q_pos_z;
			real_t Q_attitude_rp;
			real_t Q_attitude_yaw;
			real_t Q_velocity;
			real_t Q_payload_xy;
			real_t Q_payload_z;
			real_t Q_payload_velocity;
			real_t Q_cable;
			real_t Q_dcable;
		};

		struct R_Gain
		{
			real_t R_thrust;
			real_t R_pitchroll;
			real_t R_yaw;
			real_t R_velocity_slack_xy;
			real_t R_velocity_slack_z;
		};

		struct ForceEstimator
		{
			double sample_freq_fq{100.0};
			double sample_freq_fl{100.0};
			double cutoff_freq_fq{10.0};
			double cutoff_freq_fl{10.0};
			double imu_body_length;
			double kf;
			double sqrt_kf;
			// 外力估计结果的世界系三维模长上限，单位 N。
			double max_force;
			// 实际写入 NMPC OnlineData 的世界系补偿力模长上限，单位 N。
			double max_applied_force;
			// 实际补偿力的水平向量与垂直分量最大变化率，单位 N/s。
			double max_applied_force_rate_xy;
			double max_applied_force_rate_z;
			// 世界系外力补偿分轴增益；仅作用于写入 NMPC 的补偿量。
			double force_axis_gain_x{1.0};
			double force_axis_gain_y{1.0};
			double force_axis_gain_z{1.0};
			// 是否运行并发布世界系无人机外力 f_Q，单位 N；该开关本身不授权 NMPC 使用补偿。
			bool enable_force_estimation;
			// 是否把有效的 f_Q 写入 NMPC OnlineData；关闭时 NMPC 始终接收零外力。
			bool enable_disturbance_compensation;
			// PX4 确认无人机已在空中后，允许外力进入 NMPC 前的等待时间，单位 s。
			double compensation_airborne_delay;
			// 四路电机用于外力估计和补偿的最低有效机械转速，单位 rpm。
			double min_valid_rpm;
			// true：使用 MAVROS IMU 姿态并执行启动 yaw 对齐；false：使用独立的 MAVROS 融合 odom 姿态。
			bool use_px4_imu_attitude;
			// 地面静止姿态对齐所需的连续采样时间，单位 s。
			double attitude_alignment_duration;
			// 姿态对齐至少需要的独立 FAST-LIO odom 样本数。
			int attitude_alignment_min_samples;
			// 姿态对齐允许的最大机体系角速度，单位 rad/s。
			double attitude_alignment_max_body_rate;
			// 姿态对齐允许的最大 FAST-LIO 世界系速度，单位 m/s。
			double attitude_alignment_max_speed;
			// 两路姿态相对 roll/pitch 的最大绝对误差，单位 deg。
			double attitude_alignment_max_tilt_error_deg;
			// 偏航差样本的最大圆周标准差，单位 deg。
			double attitude_alignment_max_yaw_std_deg;
			int USE_CONSTANT_MOMENT;
			int max_queue;
			double force_observer_freq;
			double var_weight;
		};

		struct filter
		{
			double sample_freq_quad_acc{333.333};
			double sample_freq_quad_omg{333.333};
			double sample_freq_load_acc{333.333};
			double sample_freq_load_omg{333.333};
			double sample_freq_rpm{333.333};
			double sample_freq_cable{333.333};
			double sample_freq_dcable{333.333};

			double cutoff_freq_quad_acc{100.0};
			double cutoff_freq_quad_omg{100.0};
			double cutoff_freq_load_acc{100.0};
			double cutoff_freq_load_omg{100.0};
			double cutoff_freq_rpm{100.0};
			double cutoff_freq_cable{100.0};
			double cutoff_freq_dcable{333.333};
		};

		// For real-world experiments
		struct MsgTimeout
		{
			double odom;
			// 外力估计专用 /mavros/local_position/odom 的消息超时阈值，单位 s。
			double force_attitude_odom;
			double rc;
			double cmd;
			double imu;
			double bat;
			// /mavros/esc_status 消息超时阈值，单位 s；超时后外力估计无效并清零补偿。
			double rpm;
			// /mavros/state 与 /mavros/extended_state 的超时阈值，单位 s。
			double state;
			double extended_state;
		};

		struct ThrustMapping
		{
			bool print_val;
			int accurate_thrust_model;
			// 0：固定推力比例；1：使用四路机械转速在线辨识推力比例。普通四旋翼版本禁止模式 2。
			double hover_percentage;
			double filter_factor;
			// 模式 1 允许学习的单电机最低机械转速，单位 rpm；与外力估计器门限相互独立。
			double min_learning_rpm;
			// 发布给 MAVROS/PX4 AttitudeTarget.thrust 的最终归一化上限，不是牛顿力。
			double max_normalized_thrust;
			// 单次 RLS 更新允许的悬停归一化推力比例最大变化量。
			double max_hover_percentage_step;
		};

		struct DynmaicParams
		{
			real_t mass_q;
			real_t mass_l;
			real_t l_length;
		};

		struct RCReverse
		{
			bool roll;
			bool pitch;
			bool yaw;
			bool throttle;
		};

		struct RCMode
		{
			int mode_channel{7};
			int land_channel{9};
			// RCIn.channels 下标 5 对应 CH6；CH6 由 PX4 RC_MAP_OFFB_SW 切换 OFFBOARD。
			int low_threshold{1300};
			int mid_low_threshold{1300};
			int mid_high_threshold{1700};
			int high_threshold{1800};
		};

		struct Land
		{
			double descent_rate{0.2};
			double min_target_z{0.05};
			double switch_odom_z{0.25};
			double timeout{8.0};
		};

		struct Safety
		{
			// NMPC 首次失败后锁点恢复的告警阈值，单位 s；超时继续重试，不自动降落。
			double mpc_recovery_timeout{1.0};
			// 位置、速度和倾角同时收敛且 NMPC 连续成功该次数后，才恢复接收规划轨迹。
			int mpc_recovery_success_cycles{5};
			// 仅在姿态、速度和缓存角速度均安全时短暂复用最后有效输入，单位 s。
			double mpc_recovery_last_valid_hold{0.05};
			double mpc_recovery_last_valid_max_bodyrate{0.25};
			// 连续失败时重新初始化完整 ACADO 工作区的周期，单位 s。
			double mpc_recovery_full_reset_period{0.20};
			// 重置时临时提高速度软约束阈值，避免恢复阶段为超限速度支付过高代价，单位 m/s。
			double mpc_recovery_velocity_margin_xy{0.20};
			double mpc_recovery_velocity_margin_z{0.15};
			// 退出恢复模式前必须满足的实际状态阈值。
			double mpc_recovery_exit_speed_xy{0.15};
			double mpc_recovery_exit_speed_z{0.10};
			double mpc_recovery_exit_tilt_deg{8.0};
			double mpc_recovery_exit_position_error{0.15};
			// 拉平阶段倾斜推力补偿采用的最大倾角，单位 deg。
			double mpc_recovery_max_thrust_comp_tilt_deg{30.0};
			// 请求 PX4 AUTO.LAND 的最小重试周期，单位 s。
			double auto_land_retry_period{1.0};
		};

		struct OdomSpikeGuard
		{
			bool enabled{true};
			// 仅在相邻高频里程计间隔不超过该值时检查连续性，单位 s。
			double max_sample_interval{0.10};
			// 当前位置相对上一原始样本匀速预测值的最大残差，单位 m。
			double max_position_residual_xy{0.08};
			double max_position_residual_z{0.06};
			// 相邻原始样本的最大速度突变量，单位 m/s。
			double max_velocity_jump_xy{0.60};
			double max_velocity_jump_z{0.60};
			// 异常连续达到该时间后进入已有 MPC_RECOVERY_HOVER，单位 s。
			double fault_duration{0.10};
			// 故障锁存后必须连续收到该数量的正常样本才解除门控。
			int recovery_good_samples{5};
		};

		struct Takeoff
		{
			// AUTO_TAKEOFF 只在 PX4 已经进入 OFFBOARD 后执行，不自动解锁或切换 OFFBOARD。
			bool enabled{true};
			// 唯一 ENU 世界系起飞/悬停目标点的高度，单位 m。
			double target_z{1.0};
			// z 参考值上升速度，单位 m/s；达到 target_z 后保持目标点。
			double climb_rate{0.25};
			// 固定悬停点启用时，起飞前允许的水平距离，单位 m。
			double max_initial_xy_error{0.5};
		};

		struct FixedHover
		{
			// 固定悬停点使用 /mavros/local_position/odom 对应的 ENU 世界坐标系，单位 m。
			bool enabled{false};
			double x{0.0};
			double y{0.0};
			double z{1.0};
		};

		Q_Gain q_gain_;
		R_Gain r_gain_;
		MsgTimeout msg_timeout_;
		ThrustMapping thr_map_;
		DynmaicParams dyn_params_;
		RCReverse rc_reverse_;
		RCMode rc_mode_;
		Land land_;
		Safety safety_;
		OdomSpikeGuard odom_spike_guard_;
		Takeoff takeoff_;
		FixedHover fixed_hover_;

		ForceEstimator force_estimator_param_;
		filter filter_param_;

		real_t gravity_;
		// mpc constraint
		real_t min_thrust_;
		real_t max_thrust_;
		real_t max_bodyrate_xy_;
		real_t max_bodyrate_z_;
		// 世界系 ENU 正常速度软约束及松弛量硬上限，单位 m/s。
		real_t max_velocity_xy_;
		real_t max_velocity_z_;
		real_t max_velocity_slack_xy_;
		real_t max_velocity_slack_z_;

		real_t state_cost_exponential_;
		real_t input_cost_exponential_;

		Eigen::Matrix<real_t, kCostSize, kCostSize> Q_;
		Eigen::Matrix<real_t, kInputSize, kInputSize> R_;
		double step_T_;
		int step_N_;

		double ctrl_freq_max_;

		bool use_trajectory_ending_pos_;

		bool print_info_;

		double max_angle_;
		double max_manual_vel_;
		bool enable_rc_hover_adjust_;
		double low_voltage_;

		bool use_simulation_;
		bool use_fix_yaw_;

		MpcParams()
		{
			print_info_ = false;
			state_cost_exponential_ = 0.0;
			input_cost_exponential_ = 0.0;
			min_thrust_ = 0.0;
			max_thrust_ = 0.0;
			max_bodyrate_z_ = 0.0;
			max_bodyrate_xy_ = 0.0;
			max_velocity_xy_ = 0.0;
			max_velocity_z_ = 0.0;
			max_velocity_slack_xy_ = 0.0;
			max_velocity_slack_z_ = 0.0;
			enable_rc_hover_adjust_ = false;
		}

		~MpcParams()
		{
		}

		void config_from_ros_handle(const ros::NodeHandle &nh)
		{
			read_essential_param(nh, "Q_pos_xy", q_gain_.Q_pos_xy);
			read_essential_param(nh, "Q_pos_z", q_gain_.Q_pos_z);
			read_essential_param(nh, "Q_attitude_rp", q_gain_.Q_attitude_rp);
			read_essential_param(nh, "Q_attitude_yaw", q_gain_.Q_attitude_yaw);
			read_essential_param(nh, "Q_velocity", q_gain_.Q_velocity);
			read_essential_param(nh, "Q_payload_xy", q_gain_.Q_payload_xy);
			read_essential_param(nh, "Q_payload_z", q_gain_.Q_payload_z);
			read_essential_param(nh, "Q_payload_velocity", q_gain_.Q_payload_velocity);
			read_essential_param(nh, "Q_cable", q_gain_.Q_cable);
			read_essential_param(nh, "Q_dcable", q_gain_.Q_dcable);

			read_essential_param(nh, "R_thrust", r_gain_.R_thrust);
			read_essential_param(nh, "R_pitchroll", r_gain_.R_pitchroll);
			read_essential_param(nh, "R_yaw", r_gain_.R_yaw);
			read_essential_param(nh, "R_velocity_slack_xy", r_gain_.R_velocity_slack_xy);
			read_essential_param(nh, "R_velocity_slack_z", r_gain_.R_velocity_slack_z);
			if (!std::isfinite(r_gain_.R_velocity_slack_xy) ||
				!std::isfinite(r_gain_.R_velocity_slack_z) ||
				r_gain_.R_velocity_slack_xy <= 0.0 || r_gain_.R_velocity_slack_z <= 0.0)
			{
				ROS_ERROR("[参数] R_velocity_slack_xy/z 必须为有限正数。");
				ROS_BREAK();
			}

			read_essential_param(nh, "min_thrust", min_thrust_);
			read_essential_param(nh, "max_thrust", max_thrust_);
			read_essential_param(nh, "max_bodyrate_xy", max_bodyrate_xy_);
			read_essential_param(nh, "max_bodyrate_z", max_bodyrate_z_);
			read_essential_param(nh, "max_velocity_xy", max_velocity_xy_);
			read_essential_param(nh, "max_velocity_z", max_velocity_z_);
			read_essential_param(nh, "max_velocity_slack_xy", max_velocity_slack_xy_);
			read_essential_param(nh, "max_velocity_slack_z", max_velocity_slack_z_);
			if (!std::isfinite(max_velocity_xy_) || !std::isfinite(max_velocity_z_) ||
				!std::isfinite(max_velocity_slack_xy_) ||
				!std::isfinite(max_velocity_slack_z_) ||
				max_velocity_xy_ <= 0.0 || max_velocity_z_ <= 0.0 ||
				max_velocity_slack_xy_ <= 0.0 || max_velocity_slack_z_ <= 0.0)
			{
				ROS_ERROR("[参数] 速度上限及松弛量上限必须为有限正数，单位 m/s。");
				ROS_BREAK();
			}

			read_essential_param(nh, "state_cost_exponential", state_cost_exponential_);
			read_essential_param(nh, "input_cost_exponential", input_cost_exponential_);
			read_essential_param(nh, "step_T", step_T_);
			read_essential_param(nh, "step_N", step_N_);

			read_essential_param(nh, "use_trajectory_ending_pos", use_trajectory_ending_pos_);

			read_essential_param(nh, "mass_l", dyn_params_.mass_l);
			read_essential_param(nh, "l_length", dyn_params_.l_length);
			read_essential_param(nh, "mass_q", dyn_params_.mass_q);
			read_essential_param(nh, "gravity", gravity_);

			read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max_);

			read_essential_param(nh, "rc_reverse/roll", rc_reverse_.roll);
			read_essential_param(nh, "rc_reverse/pitch", rc_reverse_.pitch);
			read_essential_param(nh, "rc_reverse/yaw", rc_reverse_.yaw);
			read_essential_param(nh, "rc_reverse/throttle", rc_reverse_.throttle);

			read_essential_param(nh, "rc_mode/mode_channel", rc_mode_.mode_channel);
			read_essential_param(nh, "rc_mode/land_channel", rc_mode_.land_channel);
			read_essential_param(nh, "rc_mode/low_threshold", rc_mode_.low_threshold);
			read_essential_param(nh, "rc_mode/mid_low_threshold", rc_mode_.mid_low_threshold);
			read_essential_param(nh, "rc_mode/mid_high_threshold", rc_mode_.mid_high_threshold);
			read_essential_param(nh, "rc_mode/high_threshold", rc_mode_.high_threshold);

			read_essential_param(nh, "land/descent_rate", land_.descent_rate);
			read_essential_param(nh, "land/min_target_z", land_.min_target_z);
			read_essential_param(nh, "land/switch_odom_z", land_.switch_odom_z);
			read_essential_param(nh, "land/timeout", land_.timeout);

			read_essential_param(nh, "safety/mpc_recovery_timeout", safety_.mpc_recovery_timeout);
			read_essential_param(nh, "safety/mpc_recovery_success_cycles", safety_.mpc_recovery_success_cycles);
			read_essential_param(nh, "safety/mpc_recovery_last_valid_hold", safety_.mpc_recovery_last_valid_hold);
			read_essential_param(nh, "safety/mpc_recovery_last_valid_max_bodyrate", safety_.mpc_recovery_last_valid_max_bodyrate);
			read_essential_param(nh, "safety/mpc_recovery_full_reset_period", safety_.mpc_recovery_full_reset_period);
			read_essential_param(nh, "safety/mpc_recovery_velocity_margin_xy", safety_.mpc_recovery_velocity_margin_xy);
			read_essential_param(nh, "safety/mpc_recovery_velocity_margin_z", safety_.mpc_recovery_velocity_margin_z);
			read_essential_param(nh, "safety/mpc_recovery_exit_speed_xy", safety_.mpc_recovery_exit_speed_xy);
			read_essential_param(nh, "safety/mpc_recovery_exit_speed_z", safety_.mpc_recovery_exit_speed_z);
			read_essential_param(nh, "safety/mpc_recovery_exit_tilt_deg", safety_.mpc_recovery_exit_tilt_deg);
			read_essential_param(nh, "safety/mpc_recovery_exit_position_error", safety_.mpc_recovery_exit_position_error);
			read_essential_param(nh, "safety/mpc_recovery_max_thrust_comp_tilt_deg", safety_.mpc_recovery_max_thrust_comp_tilt_deg);
			read_essential_param(nh, "safety/auto_land_retry_period", safety_.auto_land_retry_period);
			if (!std::isfinite(safety_.mpc_recovery_timeout) ||
				safety_.mpc_recovery_timeout <= 0.0 ||
				safety_.mpc_recovery_success_cycles <= 0 ||
				!std::isfinite(safety_.mpc_recovery_last_valid_hold) ||
				safety_.mpc_recovery_last_valid_hold < 0.0 ||
				!std::isfinite(safety_.mpc_recovery_last_valid_max_bodyrate) ||
				safety_.mpc_recovery_last_valid_max_bodyrate <= 0.0 ||
				!std::isfinite(safety_.mpc_recovery_full_reset_period) ||
				safety_.mpc_recovery_full_reset_period <= 0.0 ||
				!std::isfinite(safety_.mpc_recovery_velocity_margin_xy) ||
				safety_.mpc_recovery_velocity_margin_xy < 0.0 ||
				!std::isfinite(safety_.mpc_recovery_velocity_margin_z) ||
				safety_.mpc_recovery_velocity_margin_z < 0.0 ||
				!std::isfinite(safety_.mpc_recovery_exit_speed_xy) ||
				safety_.mpc_recovery_exit_speed_xy <= 0.0 ||
				!std::isfinite(safety_.mpc_recovery_exit_speed_z) ||
				safety_.mpc_recovery_exit_speed_z <= 0.0 ||
				!std::isfinite(safety_.mpc_recovery_exit_tilt_deg) ||
				safety_.mpc_recovery_exit_tilt_deg <= 0.0 ||
				safety_.mpc_recovery_exit_tilt_deg >= 90.0 ||
				!std::isfinite(safety_.mpc_recovery_exit_position_error) ||
				safety_.mpc_recovery_exit_position_error <= 0.0 ||
				!std::isfinite(safety_.mpc_recovery_max_thrust_comp_tilt_deg) ||
				safety_.mpc_recovery_max_thrust_comp_tilt_deg <= 0.0 ||
				safety_.mpc_recovery_max_thrust_comp_tilt_deg >= 90.0 ||
				!std::isfinite(safety_.auto_land_retry_period) ||
				safety_.auto_land_retry_period <= 0.0)
			{
				ROS_ERROR("[参数] safety 恢复参数无效：时间、误差和速度阈值必须为有限正值，倾角必须小于 90 deg。");
				ROS_BREAK();
			}

			read_essential_param(nh, "odom_spike_guard/enabled", odom_spike_guard_.enabled);
			read_essential_param(nh, "odom_spike_guard/max_sample_interval", odom_spike_guard_.max_sample_interval);
			read_essential_param(nh, "odom_spike_guard/max_position_residual_xy", odom_spike_guard_.max_position_residual_xy);
			read_essential_param(nh, "odom_spike_guard/max_position_residual_z", odom_spike_guard_.max_position_residual_z);
			read_essential_param(nh, "odom_spike_guard/max_velocity_jump_xy", odom_spike_guard_.max_velocity_jump_xy);
			read_essential_param(nh, "odom_spike_guard/max_velocity_jump_z", odom_spike_guard_.max_velocity_jump_z);
			read_essential_param(nh, "odom_spike_guard/fault_duration", odom_spike_guard_.fault_duration);
			read_essential_param(nh, "odom_spike_guard/recovery_good_samples", odom_spike_guard_.recovery_good_samples);
			if (!std::isfinite(odom_spike_guard_.max_sample_interval) ||
				!std::isfinite(odom_spike_guard_.max_position_residual_xy) ||
				!std::isfinite(odom_spike_guard_.max_position_residual_z) ||
				!std::isfinite(odom_spike_guard_.max_velocity_jump_xy) ||
				!std::isfinite(odom_spike_guard_.max_velocity_jump_z) ||
				!std::isfinite(odom_spike_guard_.fault_duration) ||
				odom_spike_guard_.max_sample_interval <= 0.0 ||
				odom_spike_guard_.max_position_residual_xy <= 0.0 ||
				odom_spike_guard_.max_position_residual_z <= 0.0 ||
				odom_spike_guard_.max_velocity_jump_xy <= 0.0 ||
				odom_spike_guard_.max_velocity_jump_z <= 0.0 ||
				odom_spike_guard_.fault_duration <= 0.0 ||
				odom_spike_guard_.recovery_good_samples <= 0)
			{
				ROS_ERROR("[参数] odom_spike_guard 阈值、故障窗口和恢复样本数必须为有限正值。");
				ROS_BREAK();
			}

			read_essential_param(nh, "takeoff/enabled", takeoff_.enabled);
			read_essential_param(nh, "takeoff/target_z", takeoff_.target_z);
			read_essential_param(nh, "takeoff/climb_rate", takeoff_.climb_rate);
			read_essential_param(nh, "takeoff/max_initial_xy_error", takeoff_.max_initial_xy_error);
			if (!std::isfinite(takeoff_.target_z) || takeoff_.target_z < 0.0 ||
				!std::isfinite(takeoff_.climb_rate) || takeoff_.climb_rate <= 0.0 ||
				!std::isfinite(takeoff_.max_initial_xy_error) || takeoff_.max_initial_xy_error < 0.0)
			{
				ROS_ERROR("[参数] takeoff 参数无效。");
				ROS_BREAK();
			}
			read_essential_param(nh, "fixed_hover/enabled", fixed_hover_.enabled);
			read_essential_param(nh, "fixed_hover/x", fixed_hover_.x);
			read_essential_param(nh, "fixed_hover/y", fixed_hover_.y);
			read_essential_param(nh, "fixed_hover/z", fixed_hover_.z);
			if (!std::isfinite(fixed_hover_.x) || !std::isfinite(fixed_hover_.y) ||
				!std::isfinite(fixed_hover_.z) || fixed_hover_.z < 0.0)
			{
				ROS_ERROR("[参数] fixed_hover 位置必须为有限值，z 必须非负。");
				ROS_BREAK();
			}

			read_essential_param(nh, "thrust_model/print_value", thr_map_.print_val);
			read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map_.accurate_thrust_model);
			read_essential_param(nh, "thrust_model/hover_percentage", thr_map_.hover_percentage);
			read_essential_param(nh, "thrust_model/filter_factor", thr_map_.filter_factor);
			read_essential_param(nh, "thrust_model/min_learning_rpm", thr_map_.min_learning_rpm);
			read_essential_param(nh, "thrust_model/max_normalized_thrust", thr_map_.max_normalized_thrust);
			read_essential_param(nh, "thrust_model/max_hover_percentage_step", thr_map_.max_hover_percentage_step);
			if (thr_map_.accurate_thrust_model < 0 || thr_map_.accurate_thrust_model > 1)
			{
				ROS_ERROR("[参数] thrust_model/accurate_thrust_model 只能为 0 或 1；普通四旋翼不支持模式 2。");
				ROS_BREAK();
			}
			if (!std::isfinite(thr_map_.hover_percentage) ||
				thr_map_.hover_percentage < 0.1 || thr_map_.hover_percentage > 0.8)
			{
				ROS_ERROR("[参数] thrust_model/hover_percentage 必须为有限值且位于 [0.1, 0.8]。");
				ROS_BREAK();
			}
			if (!std::isfinite(thr_map_.filter_factor) ||
				thr_map_.filter_factor <= 0.0 || thr_map_.filter_factor > 1.0)
			{
				ROS_ERROR("[参数] thrust_model/filter_factor 必须为有限值且位于 (0, 1]。");
				ROS_BREAK();
			}
			if (!std::isfinite(thr_map_.min_learning_rpm) || thr_map_.min_learning_rpm <= 0.0)
			{
				ROS_ERROR("[参数] thrust_model/min_learning_rpm 必须为有限正数。");
				ROS_BREAK();
			}
			if (!std::isfinite(thr_map_.max_normalized_thrust) ||
				thr_map_.max_normalized_thrust <= 0.0 || thr_map_.max_normalized_thrust > 1.0)
			{
				ROS_ERROR("[参数] thrust_model/max_normalized_thrust 必须为有限值且位于 (0, 1]。");
				ROS_BREAK();
			}
			if (!std::isfinite(thr_map_.max_hover_percentage_step) ||
				thr_map_.max_hover_percentage_step <= 0.0 ||
				thr_map_.max_hover_percentage_step >= 0.1)
			{
				ROS_ERROR("[参数] thrust_model/max_hover_percentage_step 必须为有限值且位于 (0, 0.1)。");
				ROS_BREAK();
			}

			read_essential_param(nh, "filter/sample_freq_quad_acc", filter_param_.sample_freq_quad_acc);
			read_essential_param(nh, "filter/sample_freq_quad_omg", filter_param_.sample_freq_quad_omg);
			read_essential_param(nh, "filter/sample_freq_load_acc", filter_param_.sample_freq_load_acc);
			read_essential_param(nh, "filter/sample_freq_load_omg", filter_param_.sample_freq_load_omg);
			read_essential_param(nh, "filter/sample_freq_rpm", filter_param_.sample_freq_rpm);
			read_essential_param(nh, "filter/cutoff_freq_quad_acc", filter_param_.cutoff_freq_quad_acc);
			read_essential_param(nh, "filter/cutoff_freq_quad_omg", filter_param_.cutoff_freq_quad_omg);
			read_essential_param(nh, "filter/cutoff_freq_load_acc", filter_param_.cutoff_freq_load_acc);
			read_essential_param(nh, "filter/cutoff_freq_load_omg", filter_param_.cutoff_freq_load_omg);
			read_essential_param(nh, "filter/cutoff_freq_rpm", filter_param_.cutoff_freq_rpm);
			read_essential_param(nh, "filter/sample_freq_cable", filter_param_.sample_freq_cable);
			read_essential_param(nh, "filter/cutoff_freq_cable", filter_param_.cutoff_freq_cable);
			read_essential_param(nh, "filter/sample_freq_dcable", filter_param_.sample_freq_dcable);
			read_essential_param(nh, "filter/cutoff_freq_dcable", filter_param_.cutoff_freq_dcable);

			read_essential_param(nh, "force_estimator/kf", force_estimator_param_.kf);
			if (!std::isfinite(force_estimator_param_.kf) || force_estimator_param_.kf <= 0.0)
			{
				ROS_ERROR("[参数] force_estimator/kf 必须为有限正数。");
				ROS_BREAK();
			}
			force_estimator_param_.sqrt_kf = sqrt(force_estimator_param_.kf);
			read_essential_param(nh, "force_estimator/sample_freq_fq", force_estimator_param_.sample_freq_fq);
			read_essential_param(nh, "force_estimator/sample_freq_fl", force_estimator_param_.sample_freq_fl);
			read_essential_param(nh, "force_estimator/cutoff_freq_fq", force_estimator_param_.cutoff_freq_fq);
			read_essential_param(nh, "force_estimator/cutoff_freq_fl", force_estimator_param_.cutoff_freq_fl);
			read_essential_param(nh, "force_estimator/imu_body_length", force_estimator_param_.imu_body_length);
			read_essential_param(nh, "force_estimator/enable_force_estimation", force_estimator_param_.enable_force_estimation);
			read_essential_param(nh, "force_estimator/enable_disturbance_compensation", force_estimator_param_.enable_disturbance_compensation);
			read_essential_param(nh, "force_estimator/compensation_airborne_delay", force_estimator_param_.compensation_airborne_delay);
			read_essential_param(nh, "force_estimator/min_valid_rpm", force_estimator_param_.min_valid_rpm);
			read_essential_param(nh, "force_estimator/use_px4_imu_attitude", force_estimator_param_.use_px4_imu_attitude);
			read_essential_param(nh, "force_estimator/attitude_alignment_duration", force_estimator_param_.attitude_alignment_duration);
			read_essential_param(nh, "force_estimator/attitude_alignment_min_samples", force_estimator_param_.attitude_alignment_min_samples);
			read_essential_param(nh, "force_estimator/attitude_alignment_max_body_rate", force_estimator_param_.attitude_alignment_max_body_rate);
			read_essential_param(nh, "force_estimator/attitude_alignment_max_speed", force_estimator_param_.attitude_alignment_max_speed);
			read_essential_param(nh, "force_estimator/attitude_alignment_max_tilt_error_deg", force_estimator_param_.attitude_alignment_max_tilt_error_deg);
			read_essential_param(nh, "force_estimator/attitude_alignment_max_yaw_std_deg", force_estimator_param_.attitude_alignment_max_yaw_std_deg);
			if (!std::isfinite(force_estimator_param_.attitude_alignment_duration) ||
				!std::isfinite(force_estimator_param_.attitude_alignment_max_body_rate) ||
				!std::isfinite(force_estimator_param_.attitude_alignment_max_speed) ||
				!std::isfinite(force_estimator_param_.attitude_alignment_max_tilt_error_deg) ||
				!std::isfinite(force_estimator_param_.attitude_alignment_max_yaw_std_deg) ||
				force_estimator_param_.attitude_alignment_duration <= 0.0 ||
				force_estimator_param_.attitude_alignment_min_samples < 2 ||
				force_estimator_param_.attitude_alignment_max_body_rate <= 0.0 ||
				force_estimator_param_.attitude_alignment_max_speed <= 0.0 ||
				force_estimator_param_.attitude_alignment_max_tilt_error_deg <= 0.0 ||
				force_estimator_param_.attitude_alignment_max_tilt_error_deg >= 45.0 ||
				force_estimator_param_.attitude_alignment_max_yaw_std_deg <= 0.0 ||
				force_estimator_param_.attitude_alignment_max_yaw_std_deg >= 45.0)
			{
				ROS_ERROR("[参数] 外力估计姿态对齐参数无效。");
				ROS_BREAK();
			}
			if (force_estimator_param_.enable_disturbance_compensation &&
				!force_estimator_param_.enable_force_estimation)
			{
				ROS_ERROR("[参数] 外力补偿依赖外力估计，已自动关闭补偿。");
				force_estimator_param_.enable_disturbance_compensation = false;
			}
			read_essential_param(nh, "force_estimator/USE_CONSTANT_MOMENT", force_estimator_param_.USE_CONSTANT_MOMENT);
			read_essential_param(nh, "force_estimator/max_force", force_estimator_param_.max_force);
			read_essential_param(nh, "force_estimator/max_applied_force", force_estimator_param_.max_applied_force);
			read_essential_param(nh, "force_estimator/max_applied_force_rate_xy", force_estimator_param_.max_applied_force_rate_xy);
			read_essential_param(nh, "force_estimator/max_applied_force_rate_z", force_estimator_param_.max_applied_force_rate_z);
			read_essential_param(nh, "force_estimator/force_axis_gain_x", force_estimator_param_.force_axis_gain_x);
			read_essential_param(nh, "force_estimator/force_axis_gain_y", force_estimator_param_.force_axis_gain_y);
			read_essential_param(nh, "force_estimator/force_axis_gain_z", force_estimator_param_.force_axis_gain_z);
			if (!std::isfinite(force_estimator_param_.max_force) ||
				!std::isfinite(force_estimator_param_.max_applied_force) ||
				force_estimator_param_.max_force <= 0.0 ||
				force_estimator_param_.max_applied_force <= 0.0 ||
				force_estimator_param_.max_applied_force > force_estimator_param_.max_force)
			{
				ROS_ERROR("[参数] force_estimator 限幅必须满足 0 < max_applied_force <= max_force。");
				ROS_BREAK();
			}
			if (!std::isfinite(force_estimator_param_.max_applied_force_rate_xy) ||
				!std::isfinite(force_estimator_param_.max_applied_force_rate_z) ||
				force_estimator_param_.max_applied_force_rate_xy <= 0.0 ||
				force_estimator_param_.max_applied_force_rate_z <= 0.0)
			{
				ROS_ERROR("[参数] 外力补偿变化率上限必须为有限正数。");
				ROS_BREAK();
			}
			if (!std::isfinite(force_estimator_param_.force_axis_gain_x) ||
				!std::isfinite(force_estimator_param_.force_axis_gain_y) ||
				!std::isfinite(force_estimator_param_.force_axis_gain_z) ||
				force_estimator_param_.force_axis_gain_x < 0.0 || force_estimator_param_.force_axis_gain_x > 1.0 ||
				force_estimator_param_.force_axis_gain_y < 0.0 || force_estimator_param_.force_axis_gain_y > 1.0 ||
				force_estimator_param_.force_axis_gain_z < 0.0 || force_estimator_param_.force_axis_gain_z > 1.0)
			{
				ROS_ERROR("[参数] force_axis_gain_x/y/z 必须为 [0, 1] 范围内的有限数值。");
				ROS_BREAK();
			}
			read_essential_param(nh, "force_estimator/max_queue", force_estimator_param_.max_queue);
			read_essential_param(nh, "force_estimator/force_observer_freq", force_estimator_param_.force_observer_freq);
			read_essential_param(nh, "force_estimator/var_weight", force_estimator_param_.var_weight);

			read_essential_param(nh, "use_simulation", use_simulation_);
			read_essential_param(nh, "use_fix_yaw", use_fix_yaw_);
			read_essential_param(nh, "print_info", print_info_);

			// revised by wyz
			read_essential_param(nh, "max_manual_vel", max_manual_vel_);
			read_essential_param(nh, "enable_rc_hover_adjust", enable_rc_hover_adjust_);
			if (fixed_hover_.enabled && enable_rc_hover_adjust_)
			{
				ROS_WARN("[参数] fixed_hover 已启用，RC 悬停参考调节将被忽略。");
			}
			read_essential_param(nh, "max_angle", max_angle_);
			read_essential_param(nh, "low_voltage", low_voltage_);

			read_essential_param(nh, "msg_timeout/odom", msg_timeout_.odom);
			read_essential_param(nh, "msg_timeout/force_attitude_odom", msg_timeout_.force_attitude_odom);
			read_essential_param(nh, "msg_timeout/rc", msg_timeout_.rc);
			read_essential_param(nh, "msg_timeout/cmd", msg_timeout_.cmd);
			read_essential_param(nh, "msg_timeout/imu", msg_timeout_.imu);
			read_essential_param(nh, "msg_timeout/bat", msg_timeout_.bat);
			read_essential_param(nh, "msg_timeout/rpm", msg_timeout_.rpm);
			read_essential_param(nh, "msg_timeout/state", msg_timeout_.state);
			read_essential_param(nh, "msg_timeout/extended_state", msg_timeout_.extended_state);
			if (!std::isfinite(msg_timeout_.odom) || !std::isfinite(msg_timeout_.force_attitude_odom) ||
				!std::isfinite(msg_timeout_.rc) ||
				!std::isfinite(msg_timeout_.cmd) || !std::isfinite(msg_timeout_.imu) ||
				!std::isfinite(msg_timeout_.bat) || !std::isfinite(msg_timeout_.rpm) ||
				!std::isfinite(msg_timeout_.state) || !std::isfinite(msg_timeout_.extended_state) ||
				msg_timeout_.odom <= 0.0 || msg_timeout_.force_attitude_odom <= 0.0 ||
				msg_timeout_.rc <= 0.0 ||
				msg_timeout_.cmd <= 0.0 || msg_timeout_.imu <= 0.0 ||
				msg_timeout_.bat <= 0.0 || msg_timeout_.rpm <= 0.0 ||
				msg_timeout_.state <= 0.0 || msg_timeout_.extended_state <= 0.0)
			{
				ROS_ERROR("[参数] 所有 msg_timeout 必须为有限正数。");
				ROS_BREAK();
			}

			max_angle_ /= (180.0 / M_PI);

			if (thr_map_.print_val)
			{
				ROS_WARN("[参数] 日常飞行建议关闭 print_value，避免日志过多。");
			}
			if (rc_reverse_.roll || rc_reverse_.pitch || rc_reverse_.yaw || rc_reverse_.throttle)
			{
				ROS_WARN("[参数] RC 反向已启用，请确认遥控器方向。");
			}
			if (use_simulation_)
			{
				ROS_WARN("[参数] 当前使用仿真模式，实机不可沿用该设置。");
			}

			std::cout << "param ended!" << std::endl;
		};
		// void config_full_thrust(double hov);

	private:
		template <typename TName, typename TVal>
		void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
		{
			if (nh.getParam(name, val))
			{
				// pass
			}
			else
			{
				ROS_ERROR_STREAM("[参数] 读取参数失败：" << name << "。");
				ROS_BREAK();
			}
		};
	};
}
