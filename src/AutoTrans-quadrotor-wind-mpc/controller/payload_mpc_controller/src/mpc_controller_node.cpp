#include "mpc_wrapper.h"
#include "mpc_fsm.h"
#include "mpc_controller.h"
#include <clocale>
#include <ros/ros.h>

std::unique_ptr<PayloadMPC::MPCFSM> fsm_ptr;
void MPC_controller_main(const ros::TimerEvent &)
{
    fsm_ptr->process();
}

void system_state_update_main(const ros::TimerEvent &)
{
    fsm_ptr->addNewForceObseverState();
}

int main(int argc, char **argv)
{
    // 让 ROS1 log4cxx 按系统 UTF-8 locale 处理中文日志，避免非 ASCII 文本被替换为问号。
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "MPCctrl");
    ros::NodeHandle nh("~");

    PayloadMPC::MpcParams param;
    param.config_from_ros_handle(nh);

    PayloadMPC::MpcController controller(param);
    fsm_ptr.reset(new PayloadMPC::MPCFSM(nh, param, controller));
    PayloadMPC::MPCFSM &fsm = *fsm_ptr;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                                                  10,
                                                                                  boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
										 &PayloadMPC::MPCFSM::odomCallback,
										 &fsm,
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber force_attitude_odom_sub;
    if (!param.force_estimator_param_.force_attitude_from_odom)
    {
        force_attitude_odom_sub =
            nh.subscribe<nav_msgs::Odometry>("force_attitude_odom",
                                             100,
                                             &PayloadMPC::MPCFSM::forceAttitudeOdomCallback,
                                             &fsm,
                                             ros::TransportHints().tcpNoDelay());
    }

    ros::Subscriber cmd_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("cmd_trigger",
                                                 10,
                                                 boost::bind(&Cmd_Trigger_Data_t::feed, &fsm.cmd_trigger_data, _1));

    ros::Subscriber mpc_traj_sub =
        nh.subscribe<quadrotor_msgs::PolynomialTraj>("traj",
                                                     100,
                                                     boost::bind(&Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                     ros::VoidConstPtr(),
                                                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    // 前视 ArUco 扫描期间直接由 AutoTrans 锁定当前位置并跟踪搜索航向；
    // 不再依赖只接入旧简单控制器的 yaw-only 通路。
    ros::Subscriber landing_search_state_sub =
        nh.subscribe<std_msgs::String>("landing_search_state", 10,
                                       &PayloadMPC::MPCFSM::landingSearchStateCallback, &fsm);
    ros::Subscriber landing_search_yaw_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("landing_search_yaw", 20,
                                                      &PayloadMPC::MPCFSM::landingSearchYawCallback,
                                                      &fsm,
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("drone_imu/data",
                                       100,
                                       &PayloadMPC::MPCFSM::imuCallback,
                                       &fsm,
                                       ros::TransportHints().tcpNoDelay());
    fsm.imu_data.set_filter_params(param.filter_param_.sample_freq_quad_acc, param.filter_param_.cutoff_freq_quad_acc,
                                   param.filter_param_.sample_freq_quad_omg, param.filter_param_.cutoff_freq_quad_omg);

    ros::Subscriber rc_sub;
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    ros::Subscriber esc_sub =
        nh.subscribe<mavros_msgs::ESCStatus>("/mavros/esc_status",
                                             100,
                                             &PayloadMPC::MPCFSM::rpmCallback,
                                             &fsm,
                                             ros::TransportHints().tcpNoDelay());
    fsm.rpm_data.set_filter_params(param.filter_param_.sample_freq_rpm, param.filter_param_.cutoff_freq_rpm);

    fsm.planning_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning_stop_trigger", 10);
    fsm.planning_restart_pub_ = nh.advertise<std_msgs::Empty>("/planning_restart_trigger", 10);
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    fsm.des_yaw_pub = nh.advertise<nav_msgs::Odometry>("/des_yaw_pub", 10);

    // fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();
    if (!param.use_simulation_)
    {
        ROS_INFO("[启动] 等待遥控器数据。");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[启动] 已收到遥控器数据。");
                break;
            }
            ros::Duration(0.1).sleep();
        }
        int trials = 0;
        while (ros::ok() && !fsm.state_data.current_state.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("[启动] 无法连接 PX4。");
        }
        if (!ros::ok())
            return 1;
    }
    else
    {
        ROS_WARN("[启动] 遥控器输入已禁用，请谨慎操作。");
    }

    // Create a ROS timer for force observer
    ros::Timer force_state_timer = nh.createTimer(ros::Duration(1.0 / param.force_estimator_param_.force_observer_freq), system_state_update_main);
    // Create a ROS timer for controller
    ros::Timer MPC_controller_main_timer = nh.createTimer(ros::Duration(1.0 / param.ctrl_freq_max_), MPC_controller_main);
    // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.

    ros::spin();

    return 0;
}
