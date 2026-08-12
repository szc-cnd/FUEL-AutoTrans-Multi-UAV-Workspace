#include "mpc_wrapper.h"
#include "mpc_fsm.h"
#include "mpc_controller.h"
#include <clocale>
#include <cmath>
#include <mavros_msgs/ParamGet.h>
#include <ros/ros.h>

namespace
{
bool readPx4Parameter(ros::ServiceClient &client, const std::string &name,
                      mavros_msgs::ParamValue &value)
{
    mavros_msgs::ParamGet request;
    request.request.param_id = name;
    if (!client.call(request) || !request.response.success)
    {
        ROS_FATAL("[启动] 无法读取 PX4 参数 %s。", name.c_str());
        return false;
    }
    value = request.response.value;
    return true;
}

bool validatePx4OffboardFailsafe(ros::NodeHandle &nh)
{
    ros::ServiceClient client = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    if (!client.waitForExistence(ros::Duration(5.0)))
    {
        ROS_FATAL("[启动] /mavros/param/get 服务不可用，禁止开始 AutoTrans 自动控制。");
        return false;
    }

    mavros_msgs::ParamValue action;
    mavros_msgs::ParamValue timeout;
    if (!readPx4Parameter(client, "COM_OBL_RC_ACT", action) ||
        !readPx4Parameter(client, "COM_OF_LOSS_T", timeout))
        return false;

    const bool action_ok = action.integer == 4;
    const double timeout_value = std::abs(timeout.real) > 1.0e-9
        ? timeout.real : static_cast<double>(timeout.integer);
    const bool timeout_ok = std::abs(timeout_value - 0.3) <= 0.02;
    if (!action_ok || !timeout_ok)
    {
        ROS_FATAL("[启动] PX4 OFFBOARD 失联保护参数不符合要求：COM_OBL_RC_ACT=%lld（要求 4=Land），COM_OF_LOSS_T=%.3f s（要求 0.3 s）。",
                  static_cast<long long>(action.integer), timeout_value);
        return false;
    }
    ROS_INFO("[启动] PX4 OFFBOARD 失联保护已确认：COM_OBL_RC_ACT=4，COM_OF_LOSS_T=0.3 s。");
    return true;
}
}

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
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    // PX4 EKF 融合里程计只向外力估计器提供机体系到 ENU 世界系的姿态四元数；
    // NMPC 状态仍由上面的 FAST-LIO odom_sub 提供。
    ros::Subscriber force_attitude_odom_sub =
        nh.subscribe<nav_msgs::Odometry>("force_attitude_odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.force_attitude_odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

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

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("drone_imu/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
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
                                             boost::bind(&Rpm_Data_t::feed, &fsm.rpm_data, _1),
                                             ros::VoidConstPtr(),
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
        if (!ros::ok() || !validatePx4OffboardFailsafe(nh))
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
