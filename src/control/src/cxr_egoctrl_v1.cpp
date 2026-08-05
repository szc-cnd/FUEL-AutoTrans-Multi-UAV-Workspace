/*****************************************************************************************
 * 自定义控制器跟踪规划轨迹（悬停交接门控 + 加速度前馈 + 自适应航向优化版）
 * * 逻辑说明：
 * 1. 启动控制器 → 持续发布安全起飞 setpoint，允许 PX4 切入 OFFBOARD；
 * 2. OFFBOARD 后保持起始水平位置并爬升到设定高度；
 * 3. 本机 FAST-LIO 里程计高度首次超过门限后，控制器才接收规划轨迹；
 * 4. 只有收到规划器轨迹 /planning/pos_cmd 后，才进入轨迹跟踪；
 * 5. 路径跟踪阶段：
 * - 使用位置+速度+加速度前馈控制，解决滞后和超调；
 * - 【新增】自适应航向：速度 > 0.2m/s 时机头自动对准飞行方向。
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <cmath>

// 100000000111 (二进制) -> 第11位是1(忽略角速度)，第10位是0(启用Yaw角度)
#define VELOCITY2D_CONTROL 0b100000000111

bool allow_yaw = true;

namespace {
double wrapAngle(double ang)
{
    while (ang > M_PI) ang -= 2.0 * M_PI;
    while (ang < -M_PI) ang += 2.0 * M_PI;
    return ang;
}
}

class Ctrl
{
public:
    Ctrl();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void position_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void mavros_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void safety_hold_cb(const std_msgs::Bool::ConstPtr& msg);
    void landing_request_cb(const std_msgs::Bool::ConstPtr& msg);
    void control(const ros::TimerEvent&);

    ros::NodeHandle nh;
    ros::NodeHandle pnh{"~"};
    visualization_msgs::Marker trackpoint;
    quadrotor_msgs::PositionCommand ego;
    tf::StampedTransform ts;
    tf::TransformBroadcaster tfBroadcasterPointer;

    unsigned short velocity_mask = VELOCITY2D_CONTROL;
    mavros_msgs::PositionTarget current_goal;
    mavros_msgs::RCIn rc;
    nav_msgs::Odometry position_msg;
    geometry_msgs::PoseStamped target_pos;
    mavros_msgs::State current_state;

    // 位置与姿态变量
    double position_x, position_y, position_z;
    double now_x, now_y, now_yaw, current_yaw;
    double targetpos_x, targetpos_y;

    // EGO 轨迹信息
    double ego_pos_x, ego_pos_y, ego_pos_z;
    double ego_vel_x, ego_vel_y, ego_vel_z;
    double ego_a_x, ego_a_y, ego_a_z;
    double ego_yaw, ego_yaw_rate;

    // 状态控制变量
    bool receive, get_now_pos;
    bool have_odom;
    bool planner_cmd_enabled;
    // 2026-07-13: 规划安全悬停与终点降落拥有高于普通轨迹的控制优先级。
    bool safety_hold_active;
    // 2026-07-27: safety hold 锁存 PX4 本地位置，避免仅发零速度时在长时间规划失败期间持续漂移。
    bool safety_hold_position_latched;
    bool have_mavros_pose;
    // 2026-07-28: 保存 MAVROS 与规划里程计的初始平移关系，运行中检查两者是否灾难性分裂。
    bool mavros_odom_alignment_ready;
    bool mavros_pose_consistent;
    double mavros_odom_offset_x, mavros_odom_offset_y, mavros_odom_offset_z;
    double mavros_odom_max_horizontal_error, mavros_odom_max_vertical_error;
    double mavros_position_x, mavros_position_y, mavros_position_z, mavros_yaw;
    double safety_hold_x, safety_hold_y, safety_hold_z, safety_hold_yaw;
    bool safety_hold_uses_mavros_frame;
    bool landing_requested;
    std::string odom_topic;
    // 2026-07-13: 控制输入、任务门控和 MAVROS 输出按车辆参数隔离，默认实例为 iris_0。
    std::string vehicle_ns, position_cmd_topic, safety_hold_topic;
    std::string landing_request_topic, goal_topic, marker_topic, world_frame, drone_frame;
    double traj_cmd_timeout; // 规划轨迹超时保护
    double planner_enable_height; // FAST-LIO z 高度接管门限
    double offboard_takeoff_height; // 未收到规划轨迹时的 OFFBOARD 起飞/悬停高度
    ros::Time last_traj_cmd_time;
    ros::Time last_tf_stamp;
    double max_reverse_speed;
    double vel_slew_rate_xy;
    double vel_slew_rate_z;
    double max_cmd_speed_xy;
    double max_cmd_speed_z;
    double max_cmd_acc_xy;
    double max_cmd_acc_z;
    double last_cmd_vx, last_cmd_vy, last_cmd_vz;
    ros::Time last_control_stamp;

    ros::Subscriber state_sub, twist_sub, target_sub, position_sub, mavros_pose_sub;
    ros::Subscriber safety_hold_sub, landing_request_sub;
    ros::Publisher local_pos_pub, pubMarker;
    ros::ServiceClient set_mode_client;
    ros::Timer timer;
    ros::Time last_land_mode_request;
};

// ===============================================
// 构造函数
// ===============================================
Ctrl::Ctrl()
{
    // 2026-07-27: 控制器默认 ROS/MAVROS 前缀由 iris_0 统一改为 UAV0；UAV1 继续通过 vehicle_ns 复用同一二进制。
    pnh.param<std::string>("vehicle_ns", vehicle_ns, std::string("/UAV0"));
    if (vehicle_ns.empty()) vehicle_ns = "/UAV0";
    if (vehicle_ns.front() != '/') vehicle_ns.insert(vehicle_ns.begin(), '/');
    pnh.param<std::string>("odom_topic", odom_topic, vehicle_ns + "/fast_lio/Odometry");
    pnh.param<std::string>("position_cmd_topic", position_cmd_topic, vehicle_ns + "/planning/pos_cmd");
    pnh.param<std::string>("safety_hold_topic", safety_hold_topic, vehicle_ns + "/planning/safety_hold");
    pnh.param<std::string>("landing_request_topic", landing_request_topic,
                           vehicle_ns + "/mission/landing_request");
    pnh.param<std::string>("goal_topic", goal_topic, vehicle_ns + "/move_base_simple/goal");
    pnh.param<std::string>("marker_topic", marker_topic, vehicle_ns + "/track_drone_point");
    pnh.param<std::string>("world_frame", world_frame, std::string("world"));
    pnh.param<std::string>("drone_frame", drone_frame, vehicle_ns.substr(1) + "/drone_frame");
    timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this); // 50Hz 控制周期
    state_sub = nh.subscribe(vehicle_ns + "/mavros/state", 10, &Ctrl::state_cb, this);
    position_sub = nh.subscribe(odom_topic, 10, &Ctrl::position_cb, this);//雷达/真值对齐里程计
    // 2026-07-27: 额外监听 PX4 本地位姿，只用于 safety hold 的同坐标系位置锁定，不替换规划跟踪用 FAST-LIO 里程计。
    mavros_pose_sub = nh.subscribe(vehicle_ns + "/mavros/local_position/pose", 20,
                                   &Ctrl::mavros_pose_cb, this);
    // 2026-07-27: 真值调试示例同步使用 UAV0 前缀。
    // position_sub = nh.subscribe("/UAV0/mavros/local_position/odom", 10, &Ctrl::position_cb, this); //真值
    target_sub = nh.subscribe(goal_topic, 10, &Ctrl::target_cb, this);
    twist_sub = nh.subscribe(position_cmd_topic, 10, &Ctrl::twist_cb, this);
    // 2026-07-13: 规划失败立即刹停；终点二维码确认后由独立请求切换 PX4 AUTO.LAND。
    safety_hold_sub = nh.subscribe(safety_hold_topic, 5, &Ctrl::safety_hold_cb, this);
    landing_request_sub =
        nh.subscribe(landing_request_topic, 2, &Ctrl::landing_request_cb, this);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(vehicle_ns + "/mavros/set_mode");

    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(vehicle_ns + "/mavros/setpoint_raw/local", 10);
    pubMarker = nh.advertise<visualization_msgs::Marker>(marker_topic, 5);

    get_now_pos = false;
    receive = false;
    planner_cmd_enabled = false;
    safety_hold_active = false;
    safety_hold_position_latched = false;
    have_mavros_pose = false;
    mavros_odom_alignment_ready = false;
    mavros_pose_consistent = false;
    mavros_odom_offset_x = mavros_odom_offset_y = mavros_odom_offset_z = 0.0;
    // 2026-07-28: 超过该差值时 MAVROS 锁点不可信；默认覆盖正常 EKF/SLAM 小幅偏差但拒绝数十米跳变。
    pnh.param("mavros_odom_max_horizontal_error", mavros_odom_max_horizontal_error, 0.80);
    pnh.param("mavros_odom_max_vertical_error", mavros_odom_max_vertical_error, 0.50);
    mavros_position_x = mavros_position_y = mavros_position_z = mavros_yaw = 0.0;
    safety_hold_x = safety_hold_y = safety_hold_z = safety_hold_yaw = 0.0;
    safety_hold_uses_mavros_frame = false;
    landing_requested = false;
    have_odom = false;
    traj_cmd_timeout = 0.6;
    pnh.param("planner_enable_height", planner_enable_height, 0.5);
    pnh.param("offboard_takeoff_height", offboard_takeoff_height, 0.6);
    // 2026-07-08: 为了避免窄通道里重规划后突然大幅后退导致炸机，增加反向速度上限与速度斜率限制参数。
    pnh.param("max_reverse_speed", max_reverse_speed, 0.25);
    // 2026-07-27: 前后机水平加速度约束统一为1.0m/s^2；该斜率参数直接限制发送给PX4的速度指令变化率。
    pnh.param("vel_slew_rate_xy", vel_slew_rate_xy, 1.0);
    pnh.param("vel_slew_rate_z", vel_slew_rate_z, 1.0);
    // 2026-07-27: 控制器默认水平速度上限与前机FUEL规划的0.5m/s一致，UAV0/UAV1共用同一安全上限。
    pnh.param("max_cmd_speed_xy", max_cmd_speed_xy, 0.50);
    pnh.param("max_cmd_speed_z", max_cmd_speed_z, 0.35);
    // 2026-07-13: 加速度前馈同样限制到本次保守规划范围，避免柱边速度虽限幅但前馈仍瞬间推得过猛。
    pnh.param("max_cmd_acc_xy", max_cmd_acc_xy, 1.00);
    pnh.param("max_cmd_acc_z", max_cmd_acc_z, 0.80);
    last_cmd_vx = 0.0;
    last_cmd_vy = 0.0;
    last_cmd_vz = 0.0;
    last_traj_cmd_time = ros::Time(0);
    last_tf_stamp = ros::Time(0);
    last_control_stamp = ros::Time(0);
    last_land_mode_request = ros::Time(0);
}

// ===============================================
// 回调函数
// ===============================================
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 避免对同一 frame 用重复或回退的时间戳反复广播，触发 TF_REPEATED_DATA。
    if (msg->header.stamp > last_tf_stamp)
    {
        ts.stamp_ = msg->header.stamp;
        ts.frame_id_ = world_frame;
        ts.child_frame_id_ = drone_frame;
        ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tfBroadcasterPointer.sendTransform(ts);
        last_tf_stamp = msg->header.stamp;
    }

    if (!get_now_pos)
    {
        now_x = msg->pose.pose.position.x;
        now_y = msg->pose.pose.position.y;
        now_yaw = yaw;
        get_now_pos = true;
    }
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;
    position_z = msg->pose.pose.position.z;
    current_yaw = yaw;
    have_odom = true;

    // 高度门控只在首次越过阈值时锁存。避免临界高度波动或后续降落时
    // 反复启停控制器；UAV0/UAV1 分别使用各自的 FAST-LIO 里程计。
    if (!planner_cmd_enabled && std::isfinite(position_z) &&
        position_z > planner_enable_height)
    {
        planner_cmd_enabled = true;
        receive = false;
        last_traj_cmd_time = ros::Time(0);
        ROS_WARN("[%s] FAST-LIO z=%.3fm > %.3fm，控制器开始接收规划轨迹",
                 vehicle_ns.c_str(), position_z, planner_enable_height);
    }

    // 2026-07-28: 两路位姿到齐后锁定初始平移，不要求其坐标原点完全相同。
    if (have_mavros_pose && !mavros_odom_alignment_ready)
    {
        mavros_odom_offset_x = mavros_position_x - position_x;
        mavros_odom_offset_y = mavros_position_y - position_y;
        mavros_odom_offset_z = mavros_position_z - position_z;
        mavros_odom_alignment_ready = true;
        mavros_pose_consistent = true;
        ROS_WARN("[pose_consistency] aligned MAVROS-planner offset=(%.3f,%.3f,%.3f)",
                 mavros_odom_offset_x, mavros_odom_offset_y, mavros_odom_offset_z);
    }
}

void Ctrl::mavros_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 2026-07-27: PositionTarget 发往 PX4 local NED，因此锁点优先使用 MAVROS 本地位姿，避免 FAST-LIO/world 偏移直接混入飞控坐标。
    mavros_position_x = msg->pose.position.x;
    mavros_position_y = msg->pose.position.y;
    mavros_position_z = msg->pose.position.z;
    mavros_yaw = tf::getYaw(msg->pose.orientation);
    have_mavros_pose = std::isfinite(mavros_position_x) && std::isfinite(mavros_position_y) &&
                       std::isfinite(mavros_position_z) && std::isfinite(mavros_yaw);
    if (have_mavros_pose && have_odom)
    {
        if (!mavros_odom_alignment_ready)
        {
            mavros_odom_offset_x = mavros_position_x - position_x;
            mavros_odom_offset_y = mavros_position_y - position_y;
            mavros_odom_offset_z = mavros_position_z - position_z;
            mavros_odom_alignment_ready = true;
        }
        const double error_x = mavros_position_x - (position_x + mavros_odom_offset_x);
        const double error_y = mavros_position_y - (position_y + mavros_odom_offset_y);
        const double error_z = mavros_position_z - (position_z + mavros_odom_offset_z);
        const double horizontal_error = std::hypot(error_x, error_y);
        mavros_pose_consistent = horizontal_error <= mavros_odom_max_horizontal_error &&
                                 std::fabs(error_z) <= mavros_odom_max_vertical_error;
        if (!mavros_pose_consistent)
            ROS_ERROR_THROTTLE(0.5,
                               "[pose_consistency] reject MAVROS pose: horizontal=%.2fm vertical=%.2fm "
                               "mavros=(%.2f,%.2f,%.2f) planner=(%.2f,%.2f,%.2f)",
                               horizontal_error, std::fabs(error_z), mavros_position_x,
                               mavros_position_y, mavros_position_z, position_x, position_y,
                               position_z);
    }
    else
    {
        mavros_pose_consistent = false;
    }
}

void Ctrl::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;
    ROS_INFO_THROTTLE(2.0,
                      "收到目标点 (%.2f, %.2f)，等待 /planning/pos_cmd 后再进入轨迹跟踪",
                      targetpos_x, targetpos_y);
}

void Ctrl::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if (!planner_cmd_enabled)
    {
        ROS_INFO_THROTTLE(2.0, "FAST-LIO z 尚未超过 %.2fm，忽略规划器轨迹",
                          planner_enable_height);
        return;
    }

    ego = *msg;

    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;

    ego_vel_x = ego.velocity.x;
    ego_vel_y = ego.velocity.y;
    ego_vel_z = ego.velocity.z;
    
    // 获取轨迹加速度
    ego_a_x = ego.acceleration.x;
    ego_a_y = ego.acceleration.y;
    ego_a_z = ego.acceleration.z;

    ego_yaw = ego.yaw;
    ego_yaw_rate = ego.yaw_dot;

    receive = true;
    last_traj_cmd_time = ros::Time::now();
}

void Ctrl::safety_hold_cb(const std_msgs::Bool::ConstPtr& msg)
{
    // 2026-07-27: 只在 false->true 边沿锁存一次位置；FUEL 重复发布 true 时不能跟着漂移位置不断重置锁点。
    const bool entering_hold = msg->data && !safety_hold_active;
    safety_hold_active = msg->data;
    if (safety_hold_active)
    {
        receive = false;
        last_cmd_vx = last_cmd_vy = last_cmd_vz = 0.0;
        if (entering_hold || !safety_hold_position_latched)
        {
            // 2026-07-28: 只有通过初始偏移一致性检查的 MAVROS 位姿才允许成为位置锁点。
            safety_hold_uses_mavros_frame = have_mavros_pose && mavros_pose_consistent;
            if (safety_hold_uses_mavros_frame)
            {
                safety_hold_x = mavros_position_x;
                safety_hold_y = mavros_position_y;
                safety_hold_z = mavros_position_z;
                safety_hold_yaw = mavros_yaw;
            }
            else if (have_odom)
            {
                // 2026-07-27: MAVROS 位姿尚未到达时保留 FAST-LIO 闭环降级，仍比无位置反馈的纯零速度可靠。
                safety_hold_x = position_x;
                safety_hold_y = position_y;
                safety_hold_z = position_z;
                safety_hold_yaw = current_yaw;
            }
            safety_hold_position_latched = safety_hold_uses_mavros_frame || have_odom;
            if (safety_hold_position_latched)
                ROS_ERROR("[safety_hold] 锁点悬停，旧轨迹已丢弃，target=(%.3f,%.3f,%.3f) source=%s",
                          safety_hold_x, safety_hold_y, safety_hold_z,
                          safety_hold_uses_mavros_frame ? "mavros_local" : "fastlio_fallback");
            else
                ROS_ERROR("[safety_hold] 已丢弃旧轨迹，等待首帧位姿后锁定悬停点");
        }
    }
    else
    {
        safety_hold_position_latched = false;
        ROS_WARN("[safety_hold] 新轨迹已发布，解除悬停门控");
    }
}

void Ctrl::landing_request_cb(const std_msgs::Bool::ConstPtr& msg)
{
    // 2026-07-16: true表示任务层已锁定最终落点；来源可以是二维码确认，也可以是当前启用的地图直降模式。
    if (!msg->data) return;
    landing_requested = true;
    safety_hold_active = true;
    receive = false;
    // 2026-07-27: AUTO.LAND 服务暂未接受时仍需拥有有效锁点，不能绕过普通 safety_hold 回调后使用零值目标。
    // 2026-07-28: 降落请求同样不得锁存已经与规划里程计分裂的 MAVROS 坐标。
    safety_hold_uses_mavros_frame = have_mavros_pose && mavros_pose_consistent;
    if (safety_hold_uses_mavros_frame)
    {
        safety_hold_x = mavros_position_x;
        safety_hold_y = mavros_position_y;
        safety_hold_z = mavros_position_z;
        safety_hold_yaw = mavros_yaw;
    }
    else if (have_odom)
    {
        safety_hold_x = position_x;
        safety_hold_y = position_y;
        safety_hold_z = position_z;
        safety_hold_yaw = current_yaw;
    }
    safety_hold_position_latched = safety_hold_uses_mavros_frame || have_odom;
    ROS_ERROR("[mission_land] 收到最终落点确认后的降落请求，准备切换 AUTO.LAND");
}

// ===============================================
// 控制主循环
// ===============================================
void Ctrl::control(const ros::TimerEvent&)
{
    if (!have_odom)
    {
        ROS_WARN_THROTTLE(2.0, "等待里程计中...");
        return;
    }

    if (!planner_cmd_enabled)
        ROS_INFO_THROTTLE(2.0,
                          "等待 FAST-LIO z > %.2fm；持续发布 OFFBOARD 起飞 setpoint",
                          planner_enable_height);

    // 悬停交接完成后，如果暂时没有轨迹，发布零速度心跳维持 OFFBOARD。
    // 只有在解锁(armed)时允许非零速度生效。
    bool armed = current_state.armed;
    bool allow_nonzero = armed;

    // 初始化 current_goal 的 header/frame/type_mask（心跳也会发布）
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = velocity_mask;
    double dt = last_control_stamp.isZero() ? 0.02 : (current_goal.header.stamp - last_control_stamp).toSec();
    if (dt <= 1e-4 || dt > 0.2) dt = 0.02;
    last_control_stamp = current_goal.header.stamp;

    // 2026-07-13: 降落接管优先级最高；切换成功后停止发送 OFFBOARD 轨迹，交给 PX4 自动降落。
    if (landing_requested)
    {
        if (current_state.mode != "AUTO.LAND" &&
            (last_land_mode_request.isZero() ||
             (ros::Time::now() - last_land_mode_request).toSec() > 1.0))
        {
            mavros_msgs::SetMode mode_cmd;
            mode_cmd.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent)
                ROS_ERROR("[mission_land] PX4 已接受 AUTO.LAND");
            else
                ROS_ERROR("[mission_land] PX4 暂未接受 AUTO.LAND，将继续零速度悬停并重试");
            last_land_mode_request = ros::Time::now();
        }
        if (current_state.mode == "AUTO.LAND") return;
    }

    // 2026-07-27: 规划碰撞或失败期间锁住进入 hold 时的位置，不能只发无位置恢复项的零速度。
    if (safety_hold_active)
    {
        // 2026-07-27: hold 早于位姿回调到达时，在控制循环获得首帧里程计后补锁一次，禁止使用默认零值。
        if (!safety_hold_position_latched)
        {
            // 2026-07-28: 延迟获得位姿时也必须先通过 MAVROS/规划里程计一致性门控。
            safety_hold_uses_mavros_frame = have_mavros_pose && mavros_pose_consistent;
            safety_hold_x = safety_hold_uses_mavros_frame ? mavros_position_x : position_x;
            safety_hold_y = safety_hold_uses_mavros_frame ? mavros_position_y : position_y;
            safety_hold_z = safety_hold_uses_mavros_frame ? mavros_position_z : position_z;
            safety_hold_yaw = safety_hold_uses_mavros_frame ? mavros_yaw : current_yaw;
            safety_hold_position_latched = true;
        }
        if (safety_hold_position_latched && safety_hold_uses_mavros_frame)
        {
            // 2026-07-27: MAVROS/PX4 同坐标系下直接发固定位置目标，实测问题中的34秒长 hold 不再积累漂移。
            current_goal.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            current_goal.position.x = safety_hold_x;
            current_goal.position.y = safety_hold_y;
            current_goal.position.z = safety_hold_z;
            current_goal.yaw = safety_hold_yaw;
        }
        else
        {
            // 2026-07-27: MAVROS 位姿缺失时以 FAST-LIO 锁点误差生成限速回正速度，保留可控的安全降级。
            current_goal.type_mask = velocity_mask;
            const double hold_kp_xy = 0.8;
            const double hold_kp_z = 0.8;
            double hold_vx = hold_kp_xy * (safety_hold_x - position_x);
            double hold_vy = hold_kp_xy * (safety_hold_y - position_y);
            const double hold_speed_xy = std::hypot(hold_vx, hold_vy);
            const double hold_speed_xy_max = 0.20;
            if (hold_speed_xy > hold_speed_xy_max && hold_speed_xy > 1e-6)
            {
                hold_vx *= hold_speed_xy_max / hold_speed_xy;
                hold_vy *= hold_speed_xy_max / hold_speed_xy;
            }
            current_goal.velocity.x = hold_vx;
            current_goal.velocity.y = hold_vy;
            current_goal.velocity.z = std::max(-0.15, std::min(
                0.15, hold_kp_z * (safety_hold_z - position_z)));
            current_goal.acceleration_or_force.x = 0.0;
            current_goal.acceleration_or_force.y = 0.0;
            current_goal.acceleration_or_force.z = 0.0;
            current_goal.yaw = safety_hold_yaw;
        }
        local_pos_pub.publish(current_goal);
        last_cmd_vx = last_cmd_vy = last_cmd_vz = 0.0;
        ROS_WARN_THROTTLE(1.0,
                          "[safety_hold] 锁点悬停中 target=(%.2f,%.2f,%.2f) source=%s，等待安全新轨迹",
                          safety_hold_x, safety_hold_y, safety_hold_z,
                          safety_hold_uses_mavros_frame ? "mavros_local" : "fastlio_fallback");
        return;
    }

    // 没有收到轨迹时保持起始水平位置并爬升/悬停。
    // 该 setpoint 从地面阶段就以 50Hz 发布，以满足 PX4 进入 OFFBOARD 前必须先收到
    // 连续 setpoint 流的要求；规划轨迹仍受 planner_cmd_enabled 高度门控保护。
    if (!receive)
    {
        const double hold_kp_xy = 1.0;
        const double takeoff_kp_z = 0.5;
        current_goal.velocity.x = std::max(-0.20, std::min(0.20,
            hold_kp_xy * (now_x - position_x)));
        current_goal.velocity.y = std::max(-0.20, std::min(0.20,
            hold_kp_xy * (now_y - position_y)));
        current_goal.velocity.z = std::max(-max_cmd_speed_z, std::min(max_cmd_speed_z,
            takeoff_kp_z * (offboard_takeoff_height - position_z)));
        current_goal.acceleration_or_force.x = 0.0;
        current_goal.acceleration_or_force.y = 0.0;
        current_goal.acceleration_or_force.z = 0.0;
        current_goal.yaw = now_yaw;
        local_pos_pub.publish(current_goal);
        last_cmd_vx = current_goal.velocity.x;
        last_cmd_vy = current_goal.velocity.y;
        last_cmd_vz = current_goal.velocity.z;
        ROS_INFO_THROTTLE(2.0,
                          "未收到轨迹，OFFBOARD 起飞/悬停 z=%.2f -> %.2f，armed=%d",
                          position_z, offboard_takeoff_height, armed);
        return;
    }

    if (!last_traj_cmd_time.isZero())
    {
        const double traj_stale = (ros::Time::now() - last_traj_cmd_time).toSec();
        if (traj_stale > traj_cmd_timeout)
        {
            receive = false;
            ROS_WARN_THROTTLE(1.0,
                              "规划轨迹超时 %.2fs > %.2fs，停止跟踪并等待新轨迹",
                              traj_stale, traj_cmd_timeout);
            return;
        }
    }

    // 轨迹跟踪逻辑：计算期望速度（但仅在 armed 时允许非零）
    // 【参数优化】Kp 降低至 0.7 配合加速度前馈；Kv 设为 1.0 完全利用规划速度
    double Kp = 0.7; 
    double Kv = 1.0; 
    // 2026-07-13: 使用参数化矢量限速，防止 Kp 位置误差把 0.5m/s 规划轨迹放大到数米每秒。
    double max_v = max_cmd_speed_z;
    double max_v_xy = max_cmd_speed_xy;

    double vx = Kv * ego_vel_x + Kp * (ego_pos_x - position_x);
    double vy = Kv * ego_vel_y + Kp * (ego_pos_y - position_y);
    double vz = Kv * ego_vel_z + Kp * (ego_pos_z - position_z);

    // 限幅
    const double speed_xy = std::hypot(vx, vy);
    if (speed_xy > max_v_xy && speed_xy > 1e-6)
    {
        vx *= max_v_xy / speed_xy;
        vy *= max_v_xy / speed_xy;
    }
    vz = std::max(std::min(vz, max_v), -max_v);

    // 2026-07-08: 将世界系速度投到机体系，限制“朝机尾方向”的后退速度，避免窄通道里因重规划抖动出现大幅倒飞。
    double body_vx =  cos(current_yaw) * vx + sin(current_yaw) * vy;
    double body_vy = -sin(current_yaw) * vx + cos(current_yaw) * vy;
    if (body_vx < -max_reverse_speed)
    {
        body_vx = -max_reverse_speed;
        vx = cos(current_yaw) * body_vx - sin(current_yaw) * body_vy;
        vy = sin(current_yaw) * body_vx + cos(current_yaw) * body_vy;
    }

    // 2026-07-08: 对速度指令增加斜率限制，削掉重规划切换瞬间的尖跳，避免看起来“猛地倒一截”。
    const double max_dv_xy = vel_slew_rate_xy * dt;
    const double max_dv_z = vel_slew_rate_z * dt;
    auto clamp_delta = [](double target, double last, double max_delta)
    {
        double delta = target - last;
        if (delta > max_delta) return last + max_delta;
        if (delta < -max_delta) return last - max_delta;
        return target;
    };
    // 2026-07-27: 水平速度增量按XY矢量统一限幅，确保斜向飞行时的合加速度也不超过设定的1.0m/s^2。
    double delta_vx = vx - last_cmd_vx;
    double delta_vy = vy - last_cmd_vy;
    const double delta_v_xy = std::hypot(delta_vx, delta_vy);
    if (delta_v_xy > max_dv_xy && delta_v_xy > 1e-6)
    {
        delta_vx *= max_dv_xy / delta_v_xy;
        delta_vy *= max_dv_xy / delta_v_xy;
        vx = last_cmd_vx + delta_vx;
        vy = last_cmd_vy + delta_vy;
    }
    vz = clamp_delta(vz, last_cmd_vz, max_dv_z);

    // 如果未解锁，全部置零（只发布心跳）
    if (!allow_nonzero)
    {
        current_goal.velocity.x = 0.0;
        current_goal.velocity.y = 0.0;
        current_goal.velocity.z = 0.0;
        current_goal.acceleration_or_force.x = 0.0;
        current_goal.acceleration_or_force.y = 0.0;
        current_goal.acceleration_or_force.z = 0.0;
    }
    else
    {
        current_goal.velocity.x = vx;
        current_goal.velocity.y = vy;
        current_goal.velocity.z = vz;
        
        // 2026-07-13: 矢量限制水平加速度、单独限制竖直加速度，保留前馈但不允许超过窄道安全配置。
        double ax = ego_a_x;
        double ay = ego_a_y;
        const double acc_xy = std::hypot(ax, ay);
        if (acc_xy > max_cmd_acc_xy && acc_xy > 1e-6)
        {
            ax *= max_cmd_acc_xy / acc_xy;
            ay *= max_cmd_acc_xy / acc_xy;
        }
        current_goal.acceleration_or_force.x = ax;
        current_goal.acceleration_or_force.y = ay;
        current_goal.acceleration_or_force.z =
            std::max(std::min(ego_a_z, max_cmd_acc_z), -max_cmd_acc_z);
    }

    // 2026-07-20: 控制器必须直接执行规划器的平滑ego_yaw，使Gazebo实机、RViz机体和规划航向框
    // 使用同一航向源；旧逻辑按瞬时速度重新计算yaw，会在低速/重规划时与框不一致并产生跳变。
    double desire_yaw = current_yaw;
    if (allow_yaw && std::isfinite(ego_yaw)) desire_yaw = ego_yaw;
    desire_yaw = wrapAngle(desire_yaw);

    current_goal.type_mask = velocity_mask;
    current_goal.yaw = desire_yaw;

    // publish (心跳或真实控制值)
    local_pos_pub.publish(current_goal);
    last_cmd_vx = current_goal.velocity.x;
    last_cmd_vy = current_goal.velocity.y;
    last_cmd_vz = current_goal.velocity.z;

    double cmd_dx = ego_pos_x - position_x;
    double cmd_dy = ego_pos_y - position_y;
    double cmd_dist_xy = sqrt(cmd_dx * cmd_dx + cmd_dy * cmd_dy);

    ROS_INFO_THROTTLE(1.0, "跟踪中 pos:(%.3f,%.3f,%.3f) | raw_cmd:(%.3f,%.3f,%.3f) | dist_xy=%.3f | vel:(%.2f,%.2f) | Yaw: curr=%.2f -> cmd=%.2f",
                      position_x, position_y, position_z,
                      ego_pos_x, ego_pos_y, ego_pos_z,
                      cmd_dist_xy,
                      current_goal.velocity.x, current_goal.velocity.y, 
                      current_yaw, desire_yaw);
}

// ===============================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cxr_egoctrl_v1");
    setlocale(LC_ALL,"");
    Ctrl ctrl;
    ros::spin();
    return 0;
}
