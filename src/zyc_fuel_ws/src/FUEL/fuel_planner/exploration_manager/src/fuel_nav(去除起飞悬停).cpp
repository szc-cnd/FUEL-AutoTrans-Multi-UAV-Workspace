//在原版的基础上去除了起飞悬停的功能


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

#define VELOCITY2D_CONTROL 0b101111000111

visualization_msgs::Marker trackpoint;
ros::Publisher *pubMarkerPointer;
tf::StampedTransform ts;
tf::TransformBroadcaster *tfBroadcasterPointer;
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw, targetpos_x, targetpos_y;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate;
quadrotor_msgs::PositionCommand ego;
bool receive = false, get_now_pos = false, hover_done = false;
float pi = 3.14159265;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void position_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    position_msg = *msg;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_frame";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer->sendTransform(ts);
    if (!get_now_pos) {
        now_x = position_msg.pose.pose.position.x;
        now_y = position_msg.pose.pose.position.y;
        tf2::Quaternion quat;
        tf2::convert(msg->pose.pose.orientation, quat);
        now_yaw = yaw;
        get_now_pos = true;
    }
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
    current_yaw = yaw;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (hover_done) {
        target_pos = *msg;
        targetpos_x = target_pos.pose.position.x;
        targetpos_y = target_pos.pose.position.y;
    }
}

void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    receive = true;
    ego = *msg;
    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;
    ego_vel_x = ego.velocity.x;
    ego_vel_y = ego.velocity.y;
    ego_vel_z = ego.velocity.z;
    ego_yaw = ego.yaw;
    ego_yaw_rate = ego.yaw_dot;
}

void hover_done_cb(const std_msgs::Bool::ConstPtr& msg) {
    hover_done = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cxr_egoctrl_v1");
    setlocale(LC_ALL,"");
    ros::NodeHandle nh;
    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
    ("/mavros/setpoint_raw/target_local", 1);
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker> ("/track_drone_point", 5);
    pubMarkerPointer = &pubMarker;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("/mavros/cmd/arming");
    ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>
    ("/mavros/cmd/command");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("/mavros/set_mode");
    ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
    ("/planning/pos_cmd", 10, twist_cb);
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("move_base_simple/goal", 10, target_cb);
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>
    ("/converted_odom",10,position_cb);
    ros::Subscriber hover_done_sub = nh.subscribe<std_msgs::Bool>
    ("/hover_done", 10, hover_done_cb);
    
    ros::Rate rate(50.0);
    
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    for(int i = 100; ros::ok() && i > 0; --i) {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()) {
        if(!hover_done) {
            ros::spinOnce();
            rate.sleep();
        } else {
            if(receive) {
                float yaw_erro;
                yaw_erro = (ego_yaw - current_yaw);
                current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                current_goal.header.stamp = ros::Time::now();
                current_goal.type_mask = velocity_mask;
                current_goal.velocity.x =  0.5 * ego_vel_x + (ego_pos_x - position_x) * 1;
                current_goal.velocity.y =  0.5 * ego_vel_y + (ego_pos_y - position_y) * 1;
                current_goal.velocity.z =  (ego_pos_z - position_z) * 1;
                current_goal.yaw = ego_yaw;
                ROS_INFO("EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2) + pow(current_goal.velocity.y, 2)));
            }
            local_pos_pub.publish(current_goal);
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
