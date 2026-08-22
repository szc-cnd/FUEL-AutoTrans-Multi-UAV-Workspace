#!/usr/bin/env python3
"""Static checks for the optional high-frequency FAST-LIO odometry output."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "laserMapping.cpp").read_text(encoding="utf-8")
IMU_PROCESSING = (ROOT / "src" / "IMU_Processing.hpp").read_text(encoding="utf-8")
LAUNCH = (ROOT / "launch" / "mapping_mid360.launch").read_text(encoding="utf-8")
INTEGRATION_LAUNCH = (
    ROOT.parent / "autotrans_reference_bridge" / "launch" / "uav1_diff_autotrans.launch"
).read_text(encoding="utf-8")


def main():
    required_source = (
        "void updateLatestStates()",
        "void fastPredictIMU(double t, V3D acc, V3D gyr)",
        "acc = acc * G_m_s2 / p_imu->GetMeanAccNorm();",
        "fastPredictIMU(\n",
        "nh.param<string>(\"publish/high_freq_odom_topic\"",
        "odomHigh_speed = nh.advertise<nav_msgs::Odometry>",
        "imu_mps2_pub.publish(imu_mps2);",
        "imu_mps2.linear_acceleration.x *= acceleration_scale;",
        "imu_mps2.angular_velocity = msg->angular_velocity;",
        "updateLatestStates();",
    )
    missing = [item for item in required_source if item not in SOURCE]
    if missing:
        raise AssertionError("missing source integration: " + ", ".join(missing))

    if "double GetMeanAccNorm() const { return mean_acc.norm(); }" not in IMU_PROCESSING:
        raise AssertionError("FAST-LIO IMU acceleration scale is not exposed to high-frequency prediction")

    required_launch = (
        '<arg name="high_freq_odom_topic"',
        '<arg name="imu_mps2_topic"',
        '<param name="publish/high_freq_odom_topic"',
        '<param name="publish/imu_mps2_topic"',
    )
    missing = [item for item in required_launch if item not in LAUNCH]
    if missing:
        raise AssertionError("missing launch integration: " + ", ".join(missing))

    if 'default="UAV1"' not in LAUNCH:
        raise AssertionError("launch default vehicle namespace is not UAV1")
    if 'default="/$(arg vehicle_ns)/fast_lio/Odom_high_freq"' not in LAUNCH:
        raise AssertionError("launch high-frequency topic is not vehicle-namespaced")
    if 'default="/$(arg vehicle_ns)/livox/imu_mps2"' not in LAUNCH:
        raise AssertionError("converted IMU topic is not vehicle-namespaced")
    if '<arg name="imu_topic" default="/UAV1/livox/imu_mps2"/>' not in INTEGRATION_LAUNCH:
        raise AssertionError("AutoTrans does not subscribe to the body-aligned MID360 IMU topic")
    if '<arg name="force_attitude_odom_topic" default="$(arg odom_topic)"/>' not in INTEGRATION_LAUNCH:
        raise AssertionError("AutoTrans force estimator does not share the FAST-LIO high-frequency attitude source")

    print("high-frequency FAST-LIO static checks passed")


if __name__ == "__main__":
    main()
