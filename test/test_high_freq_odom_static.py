#!/usr/bin/env python3
"""Static checks for the UAV1-compatible high-frequency FAST-LIO odometry output."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "laserMapping.cpp").read_text(encoding="utf-8")
IMU_PROCESSING = (ROOT / "src" / "IMU_Processing.hpp").read_text(encoding="utf-8")
LAUNCH = (ROOT / "launch" / "mapping_mid360.launch").read_text(encoding="utf-8")


def test_uav1_compatible_high_frequency_odom_source():
    required_source = (
        "void updateLatestStates()",
        "void fastPredictIMU(double t, V3D acc, V3D gyr)",
        "acc = acc * G_m_s2 / p_imu->GetMeanAccNorm();",
        "fastPredictIMU(\n",
        'nh.param<string>("publish/high_freq_odom_topic"',
        "odomHigh_speed = nh.advertise<nav_msgs::Odometry>",
        "imu_mps2_pub.publish(imu_mps2);",
        "imu_mps2.linear_acceleration.x *= acceleration_scale;",
        "imu_mps2.angular_velocity = msg->angular_velocity;",
        "updateLatestStates();",
    )
    missing = [item for item in required_source if item not in SOURCE]
    assert not missing, "missing source integration: " + ", ".join(missing)

    assert "double GetMeanAccNorm() const { return mean_acc.norm(); }" in IMU_PROCESSING


def test_uav0_namespaced_high_frequency_topics():
    required_launch = (
        '<arg name="high_freq_odom_topic"',
        '<arg name="imu_mps2_topic"',
        '<param name="publish/high_freq_odom_topic"',
        '<param name="publish/imu_mps2_topic"',
    )
    missing = [item for item in required_launch if item not in LAUNCH]
    assert not missing, "missing launch integration: " + ", ".join(missing)
    assert 'default="UAV0"' in LAUNCH
    assert 'default="/$(arg vehicle_ns)/fast_lio/Odom_high_freq"' in LAUNCH
    assert 'default="/$(arg vehicle_ns)/livox/imu_mps2"' in LAUNCH
