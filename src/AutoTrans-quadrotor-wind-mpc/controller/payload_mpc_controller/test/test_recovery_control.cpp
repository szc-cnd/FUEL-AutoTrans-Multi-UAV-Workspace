#include <gtest/gtest.h>

#include "recovery_control.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

TEST(RecoveryControl, LevelsAttitudeAtLockedYawAndCompensatesTilt)
{
    const double yaw = 0.7;
    const Eigen::Quaterniond attitude =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(30.0 * kPi / 180.0, Eigen::Vector3d::UnitX());

    const PayloadMPC::RecoveryAttitudeCommand command =
        PayloadMPC::makeRecoveryAttitudeCommand(
            attitude, yaw, 0.54, 0.8, 30.0 * kPi / 180.0);

    ASSERT_TRUE(command.valid);
    const Eigen::Matrix3d rotation = command.orientation.toRotationMatrix();
    EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), yaw, 1.0e-9);
    EXPECT_NEAR(rotation(2, 2), 1.0, 1.0e-9);
    EXPECT_NEAR(command.tilt_rad, 30.0 * kPi / 180.0, 1.0e-9);
    EXPECT_NEAR(command.normalized_thrust, 0.54 / std::cos(30.0 * kPi / 180.0),
                1.0e-9);
}

TEST(RecoveryControl, CapsTiltCompensationAndNormalizedThrust)
{
    const Eigen::Quaterniond attitude(
        Eigen::AngleAxisd(70.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));
    const PayloadMPC::RecoveryAttitudeCommand command =
        PayloadMPC::makeRecoveryAttitudeCommand(
            attitude, -0.3, 0.70, 0.75, 30.0 * kPi / 180.0);

    ASSERT_TRUE(command.valid);
    EXPECT_NEAR(command.normalized_thrust, 0.75, 1.0e-9);
}

TEST(RecoveryControl, RejectsInvalidAttitude)
{
    const Eigen::Quaterniond invalid(0.0, 0.0, 0.0, 0.0);
    const PayloadMPC::RecoveryAttitudeCommand command =
        PayloadMPC::makeRecoveryAttitudeCommand(
            invalid, 0.0, 0.54, 0.8, 30.0 * kPi / 180.0);

    EXPECT_FALSE(command.valid);
    EXPECT_TRUE(std::isfinite(command.normalized_thrust));
}

TEST(RecoveryControl, RequiresPositionVelocityAndTiltConvergence)
{
    const Eigen::Vector3d locked_position(1.0, 2.0, 0.6);
    const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(PayloadMPC::recoveryStateConverged(
        locked_position + Eigen::Vector3d(0.05, 0.02, -0.01), locked_position,
        Eigen::Vector3d(0.05, 0.04, 0.03), level,
        0.15, 0.15, 0.10, 8.0 * kPi / 180.0));
    EXPECT_FALSE(PayloadMPC::recoveryStateConverged(
        locked_position, locked_position, Eigen::Vector3d(0.16, 0.0, 0.0), level,
        0.15, 0.15, 0.10, 8.0 * kPi / 180.0));
    EXPECT_FALSE(PayloadMPC::recoveryStateConverged(
        locked_position, locked_position, Eigen::Vector3d::Zero(),
        Eigen::Quaterniond(Eigen::AngleAxisd(9.0 * kPi / 180.0,
                                             Eigen::Vector3d::UnitX())),
        0.15, 0.15, 0.10, 8.0 * kPi / 180.0));
}

TEST(RecoveryControl, ReusesLastInputOnlyForCalmFiniteState)
{
    const Eigen::Vector3d velocity(0.05, 0.02, 0.01);
    const Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    Eigen::Vector4d input(17.8, 0.10, -0.08, 0.05);
    EXPECT_TRUE(PayloadMPC::conservativeLastValidInput(
        velocity, attitude, input, 0.15, 0.10, 8.0 * kPi / 180.0,
        0.25, 1.0, 29.5));

    input.y() = 0.40;
    EXPECT_FALSE(PayloadMPC::conservativeLastValidInput(
        velocity, attitude, input, 0.15, 0.10, 8.0 * kPi / 180.0,
        0.25, 1.0, 29.5));

    input.y() = 0.10;
    EXPECT_FALSE(PayloadMPC::conservativeLastValidInput(
        Eigen::Vector3d(0.20, 0.0, 0.0), attitude, input,
        0.15, 0.10, 8.0 * kPi / 180.0, 0.25, 1.0, 29.5));
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
