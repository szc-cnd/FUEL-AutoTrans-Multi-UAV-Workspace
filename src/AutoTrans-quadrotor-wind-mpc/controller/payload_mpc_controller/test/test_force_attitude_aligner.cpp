#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "force_attitude_aligner.h"

namespace
{
using PayloadMPC::ForceAttitudeAligner;
using PayloadMPC::ForceAttitudeAlignmentConfig;
using PayloadMPC::ForceAttitudeSampleResult;

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
    return degrees * kPi / 180.0;
}

Eigen::Quaterniond yawQuaternion(double yaw)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

ForceAttitudeAlignmentConfig testConfig()
{
    ForceAttitudeAlignmentConfig config;
    config.duration = 1.0;
    config.min_samples = 3;
    config.max_body_rate = 0.15;
    config.max_speed = 0.10;
    config.max_tilt_error = radians(5.0);
    config.max_yaw_std = radians(2.0);
    return config;
}

TEST(ForceAttitudeAligner, RecoversKnownYawOffsetAndTransformsPx4Attitude)
{
    ForceAttitudeAligner aligner(testConfig());
    const Eigen::Quaterniond q_px4 =
        yawQuaternion(radians(12.0)) *
        Eigen::Quaterniond(Eigen::AngleAxisd(radians(3.0), Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond q_offset = yawQuaternion(radians(30.0));
    const Eigen::Quaterniond q_lio = q_offset * q_px4;

    EXPECT_EQ(aligner.addSample(10.0, q_lio, q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ACCUMULATING);
    EXPECT_EQ(aligner.addSample(10.5, q_lio, q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ACCUMULATING);
    EXPECT_EQ(aligner.addSample(11.0, q_lio, q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ALIGNED);

    ASSERT_TRUE(aligner.aligned());
    EXPECT_NEAR(aligner.yawOffset(), radians(30.0), 1.0e-9);

    Eigen::Quaterniond q_force;
    ASSERT_TRUE(aligner.transformPx4Attitude(q_px4, q_force));
    EXPECT_NEAR(q_force.angularDistance(q_lio), 0.0, 1.0e-9);
}

TEST(ForceAttitudeAligner, UsesCircularMeanAcrossPiBoundary)
{
    ForceAttitudeAligner aligner(testConfig());
    const Eigen::Quaterniond q_px4 = Eigen::Quaterniond::Identity();

    EXPECT_EQ(aligner.addSample(20.0, yawQuaternion(radians(179.0)), q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ACCUMULATING);
    EXPECT_EQ(aligner.addSample(20.5, yawQuaternion(radians(-179.0)), q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ACCUMULATING);
    EXPECT_EQ(aligner.addSample(21.0, yawQuaternion(radians(179.0)), q_px4,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ALIGNED);

    ASSERT_TRUE(aligner.aligned());
    EXPECT_LT(std::abs(std::abs(aligner.yawOffset()) - kPi), radians(2.0));
}

TEST(ForceAttitudeAligner, RejectsRelativeTiltAboveLimit)
{
    ForceAttitudeAligner aligner(testConfig());
    const Eigen::Quaterniond q_lio(
        Eigen::AngleAxisd(radians(6.0), Eigen::Vector3d::UnitX()));

    EXPECT_EQ(aligner.addSample(30.0, q_lio, Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::TILT_MISMATCH);
    EXPECT_FALSE(aligner.aligned());
    EXPECT_EQ(aligner.sampleCount(), 0U);
}

TEST(ForceAttitudeAligner, DuplicateTimestampDoesNotCountAndMotionResetsWindow)
{
    ForceAttitudeAligner aligner(testConfig());
    const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    EXPECT_EQ(aligner.addSample(40.0, q, q,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::ACCUMULATING);
    EXPECT_EQ(aligner.addSample(40.0, q, q,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::DUPLICATE);
    EXPECT_EQ(aligner.sampleCount(), 1U);

    EXPECT_EQ(aligner.addSample(40.5, q, q,
                                Eigen::Vector3d(0.0, 0.0, 0.2), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::MOVING);
    EXPECT_EQ(aligner.sampleCount(), 0U);
    EXPECT_FALSE(aligner.aligned());
}

TEST(ForceAttitudeAligner, RejectsNonFiniteQuaternion)
{
    ForceAttitudeAligner aligner(testConfig());
    Eigen::Quaterniond invalid = Eigen::Quaterniond::Identity();
    invalid.x() = std::numeric_limits<double>::quiet_NaN();

    EXPECT_EQ(aligner.addSample(50.0, invalid, Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              ForceAttitudeSampleResult::INVALID);
    EXPECT_EQ(aligner.sampleCount(), 0U);
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
