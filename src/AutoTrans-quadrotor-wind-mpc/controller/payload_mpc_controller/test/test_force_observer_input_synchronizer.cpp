#include <gtest/gtest.h>

#include <limits>

#include <Eigen/Geometry>

#include "force_observer_input_synchronizer.h"

namespace
{
using PayloadMPC::ForceObserverInputResult;
using PayloadMPC::ForceObserverInputSynchronizer;
using PayloadMPC::ForceObserverSynchronizedInput;
using PayloadMPC::ForceObserverSyncResult;

ForceObserverInputSynchronizer makeSynchronizer()
{
    ForceObserverInputSynchronizer synchronizer;
    synchronizer.configure(0.5, 0.03, 0.1);
    return synchronizer;
}

TEST(ForceObserverInputSynchronizer, InterpolatesAllInputsAtLatestCommonStamp)
{
    auto synchronizer = makeSynchronizer();
    EXPECT_EQ(synchronizer.addAcceleration(1.00, Eigen::Vector3d::Zero()),
              ForceObserverInputResult::ACCEPTED);
    EXPECT_EQ(synchronizer.addAcceleration(1.02, Eigen::Vector3d(2.0, 4.0, 6.0)),
              ForceObserverInputResult::ACCEPTED);
    ASSERT_EQ(synchronizer.addAttitude(1.00, Eigen::Quaterniond::Identity()),
              ForceObserverInputResult::ACCEPTED);
    ASSERT_EQ(synchronizer.addAttitude(
                  1.02,
                  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()))),
              ForceObserverInputResult::ACCEPTED);
    ASSERT_EQ(synchronizer.addRpm(1.00, Eigen::Vector4d::Constant(1000.0)),
              ForceObserverInputResult::ACCEPTED);
    ASSERT_EQ(synchronizer.addRpm(1.01, Eigen::Vector4d::Constant(1100.0)),
              ForceObserverInputResult::ACCEPTED);

    ForceObserverSynchronizedInput output;
    ASSERT_EQ(synchronizer.synchronize(1.02, output), ForceObserverSyncResult::READY);
    EXPECT_NEAR(output.stamp, 1.01, 1.0e-12);
    EXPECT_TRUE(output.acceleration.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1.0e-12));
    EXPECT_TRUE(output.rpm.isApprox(Eigen::Vector4d::Constant(1100.0), 1.0e-12));
    EXPECT_NEAR(Eigen::AngleAxisd(output.attitude).angle(), M_PI_4, 1.0e-12);
    EXPECT_NEAR(output.acceleration_offset, 0.01, 1.0e-12);
    EXPECT_NEAR(output.attitude_offset, 0.01, 1.0e-12);
    EXPECT_NEAR(output.rpm_offset, 0.0, 1.0e-12);
    EXPECT_EQ(synchronizer.synchronize(1.03, output),
              ForceObserverSyncResult::DUPLICATE_TARGET);
}

TEST(ForceObserverInputSynchronizer, QuaternionInterpolationUsesShortestPath)
{
    auto synchronizer = makeSynchronizer();
    const Eigen::Quaterniond endpoint(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond equivalent_endpoint = endpoint;
    equivalent_endpoint.coeffs() *= -1.0;
    synchronizer.addAcceleration(2.00, Eigen::Vector3d::Zero());
    synchronizer.addAcceleration(2.02, Eigen::Vector3d::Zero());
    synchronizer.addAttitude(2.00, Eigen::Quaterniond::Identity());
    synchronizer.addAttitude(2.02, equivalent_endpoint);
    synchronizer.addRpm(2.01, Eigen::Vector4d::Constant(1000.0));

    ForceObserverSynchronizedInput output;
    ASSERT_EQ(synchronizer.synchronize(2.02, output), ForceObserverSyncResult::READY);
    EXPECT_NEAR(Eigen::AngleAxisd(output.attitude).angle(), M_PI_4, 1.0e-12);
}

TEST(ForceObserverInputSynchronizer, RejectsLargeInterpolationGapAndStaleTarget)
{
    auto synchronizer = makeSynchronizer();
    synchronizer.addAcceleration(3.00, Eigen::Vector3d::Zero());
    synchronizer.addAcceleration(3.10, Eigen::Vector3d::Ones());
    synchronizer.addAttitude(3.05, Eigen::Quaterniond::Identity());
    synchronizer.addRpm(3.05, Eigen::Vector4d::Constant(1000.0));

    ForceObserverSynchronizedInput output;
    EXPECT_EQ(synchronizer.synchronize(3.06, output),
              ForceObserverSyncResult::INTERPOLATION_GAP);

    synchronizer.reset();
    synchronizer.addAcceleration(4.00, Eigen::Vector3d::Zero());
    synchronizer.addAttitude(4.00, Eigen::Quaterniond::Identity());
    synchronizer.addRpm(4.00, Eigen::Vector4d::Constant(1000.0));
    EXPECT_EQ(synchronizer.synchronize(4.11, output), ForceObserverSyncResult::STALE);
}

TEST(ForceObserverInputSynchronizer, RejectsMissingBracket)
{
    auto synchronizer = makeSynchronizer();
    synchronizer.addAcceleration(5.00, Eigen::Vector3d::Zero());
    synchronizer.addAttitude(5.01, Eigen::Quaterniond::Identity());
    synchronizer.addRpm(5.01, Eigen::Vector4d::Constant(1000.0));

    ForceObserverSynchronizedInput output;
    EXPECT_EQ(synchronizer.synchronize(5.02, output), ForceObserverSyncResult::OUT_OF_RANGE);
}

TEST(ForceObserverInputSynchronizer, InvalidAndReversedInputResetHistory)
{
    auto synchronizer = makeSynchronizer();
    Eigen::Vector3d invalid = Eigen::Vector3d::Zero();
    invalid.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(synchronizer.addAcceleration(0.0, Eigen::Vector3d::Zero()),
              ForceObserverInputResult::INVALID);
    EXPECT_EQ(synchronizer.addAcceleration(6.0, invalid),
              ForceObserverInputResult::INVALID);

    synchronizer.addAcceleration(6.00, Eigen::Vector3d::Zero());
    synchronizer.addAttitude(6.00, Eigen::Quaterniond::Identity());
    synchronizer.addRpm(6.00, Eigen::Vector4d::Constant(1000.0));
    EXPECT_EQ(synchronizer.addAcceleration(5.99, Eigen::Vector3d::Ones()),
              ForceObserverInputResult::SOURCE_RESET);
    EXPECT_EQ(synchronizer.accelerationSampleCount(), 1u);
    EXPECT_EQ(synchronizer.attitudeSampleCount(), 0u);
    EXPECT_EQ(synchronizer.rpmSampleCount(), 0u);
}

TEST(ForceObserverInputSynchronizer, BoundsHistoryAndResetClearsState)
{
    auto synchronizer = makeSynchronizer();
    for (int i = 0; i <= 100; ++i)
        synchronizer.addAcceleration(7.0 + 0.01 * i, Eigen::Vector3d::Constant(i));

    EXPECT_LT(synchronizer.accelerationSampleCount(), 55u);
    synchronizer.reset();
    EXPECT_EQ(synchronizer.accelerationSampleCount(), 0u);
    EXPECT_EQ(synchronizer.attitudeSampleCount(), 0u);
    EXPECT_EQ(synchronizer.rpmSampleCount(), 0u);
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
