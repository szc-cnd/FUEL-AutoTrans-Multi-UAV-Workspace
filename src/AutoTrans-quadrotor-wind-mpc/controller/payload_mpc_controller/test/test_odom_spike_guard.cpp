#include <gtest/gtest.h>

#include "odom_spike_guard.h"

namespace
{
using PayloadMPC::OdomSpikeGuard;
using PayloadMPC::OdomSpikeGuardConfig;
using PayloadMPC::OdomSpikeGuardResult;

TEST(OdomSpikeGuard, AcceptsConsistentMotion)
{
    OdomSpikeGuard guard(OdomSpikeGuardConfig{});
    EXPECT_EQ(guard.evaluate(1.00, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0)),
              OdomSpikeGuardResult::ACCEPTED);
    EXPECT_EQ(guard.evaluate(1.01, Eigen::Vector3d(0.005, 0.0, 0.0),
                             Eigen::Vector3d(0.5, 0.0, 0.0)),
              OdomSpikeGuardResult::ACCEPTED);
    EXPECT_FALSE(guard.faultActive());
}

TEST(OdomSpikeGuard, RejectsSingleSpikeWithoutLatchingFault)
{
    OdomSpikeGuard guard(OdomSpikeGuardConfig{});
    ASSERT_EQ(guard.evaluate(2.00, Eigen::Vector3d(0.0, 0.0, 0.52),
                             Eigen::Vector3d::Zero()),
              OdomSpikeGuardResult::ACCEPTED);
    EXPECT_EQ(guard.evaluate(2.01, Eigen::Vector3d(0.0, 0.0, 0.45),
                             Eigen::Vector3d(0.0, 0.0, -0.84)),
              OdomSpikeGuardResult::REJECTED_TRANSIENT);
    EXPECT_FALSE(guard.faultActive());
}

TEST(OdomSpikeGuard, LatchesPersistentFlightLogLikeSpikes)
{
    OdomSpikeGuard guard(OdomSpikeGuardConfig{});
    ASSERT_EQ(guard.evaluate(3.00, Eigen::Vector3d(0.0, 0.0, 0.52),
                             Eigen::Vector3d::Zero()),
              OdomSpikeGuardResult::ACCEPTED);
    EXPECT_EQ(guard.evaluate(3.01, Eigen::Vector3d(0.0, 0.0, 0.45),
                             Eigen::Vector3d(0.0, 0.0, -0.84)),
              OdomSpikeGuardResult::REJECTED_TRANSIENT);
    EXPECT_EQ(guard.evaluate(3.05, Eigen::Vector3d(0.0, 0.0, 0.30),
                             Eigen::Vector3d(0.0, 0.0, -1.09)),
              OdomSpikeGuardResult::REJECTED_TRANSIENT);
    EXPECT_EQ(guard.evaluate(3.14, Eigen::Vector3d(0.0, 0.0, 0.54),
                             Eigen::Vector3d(0.0, 0.0, 0.31)),
              OdomSpikeGuardResult::FAULT_LATCHED);
    EXPECT_TRUE(guard.faultActive());
}

TEST(OdomSpikeGuard, RequiresConsecutiveGoodSamplesToRecover)
{
    OdomSpikeGuardConfig config;
    config.recovery_good_samples = 3;
    OdomSpikeGuard guard(config);
    ASSERT_EQ(guard.evaluate(4.00, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
              OdomSpikeGuardResult::ACCEPTED);
    ASSERT_EQ(guard.evaluate(4.01, Eigen::Vector3d(0.0, 0.0, 0.10),
                             Eigen::Vector3d(0.0, 0.0, 0.8)),
              OdomSpikeGuardResult::REJECTED_TRANSIENT);
    ASSERT_EQ(guard.evaluate(4.06, Eigen::Vector3d(0.0, 0.0, -0.10),
                             Eigen::Vector3d(0.0, 0.0, -0.8)),
              OdomSpikeGuardResult::REJECTED_TRANSIENT);
    ASSERT_EQ(guard.evaluate(4.11, Eigen::Vector3d(0.0, 0.0, 0.10),
                             Eigen::Vector3d(0.0, 0.0, 0.8)),
              OdomSpikeGuardResult::FAULT_LATCHED);

    EXPECT_EQ(guard.evaluate(4.12, Eigen::Vector3d(0.0, 0.0, 0.108),
							 Eigen::Vector3d(0.0, 0.0, 0.8)),
              OdomSpikeGuardResult::REJECTED_PERSISTENT);
    EXPECT_EQ(guard.evaluate(4.13, Eigen::Vector3d(0.0, 0.0, 0.116),
							 Eigen::Vector3d(0.0, 0.0, 0.8)),
              OdomSpikeGuardResult::REJECTED_PERSISTENT);
    EXPECT_EQ(guard.evaluate(4.14, Eigen::Vector3d(0.0, 0.0, 0.124),
							 Eigen::Vector3d(0.0, 0.0, 0.8)),
              OdomSpikeGuardResult::RECOVERED);
    EXPECT_FALSE(guard.faultActive());
}
} // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
