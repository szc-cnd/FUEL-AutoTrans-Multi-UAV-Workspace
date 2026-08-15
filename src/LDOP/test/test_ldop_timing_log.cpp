#include <gtest/gtest.h>

#include <ldop/utils.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

TEST(LdopTimingLogTest, FormatsNonVerboseProcessingSummary) {
  LdopProcessingTimingTotals timing;
  timing.map_module_ms = 1.23456;
  timing.cluster_module_ms = 2.0;
  timing.tracking_module_ms = 3.4567;
  timing.prediction_module_ms = 4.0;
  timing.ldop_total_ms = 12.3456;

  EXPECT_EQ(formatLdopProcessingTimingSummary(7U, timing),
            "LDOP timing summary frame #7: mapModule=1.235ms, "
            "clusterModule=2.000ms, trackingModule=3.457ms, "
            "predictionModule=4.000ms, total=12.346ms");
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_ldop_timing_log");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
