#include <gtest/gtest.h>

#include <ldop/dynamic_object_tracker.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

DynamicObjectDetection makeDetectionAt(const double stamp,
                                       const double x,
                                       const double y,
                                       const double z,
                                       const bool realtime_only,
                                       const bool provisional) {
  DynamicObjectDetection detection;
  detection.stamp = ros::Time(stamp);
  detection.bbox.center = makePoint(x, y, z);
  detection.bbox.size.x = 0.3;
  detection.bbox.size.y = 0.3;
  detection.bbox.size.z = 0.3;
  detection.point_count = 8U;
  detection.corridor_realtime_only = realtime_only;
  detection.corridor_provisional = provisional;
  return detection;
}

DynamicObjectDetection makeDetection(const double stamp,
                                     const bool realtime_only,
                                     const bool provisional) {
  return makeDetectionAt(stamp, 1.0, 0.2, 0.6, realtime_only, provisional);
}

std_msgs::Header makeHeader(const double stamp) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(stamp);
  return header;
}

TEST(DynamicObjectTrackerTest, CorridorProvisionalPublishesAndCoastsAtLastObservation) {
  ros::NodeHandle pnh("~tracker_corridor_realtime");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_provisional_max_hits", 4);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 3);
  pnh.setParam("tracking_max_missed_frames", 10);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(10.0), {makeDetection(10.0, true, true)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(first.prediction_inputs.size(), 1U);
  EXPECT_TRUE(first.prediction_inputs.front().corridor_realtime_only);
  EXPECT_TRUE(first.prediction_inputs.front().corridor_provisional);

  const auto missed_once = tracker.processDynamicTracks(makeHeader(10.1), {});
  ASSERT_EQ(missed_once.dynamic_objects_msg.objects.size(), 1U);
  const auto& state = missed_once.dynamic_objects_msg.objects.front().model_state;
  ASSERT_GE(state.size(), 6U);
  EXPECT_NEAR(state[0], 1.0, 1e-9);
  EXPECT_NEAR(state[1], 0.2, 1e-9);
  EXPECT_NEAR(state[2], 0.6, 1e-9);
  EXPECT_DOUBLE_EQ(state[3], 0.0);
  EXPECT_DOUBLE_EQ(state[4], 0.0);
  EXPECT_DOUBLE_EQ(state[5], 0.0);

  tracker.processDynamicTracks(makeHeader(10.2), {});
  tracker.processDynamicTracks(makeHeader(10.3), {});
  const auto expired_output = tracker.processDynamicTracks(makeHeader(10.4), {});
  EXPECT_TRUE(expired_output.dynamic_objects_msg.objects.empty());
}

TEST(DynamicObjectTrackerTest, GenericFirstHitKeepsOriginalMotionGate) {
  ros::NodeHandle pnh("~tracker_generic_gate");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);

  DynamicObjectTracker tracker(pnh);
  const auto result = tracker.processDynamicTracks(
      makeHeader(20.0), {makeDetection(20.0, false, false)});
  EXPECT_TRUE(result.dynamic_objects_msg.objects.empty());
}

TEST(DynamicObjectTrackerTest, CorridorFarJumpCannotReuseExistingId) {
  ros::NodeHandle pnh("~tracker_corridor_far_jump");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.30);
  pnh.setParam("tracking_corridor_realtime_association_gate_max", 0.50);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 2);
  pnh.setParam("tracking_duplicate_merge_distance", 1.0);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(30.0), {makeDetectionAt(30.0, 1.0, 0.2, 0.6, true, true)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t first_id = first.dynamic_objects_msg.objects.front().id;

  const auto far = tracker.processDynamicTracks(
      makeHeader(30.1), {makeDetectionAt(30.1, 3.0, 0.2, 0.6, true, true)});
  ASSERT_EQ(far.dynamic_objects_msg.objects.size(), 2U);
  EXPECT_NE(far.dynamic_objects_msg.objects.front().id,
            far.dynamic_objects_msg.objects.back().id);
  for (const auto& object : far.dynamic_objects_msg.objects) {
    if (object.model_state.size() >= 3U && object.model_state[0] > 2.0) {
      EXPECT_NE(object.id, first_id);
    }
  }
}

TEST(DynamicObjectTrackerTest, CorridorReappearanceWithinMissedGateKeepsId) {
  ros::NodeHandle pnh("~tracker_corridor_reappearance");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.30);
  pnh.setParam("tracking_corridor_realtime_association_gate_max", 0.50);
  pnh.setParam("tracking_corridor_realtime_association_gate_missed_increment", 0.10);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 3);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(40.0), {makeDetectionAt(40.0, 1.0, 0.2, 0.6, true, true)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t first_id = first.dynamic_objects_msg.objects.front().id;
  tracker.processDynamicTracks(makeHeader(40.1), {});
  tracker.processDynamicTracks(makeHeader(40.2), {});

  const auto recovered = tracker.processDynamicTracks(
      makeHeader(40.3), {makeDetectionAt(40.3, 1.2, 0.2, 0.6, true, true)});
  ASSERT_EQ(recovered.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(recovered.dynamic_objects_msg.objects.front().id, first_id);
  ASSERT_GE(recovered.dynamic_objects_msg.objects.front().model_state.size(), 3U);
  EXPECT_NEAR(recovered.dynamic_objects_msg.objects.front().model_state[0], 1.2, 1e-6);
}

TEST(DynamicObjectTrackerTest, CorridorProvisionalNeedsTwoHitsBeforePublish) {
  ros::NodeHandle pnh("~tracker_corridor_provisional_warmup");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(45.0), {makeDetectionAt(45.0, 1.0, 0.2, 0.6, true, true)});
  EXPECT_TRUE(first.dynamic_objects_msg.objects.empty());

  const auto second = tracker.processDynamicTracks(
      makeHeader(45.1), {makeDetectionAt(45.1, 1.1, 0.2, 0.6, true, true)});
  ASSERT_EQ(second.dynamic_objects_msg.objects.size(), 1U);
}

TEST(DynamicObjectTrackerTest, CorridorOutputLimitKeepsOneRealtimeTrack) {
  ros::NodeHandle pnh("~tracker_corridor_single_output");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.25);
  pnh.setParam("tracking_corridor_realtime_association_gate_max", 0.35);
  pnh.setParam("tracking_corridor_realtime_association_gate_missed_increment", 0.05);
  pnh.setParam("tracking_duplicate_merge_distance", 1.0);

  DynamicObjectTracker tracker(pnh);
  const auto output = tracker.processDynamicTracks(
      makeHeader(60.0),
      {makeDetectionAt(60.0, 1.0, 0.2, 0.6, true, true),
       makeDetectionAt(60.0, 2.0, 0.2, 0.6, true, true)});
  ASSERT_EQ(output.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(output.prediction_inputs.size(), 1U);
}

TEST(DynamicObjectTrackerTest, CorridorTrackNeverMergesWithOrdinaryTrack) {
  ros::NodeHandle pnh("~tracker_corridor_merge_isolation");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_provisional_max_hits", 4);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.30);
  pnh.setParam("tracking_corridor_realtime_association_gate_max", 0.50);
  pnh.setParam("tracking_corridor_realtime_association_gate_missed_increment", 0.10);
  pnh.setParam("tracking_min_hits_to_publish", 1);
  pnh.setParam("tracking_motion_min_displacement", 0.01);
  pnh.setParam("tracking_motion_min_evidence_frames", 1);
  pnh.setParam("tracking_motion_confirmation_speed", 0.01);
  pnh.setParam("tracking_spawn_suppression_distance", 1.0);
  pnh.setParam("tracking_spawn_suppression_iou_threshold", 0.2);
  pnh.setParam("tracking_duplicate_merge_distance", 0.75);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(50.0), {makeDetectionAt(50.0, 1.0, 0.2, 0.6, false, false)});
  const auto ordinary = tracker.processDynamicTracks(
      makeHeader(50.1), {makeDetectionAt(50.1, 1.2, 0.2, 0.6, false, false)});
  ASSERT_FALSE(ordinary.dynamic_objects_msg.objects.empty());
  const std::uint32_t ordinary_id = ordinary.dynamic_objects_msg.objects.front().id;

  // Realtime and ordinary inputs overlap spatially, but they must not share
  // an association or be collapsed by duplicate_merge_distance.
  const auto mixed = tracker.processDynamicTracks(
      makeHeader(50.2), {makeDetectionAt(50.2, 1.25, 0.2, 0.6, true, true)});
  ASSERT_GE(mixed.dynamic_objects_msg.objects.size(), 2U);
  bool ordinary_id_present = false;
  bool corridor_id_present = false;
  for (const auto& object : mixed.dynamic_objects_msg.objects) {
    ordinary_id_present = ordinary_id_present || object.id == ordinary_id;
    corridor_id_present = corridor_id_present || object.id != ordinary_id;
  }
  EXPECT_TRUE(ordinary_id_present);
  EXPECT_TRUE(corridor_id_present);
}

TEST(DynamicObjectTrackerTest, CorridorOutputIsLimitedToOneTrack) {
  ros::NodeHandle pnh("~tracker_corridor_single_output");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto result = tracker.processDynamicTracks(
      makeHeader(60.0),
      {makeDetectionAt(60.0, 1.0, -0.2, 0.6, true, true),
       makeDetectionAt(60.0, 1.0, 0.2, 0.6, true, true)});
  ASSERT_EQ(result.dynamic_objects_msg.objects.size(), 1U);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_dynamic_object_tracker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
