#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include <ldop/dynamic_object_tracker.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

DynamicObjectDetection makeDetectionAt(const double stamp,
                                       const double x,
                                       const double y,
                                       const double z,
                                       const bool realtime_only,
                                       const bool provisional,
                                       const std::uint32_t source_track_id = 0U) {
  DynamicObjectDetection detection;
  detection.stamp = ros::Time(stamp);
  detection.bbox.center = makePoint(x, y, z);
  detection.bbox.size.x = 0.3;
  detection.bbox.size.y = 0.3;
  detection.bbox.size.z = 0.3;
  detection.point_count = 8U;
  detection.corridor_realtime_only = realtime_only;
  detection.corridor_provisional = provisional;
  detection.corridor_source_track_id = source_track_id;
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

TEST(DynamicObjectTrackerTest, CorridorSourceIdKeepsExternalIdAcrossCentroidJump) {
  ros::NodeHandle pnh("~tracker_corridor_source_id");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_source_association_gate", 0.75);
  pnh.setParam("tracking_corridor_realtime_association_gate_max", 0.35);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(70.0),
      {makeDetectionAt(70.0, 1.0, 0.2, 0.6, true, false, 42U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = first.dynamic_objects_msg.objects.front().id;

  // 0.55m 超过普通 realtime gate，但 mapper 源 ID 相同，仍应保持外部 ID。
  const auto jumped = tracker.processDynamicTracks(
      makeHeader(70.1),
      {makeDetectionAt(70.1, 1.0, 0.75, 0.6, true, false, 42U)});
  ASSERT_EQ(jumped.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(jumped.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_NEAR(jumped.dynamic_objects_msg.objects.front().model_state[1], 0.75, 1e-6);
}

TEST(DynamicObjectTrackerTest, CorridorChangedSourceReusesNearbyConfirmedExternalId) {
  ros::NodeHandle pnh("~tracker_corridor_changed_source");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.25);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(75.0),
      {makeDetectionAt(75.0, 1.0, 0.2, 0.6, true, false, 10U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = first.dynamic_objects_msg.objects.front().id;

  // mapper 在同一球上重建了 source track，且又从 provisional 起步。
  // tracker 承接旧 external ID，但不能借用旧 source 的 confirmed 状态。
  const auto changed = tracker.processDynamicTracks(
      makeHeader(75.1),
      {makeDetectionAt(75.1, 1.0, 0.35, 0.6, true, true, 11U)});
  ASSERT_EQ(changed.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(changed.prediction_inputs.size(), 1U);
  EXPECT_EQ(changed.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_EQ(changed.prediction_inputs.front().id, external_id);
  EXPECT_TRUE(changed.prediction_inputs.front().corridor_provisional);
  EXPECT_NEAR(changed.dynamic_objects_msg.objects.front().model_state[1], 0.35, 1e-6);

  const auto reconfirmed = tracker.processDynamicTracks(
      makeHeader(75.2),
      {makeDetectionAt(75.2, 1.0, 0.45, 0.6, true, false, 11U)});
  ASSERT_EQ(reconfirmed.prediction_inputs.size(), 1U);
  EXPECT_EQ(reconfirmed.prediction_inputs.front().id, external_id);
  EXPECT_FALSE(reconfirmed.prediction_inputs.front().corridor_provisional);
}

TEST(DynamicObjectTrackerTest, ChangedSourceKeepsCommittedOutputUntilSecondHit) {
  ros::NodeHandle pnh("~tracker_corridor_pending_changed_source");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(75.5),
      {makeDetectionAt(75.5, 1.0, 0.20, 0.6, true, false, 10U)});
  const auto committed = tracker.processDynamicTracks(
      makeHeader(75.6),
      {makeDetectionAt(75.6, 1.0, 0.20, 0.6, true, false, 10U)});
  ASSERT_EQ(committed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = committed.dynamic_objects_msg.objects.front().id;

  const auto pending = tracker.processDynamicTracks(
      makeHeader(75.7),
      {makeDetectionAt(75.7, 1.0, 0.55, 0.6, true, true, 11U)});
  ASSERT_EQ(pending.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(pending.prediction_inputs.size(), 1U);
  EXPECT_EQ(pending.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_NEAR(pending.dynamic_objects_msg.objects.front().model_state[1], 0.20, 1e-6);
  EXPECT_FALSE(pending.prediction_inputs.front().corridor_provisional);

  const auto committed_new_source = tracker.processDynamicTracks(
      makeHeader(75.8),
      {makeDetectionAt(75.8, 1.0, 0.58, 0.6, true, true, 11U)});
  ASSERT_EQ(committed_new_source.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(committed_new_source.prediction_inputs.size(), 1U);
  EXPECT_EQ(committed_new_source.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_NEAR(committed_new_source.dynamic_objects_msg.objects.front().model_state[1],
              0.58, 1e-6);
  EXPECT_TRUE(committed_new_source.prediction_inputs.front().corridor_provisional);
}

TEST(DynamicObjectTrackerTest, AlternatingChangedSourcesNeverReplaceCommittedState) {
  ros::NodeHandle pnh("~tracker_corridor_alternating_pending_sources");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(75.9),
      {makeDetectionAt(75.9, 1.0, 0.0, 0.6, true, false, 10U)});
  const auto committed = tracker.processDynamicTracks(
      makeHeader(76.0),
      {makeDetectionAt(76.0, 1.0, 0.0, 0.6, true, false, 10U)});
  ASSERT_EQ(committed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = committed.dynamic_objects_msg.objects.front().id;

  const std::array<std::uint32_t, 6U> alternating_sources{
      {11U, 12U, 11U, 12U, 11U, 12U}};
  for (std::size_t index = 0U; index < alternating_sources.size(); ++index) {
    const double stamp = 76.1 + 0.1 * static_cast<double>(index);
    const auto result = tracker.processDynamicTracks(
        makeHeader(stamp),
        {makeDetectionAt(stamp, 1.0, 0.30, 0.6, true, true,
                         alternating_sources[index])});
    if (index < 5U) {
      ASSERT_EQ(result.dynamic_objects_msg.objects.size(), 1U);
      ASSERT_EQ(result.prediction_inputs.size(), 1U);
      EXPECT_EQ(result.dynamic_objects_msg.objects.front().id, external_id);
      EXPECT_NEAR(result.dynamic_objects_msg.objects.front().model_state[1], 0.0, 1e-6);
      EXPECT_FALSE(result.prediction_inputs.front().corridor_provisional);
    } else {
      // changed source 一直无法连续确认时，旧位置只按发布保活窗口输出，
      // 第 6 帧必须停止，不能被 A/B 噪声永久续命。
      EXPECT_TRUE(result.dynamic_objects_msg.objects.empty());
      EXPECT_TRUE(result.prediction_inputs.empty());
    }
  }

  const auto original_source_returns = tracker.processDynamicTracks(
      makeHeader(76.7),
      {makeDetectionAt(76.7, 1.0, 0.05, 0.6, true, false, 10U)});
  ASSERT_EQ(original_source_returns.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(original_source_returns.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_NEAR(original_source_returns.dynamic_objects_msg.objects.front().model_state[1],
              0.05, 1e-6);
}

TEST(DynamicObjectTrackerTest, PendingSourceCannotChainBeyondCommittedAnchorGate) {
  ros::NodeHandle pnh("~tracker_corridor_pending_anchor_gate");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_corridor_changed_source_association_gate", 0.55);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(76.6),
      {makeDetectionAt(76.6, 1.0, 0.0, 0.6, true, false, 10U)});
  const auto committed = tracker.processDynamicTracks(
      makeHeader(76.7),
      {makeDetectionAt(76.7, 1.0, 0.0, 0.6, true, false, 10U)});
  ASSERT_EQ(committed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = committed.dynamic_objects_msg.objects.front().id;

  tracker.processDynamicTracks(
      makeHeader(76.8),
      {makeDetectionAt(76.8, 1.0, 0.50, 0.6, true, true, 11U)});
  const auto outside_anchor = tracker.processDynamicTracks(
      makeHeader(76.9),
      {makeDetectionAt(76.9, 1.0, 0.80, 0.6, true, true, 11U)});
  ASSERT_EQ(outside_anchor.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(outside_anchor.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_NEAR(outside_anchor.dynamic_objects_msg.objects.front().model_state[1], 0.0, 1e-6);
}

TEST(DynamicObjectTrackerTest, CorridorIdentitySurvivesBeyondPublishHold) {
  ros::NodeHandle pnh("~tracker_corridor_internal_identity_hold");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_corridor_realtime_max_internal_missed_frames", 20);
  pnh.setParam("tracking_corridor_changed_source_association_gate", 0.55);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(76.0),
      {makeDetectionAt(76.0, 1.0, 0.20, 0.6, true, false, 10U)});
  const auto first = tracker.processDynamicTracks(
      makeHeader(76.1),
      {makeDetectionAt(76.1, 1.0, 0.20, 0.6, true, false, 10U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t external_id = first.dynamic_objects_msg.objects.front().id;

  DynamicObjectTrackerFrameResult missed;
  for (int frame = 1; frame <= 6; ++frame) {
    missed = tracker.processDynamicTracks(makeHeader(76.1 + 0.1 * frame), {});
  }
  // 对规划器只原位保活 5 帧；第 6 帧必须停止发布旧障碍位置。
  EXPECT_TRUE(missed.dynamic_objects_msg.objects.empty());
  EXPECT_TRUE(missed.prediction_inputs.empty());

  // mapper 在遮挡期间换了 source ID。clusterer 当帧只保留厘米级残片，
  // preferred ID 仍承接身份，但必须先按 provisional 重新确认。
  auto sparse = makeDetectionAt(76.8, 1.0, 0.66, 0.6, true, true, 11U);
  sparse.bbox.size.x = 0.02;
  sparse.bbox.size.y = 0.047;
  sparse.bbox.size.z = 0.003;
  const auto provisional = tracker.processDynamicTracks(makeHeader(76.8), {sparse});
  EXPECT_TRUE(provisional.dynamic_objects_msg.objects.empty());

  sparse.stamp = ros::Time(76.9);
  sparse.corridor_provisional = false;
  sparse.bbox.center.y = 0.67;
  const auto reacquired = tracker.processDynamicTracks(makeHeader(76.9), {sparse});
  // mapper 确认后恢复原 external ID，而不是新建一个跳变编号。
  ASSERT_EQ(reacquired.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(reacquired.prediction_inputs.size(), 1U);
  EXPECT_EQ(reacquired.dynamic_objects_msg.objects.front().id, external_id);
  EXPECT_EQ(reacquired.prediction_inputs.front().id, external_id);
  EXPECT_FALSE(reacquired.prediction_inputs.front().corridor_provisional);
  EXPECT_NEAR(reacquired.dynamic_objects_msg.objects.front().model_state[1], 0.67, 1e-6);
}

TEST(DynamicObjectTrackerTest, PersistentProvisionalDoesNotDisappearAfterFourHits) {
  ros::NodeHandle pnh("~tracker_corridor_unlimited_provisional");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_corridor_provisional_max_hits", 0);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  DynamicObjectTrackerFrameResult result;
  for (int frame = 0; frame < 8; ++frame) {
    const double stamp = 77.0 + 0.1 * frame;
    result = tracker.processDynamicTracks(
        makeHeader(stamp),
        {makeDetectionAt(stamp, 1.0, 0.02 * frame, 0.6, true, true, 31U)});
  }
  ASSERT_EQ(result.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(result.prediction_inputs.size(), 1U);
  EXPECT_TRUE(result.prediction_inputs.front().corridor_provisional);
}

TEST(DynamicObjectTrackerTest, ConflictingSourceCannotInheritConfirmedTrack) {
  ros::NodeHandle pnh("~tracker_corridor_conflicting_source");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 2);

  DynamicObjectTracker tracker(pnh);
  const auto confirmed = tracker.processDynamicTracks(
      makeHeader(78.0),
      {makeDetectionAt(78.0, 1.0, 0.2, 0.6, true, false, 10U)});
  ASSERT_EQ(confirmed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t confirmed_id = confirmed.dynamic_objects_msg.objects.front().id;

  // 即使上游把冲突簇错误标成 confirmed，tracker 也必须强制按 provisional 处理。
  auto conflict = makeDetectionAt(78.1, 1.0, 0.22, 0.6, true, false, 0U);
  conflict.corridor_source_conflict = true;
  const auto result = tracker.processDynamicTracks(makeHeader(78.1), {conflict});
  ASSERT_EQ(result.dynamic_objects_msg.objects.size(), 2U);
  const auto new_object = std::find_if(
      result.dynamic_objects_msg.objects.begin(), result.dynamic_objects_msg.objects.end(),
      [&](const ldop::DynamicObject& object) { return object.id != confirmed_id; });
  ASSERT_NE(new_object, result.dynamic_objects_msg.objects.end());
  const auto new_prediction = std::find_if(
      result.prediction_inputs.begin(), result.prediction_inputs.end(),
      [&](const TrackPredictionInput& input) { return input.id == new_object->id; });
  ASSERT_NE(new_prediction, result.prediction_inputs.end());
  EXPECT_TRUE(new_prediction->corridor_provisional);

  const auto clean_source = tracker.processDynamicTracks(
      makeHeader(78.2),
      {makeDetectionAt(78.2, 1.0, 0.24, 0.6, true, false, 10U)});
  ASSERT_FALSE(clean_source.dynamic_objects_msg.objects.empty());
  const auto committed_track = std::find_if(
      clean_source.dynamic_objects_msg.objects.begin(),
      clean_source.dynamic_objects_msg.objects.end(),
      [&](const ldop::DynamicObject& object) { return object.id == confirmed_id; });
  ASSERT_NE(committed_track, clean_source.dynamic_objects_msg.objects.end());
  EXPECT_NEAR(committed_track->model_state[1], 0.24, 1e-6);
}

TEST(DynamicObjectTrackerTest, RepeatedConflictsStayOnOneProvisionalTrack) {
  ros::NodeHandle pnh("~tracker_corridor_repeated_conflict");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 2);

  DynamicObjectTracker tracker(pnh);
  auto conflict = makeDetectionAt(78.5, 1.0, 0.20, 0.6, true, false, 0U);
  conflict.corridor_source_conflict = true;
  const auto first = tracker.processDynamicTracks(makeHeader(78.5), {conflict});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(first.prediction_inputs.size(), 1U);
  const std::uint32_t conflict_id = first.dynamic_objects_msg.objects.front().id;
  EXPECT_TRUE(first.prediction_inputs.front().corridor_provisional);

  conflict.stamp = ros::Time(78.6);
  conflict.bbox.center.y = 0.24;
  const auto second = tracker.processDynamicTracks(makeHeader(78.6), {conflict});
  ASSERT_EQ(second.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(second.prediction_inputs.size(), 1U);
  EXPECT_EQ(second.dynamic_objects_msg.objects.front().id, conflict_id);
  EXPECT_TRUE(second.prediction_inputs.front().corridor_provisional);
}

TEST(DynamicObjectTrackerTest, ConflictTrackCannotBecomeStickyPreferred) {
  ros::NodeHandle pnh("~tracker_corridor_conflict_not_preferred");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  auto conflict = makeDetectionAt(78.7, 1.0, 0.20, 0.6, true, false, 0U);
  conflict.corridor_source_conflict = true;
  const auto first = tracker.processDynamicTracks(makeHeader(78.7), {conflict});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t conflict_id = first.dynamic_objects_msg.objects.front().id;

  // 正常 provisional 候选与冲突簇无关联关系。它应在命中当帧接管输出，
  // 而不是被上一帧冲突轨迹形成的 sticky preferred 压住。
  const auto clean = tracker.processDynamicTracks(
      makeHeader(78.8),
      {makeDetectionAt(78.8, 2.0, -0.20, 0.6, true, true, 51U)});
  ASSERT_EQ(clean.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(clean.prediction_inputs.size(), 1U);
  EXPECT_NE(clean.dynamic_objects_msg.objects.front().id, conflict_id);
  EXPECT_NEAR(clean.dynamic_objects_msg.objects.front().model_state[0], 2.0, 1e-6);
  EXPECT_TRUE(clean.prediction_inputs.front().corridor_provisional);
}

TEST(DynamicObjectTrackerTest, PreferredIdWinsChangedSourceAgainstOlderDormantTrack) {
  ros::NodeHandle pnh("~tracker_corridor_preferred_dormant");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_corridor_realtime_max_internal_missed_frames", 20);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 2);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(79.0),
      {makeDetectionAt(79.0, 1.0, 0.0, 0.6, true, false, 10U),
       makeDetectionAt(79.0, 1.0, 0.8, 0.6, true, false, 20U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 2U);
  const std::uint32_t older_id = first.dynamic_objects_msg.objects[0].id;
  const std::uint32_t current_id = first.dynamic_objects_msg.objects[1].id;

  // 只命中第二条轨迹，使其成为当前 preferred external ID。
  const auto select_current = tracker.processDynamicTracks(
      makeHeader(79.1),
      {makeDetectionAt(79.1, 1.0, 0.8, 0.6, true, false, 20U)});
  ASSERT_EQ(select_current.dynamic_objects_msg.objects.size(), 2U);

  for (int frame = 1; frame <= 6; ++frame) {
    tracker.processDynamicTracks(makeHeader(79.1 + 0.1 * frame), {});
  }
  const auto reacquired = tracker.processDynamicTracks(
      makeHeader(79.8),
      {makeDetectionAt(79.8, 1.0, 0.4, 0.6, true, false, 30U)});
  ASSERT_EQ(reacquired.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(reacquired.dynamic_objects_msg.objects.front().id, current_id);
  EXPECT_NE(reacquired.dynamic_objects_msg.objects.front().id, older_id);
}

TEST(DynamicObjectTrackerTest, TinyChangedSourceCannotReuseNonPreferredDormantId) {
  ros::NodeHandle pnh("~tracker_corridor_nonpreferred_size_gate");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 3);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(80.0),
      {makeDetectionAt(80.0, 1.0, 0.0, 0.6, true, false, 10U),
       makeDetectionAt(80.0, 1.0, 0.8, 0.6, true, false, 20U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 2U);
  const std::uint32_t older_id = first.dynamic_objects_msg.objects[0].id;

  // 第二条轨迹成为 preferred；第一条留作非 preferred dormant 轨迹。
  tracker.processDynamicTracks(
      makeHeader(80.1),
      {makeDetectionAt(80.1, 1.0, 0.8, 0.6, true, false, 20U)});

  auto tiny = makeDetectionAt(80.2, 1.0, 0.05, 0.6, true, false, 30U);
  tiny.bbox.size.x = 0.02;
  tiny.bbox.size.y = 0.04;
  tiny.bbox.size.z = 0.01;
  const auto result = tracker.processDynamicTracks(makeHeader(80.2), {tiny});
  const auto observed = std::find_if(
      result.dynamic_objects_msg.objects.begin(), result.dynamic_objects_msg.objects.end(),
      [](const ldop::DynamicObject& object) {
        return object.model_state.size() >= 2U &&
               std::abs(object.model_state[1] - 0.05) < 1e-6;
      });
  ASSERT_NE(observed, result.dynamic_objects_msg.objects.end());
  EXPECT_NE(observed->id, older_id);
}

TEST(DynamicObjectTrackerTest, CorridorInternalIdentityExpiresAfterConfiguredWindow) {
  ros::NodeHandle pnh("~tracker_corridor_internal_identity_expiry");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_max_internal_missed_frames", 20);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(81.0),
      {makeDetectionAt(81.0, 1.0, 0.2, 0.6, true, false, 10U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t old_id = first.dynamic_objects_msg.objects.front().id;

  for (int frame = 1; frame <= 21; ++frame) {
    tracker.processDynamicTracks(makeHeader(81.0 + 0.1 * frame), {});
  }
  const auto after_expiry = tracker.processDynamicTracks(
      makeHeader(83.2),
      {makeDetectionAt(83.2, 1.0, 0.2, 0.6, true, false, 11U)});
  ASSERT_EQ(after_expiry.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_NE(after_expiry.dynamic_objects_msg.objects.front().id, old_id);
}

TEST(DynamicObjectTrackerTest, CorridorSameSourceWinsOverCloserChangedSourceTrack) {
  ros::NodeHandle pnh("~tracker_corridor_source_priority");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_association_gate", 0.30);
  pnh.setParam("tracking_corridor_source_association_gate", 0.75);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 2);

  DynamicObjectTracker tracker(pnh);
  const auto first = tracker.processDynamicTracks(
      makeHeader(77.0),
      {makeDetectionAt(77.0, 1.0, 0.0, 0.6, true, false, 41U),
       makeDetectionAt(77.0, 1.0, 0.7, 0.6, true, false, 42U)});
  ASSERT_EQ(first.dynamic_objects_msg.objects.size(), 2U);
  const std::uint32_t source_41_external_id = first.dynamic_objects_msg.objects[0].id;
  const std::uint32_t source_42_external_id = first.dynamic_objects_msg.objects[1].id;

  // 新观测更靠近 source 41 的旧中心，但 source 42 仍在同源 gate 内；
  // source 优先级应防止两条轨迹交换 external ID。
  const auto next = tracker.processDynamicTracks(
      makeHeader(77.1),
      {makeDetectionAt(77.1, 1.0, 0.05, 0.6, true, false, 42U)});
  ASSERT_EQ(next.dynamic_objects_msg.objects.size(), 2U);
  for (const auto& object : next.dynamic_objects_msg.objects) {
    ASSERT_GE(object.model_state.size(), 3U);
    if (object.id == source_41_external_id) {
      EXPECT_NEAR(object.model_state[1], 0.0, 1e-6);
    } else if (object.id == source_42_external_id) {
      EXPECT_NEAR(object.model_state[1], 0.05, 1e-6);
    } else {
      ADD_FAILURE() << "unexpected external ID " << object.id;
    }
  }
}

TEST(DynamicObjectTrackerTest, ConfirmedCoastBeatsNewProvisionalCandidate) {
  ros::NodeHandle pnh("~tracker_corridor_confirmed_priority");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto confirmed = tracker.processDynamicTracks(
      makeHeader(80.0),
      {makeDetectionAt(80.0, 1.0, 0.2, 0.6, true, false, 10U)});
  ASSERT_EQ(confirmed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t confirmed_id = confirmed.dynamic_objects_msg.objects.front().id;

  const auto noise = tracker.processDynamicTracks(
      makeHeader(80.1),
      {makeDetectionAt(80.1, 2.0, -0.4, 0.6, true, true, 99U)});
  ASSERT_EQ(noise.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(noise.dynamic_objects_msg.objects.front().id, confirmed_id);
  EXPECT_NEAR(noise.dynamic_objects_msg.objects.front().model_state[0], 1.0, 1e-6);
  EXPECT_NEAR(noise.dynamic_objects_msg.objects.front().model_state[1], 0.2, 1e-6);
}

TEST(DynamicObjectTrackerTest, IndependentProvisionalWaitsForConfirmationBehindRetainedId) {
  ros::NodeHandle pnh("~tracker_corridor_hidden_provisional_after_publish_hold");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 2);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_corridor_realtime_max_internal_missed_frames", 20);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  tracker.processDynamicTracks(
      makeHeader(84.0),
      {makeDetectionAt(84.0, 1.0, 0.0, 0.6, true, false, 10U)});
  const auto selected = tracker.processDynamicTracks(
      makeHeader(84.1),
      {makeDetectionAt(84.1, 1.0, 0.0, 0.6, true, false, 10U)});
  ASSERT_EQ(selected.dynamic_objects_msg.objects.size(), 1U);

  for (int frame = 1; frame <= 6; ++frame) {
    tracker.processDynamicTracks(makeHeader(84.1 + 0.1 * frame), {});
  }
  tracker.processDynamicTracks(
      makeHeader(84.8),
      {makeDetectionAt(84.8, 2.0, 0.0, 0.6, true, true, 99U)});
  const auto provisional = tracker.processDynamicTracks(
      makeHeader(84.9),
      {makeDetectionAt(84.9, 2.0, 0.05, 0.6, true, true, 99U)});
  EXPECT_TRUE(provisional.dynamic_objects_msg.objects.empty());
  EXPECT_TRUE(provisional.prediction_inputs.empty());

  const auto confirmed = tracker.processDynamicTracks(
      makeHeader(85.0),
      {makeDetectionAt(85.0, 2.0, 0.10, 0.6, true, false, 99U)});
  ASSERT_EQ(confirmed.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_NE(confirmed.dynamic_objects_msg.objects.front().id,
            selected.dynamic_objects_msg.objects.front().id);
  EXPECT_NEAR(confirmed.dynamic_objects_msg.objects.front().model_state[0], 2.0, 1e-6);
}

TEST(DynamicObjectTrackerTest, HiddenProvisionalTrackAccumulatesHitsAndTakesOverWhenConfirmed) {
  ros::NodeHandle pnh("~tracker_corridor_hidden_candidate_lifecycle");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_max_missed_frames", 10);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto confirmed = tracker.processDynamicTracks(
      makeHeader(85.0),
      {makeDetectionAt(85.0, 1.0, 0.2, 0.6, true, false, 10U)});
  ASSERT_EQ(confirmed.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t old_external_id = confirmed.dynamic_objects_msg.objects.front().id;

  const auto provisional_first = tracker.processDynamicTracks(
      makeHeader(85.1),
      {makeDetectionAt(85.1, 2.0, -0.2, 0.6, true, true, 99U)});
  ASSERT_EQ(provisional_first.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(provisional_first.dynamic_objects_msg.objects.front().id, old_external_id);

  const auto provisional_second = tracker.processDynamicTracks(
      makeHeader(85.2),
      {makeDetectionAt(85.2, 2.0, -0.1, 0.6, true, true, 99U)});
  ASSERT_EQ(provisional_second.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(provisional_second.dynamic_objects_msg.objects.front().id, old_external_id);

  // 新候选在隐藏期间应保留同一内部轨迹。第三次命中由 mapper 确认后，
  // 它以创建时分配的下一 external ID 接管，而不是现场重新建轨。
  const auto takeover = tracker.processDynamicTracks(
      makeHeader(85.3),
      {makeDetectionAt(85.3, 2.0, 0.0, 0.6, true, false, 99U)});
  ASSERT_EQ(takeover.dynamic_objects_msg.objects.size(), 1U);
  ASSERT_EQ(takeover.prediction_inputs.size(), 1U);
  EXPECT_EQ(takeover.dynamic_objects_msg.objects.front().id, old_external_id + 1U);
  EXPECT_EQ(takeover.prediction_inputs.front().id, old_external_id + 1U);
  EXPECT_NEAR(takeover.dynamic_objects_msg.objects.front().model_state[0], 2.0, 1e-6);
  EXPECT_NEAR(takeover.dynamic_objects_msg.objects.front().model_state[1], 0.0, 1e-6);
}

TEST(DynamicObjectTrackerTest, SelectedConfirmedTrackRemainsStickyWhileCoasting) {
  ros::NodeHandle pnh("~tracker_corridor_sticky_output");
  pnh.setParam("tracking_publish_corridor_provisional", true);
  pnh.setParam("tracking_corridor_provisional_min_hits", 1);
  pnh.setParam("tracking_corridor_realtime_max_publish_missed_frames", 5);
  pnh.setParam("tracking_max_corridor_realtime_tracks", 1);

  DynamicObjectTracker tracker(pnh);
  const auto old_target = tracker.processDynamicTracks(
      makeHeader(90.0),
      {makeDetectionAt(90.0, 1.0, 0.0, 0.6, true, false, 10U)});
  ASSERT_EQ(old_target.dynamic_objects_msg.objects.size(), 1U);

  const auto current_target = tracker.processDynamicTracks(
      makeHeader(90.1),
      {makeDetectionAt(90.1, 2.0, 0.0, 0.6, true, false, 20U)});
  ASSERT_EQ(current_target.dynamic_objects_msg.objects.size(), 1U);
  const std::uint32_t current_id = current_target.dynamic_objects_msg.objects.front().id;
  EXPECT_NE(current_id, old_target.dynamic_objects_msg.objects.front().id);

  // 两条确认轨迹都进入 coast 后，不能因旧轨迹 hits/ID 排序再次切回旧编号。
  const auto coasting = tracker.processDynamicTracks(makeHeader(90.2), {});
  ASSERT_EQ(coasting.dynamic_objects_msg.objects.size(), 1U);
  EXPECT_EQ(coasting.dynamic_objects_msg.objects.front().id, current_id);
  EXPECT_NEAR(coasting.dynamic_objects_msg.objects.front().model_state[0], 2.0, 1e-6);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_dynamic_object_tracker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
