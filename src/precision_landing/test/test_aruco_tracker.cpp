#include <gtest/gtest.h>

#include <limits>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "precision_landing/aruco_tracker.hpp"

namespace {

cv::Mat makeMarkerImage(int id, int center_x = 320, int center_y = 240) {
  cv::Mat marker;
  cv::aruco::drawMarker(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250),
      id, 200, marker);
  cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));
  marker.copyTo(image(cv::Rect(center_x - 100, center_y - 100, 200, 200)));
  return image;
}

cv::Mat cameraMatrix() {
  return (cv::Mat_<double>(3, 3) <<
      600.0, 0.0, 320.0,
      0.0, 600.0, 240.0,
      0.0, 0.0, 1.0);
}

cv::Mat distortion() {
  return cv::Mat::zeros(1, 5, CV_64F);
}

precision_landing::ArucoTrackerConfig testConfig() {
  precision_landing::ArucoTrackerConfig config;
  config.stable_frames = 5;
  return config;
}

void processFrames(precision_landing::ArucoTracker* tracker,
                   const cv::Mat& image,
                   int count,
                   int requested_id = -1,
                   double start_stamp_sec = 0.0) {
  const cv::Mat k = cameraMatrix();
  const cv::Mat d = distortion();
  for (int i = 0; i < count; ++i) {
    tracker->process(
        image, k, d, start_stamp_sec + 0.1 * static_cast<double>(i),
        requested_id);
  }
}

}  // namespace

TEST(ArucoTracker, OneConsistentFrameDoesNotLock) {
  precision_landing::ArucoTracker tracker(testConfig());

  processFrames(&tracker, makeMarkerImage(37), 1);

  EXPECT_EQ(tracker.lockedId(), -1);
}

TEST(ArucoTracker, FiveConsistentFramesLockMarkerId) {
  precision_landing::ArucoTracker tracker(testConfig());

  processFrames(&tracker, makeMarkerImage(37), 5);

  EXPECT_EQ(tracker.lockedId(), 37);
}

TEST(ArucoTracker, InvalidCameraMatrixBreaksConsecutiveLockAcquisition) {
  precision_landing::ArucoTracker tracker(testConfig());
  const cv::Mat image = makeMarkerImage(37);
  processFrames(&tracker, image, 4);

  tracker.process(image, cv::Mat::eye(2, 2, CV_64F), distortion(), 0.4, -1);
  tracker.process(image, cameraMatrix(), distortion(), 0.5, -1);

  EXPECT_EQ(tracker.lockedId(), -1);

  processFrames(&tracker, image, 4, -1, 0.6);
  EXPECT_EQ(tracker.lockedId(), 37);
}

TEST(ArucoTracker, RequestedIdRejectsDifferentMarker) {
  precision_landing::ArucoTracker tracker(testConfig());
  const cv::Mat image = makeMarkerImage(37);
  const cv::Mat k = cameraMatrix();
  const cv::Mat d = distortion();

  const precision_landing::TargetObservation observation =
      tracker.process(image, k, d, 0.0, 12);
  processFrames(&tracker, image, 4, 12, 0.1);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(tracker.lockedId(), -1);
}

TEST(ArucoTracker, ExcludedIdIsRejected) {
  precision_landing::ArucoTrackerConfig config = testConfig();
  config.stable_frames = 1;
  precision_landing::ArucoTracker tracker(config);

  const precision_landing::TargetObservation observation = tracker.process(
      makeMarkerImage(37), cameraMatrix(), distortion(), 0.0, -1, true, 37);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(tracker.lockedId(), -1);
}

TEST(ArucoTracker, LockedIdIsNotReplacedByAnotherVisibleMarker) {
  precision_landing::ArucoTracker tracker(testConfig());
  processFrames(&tracker, makeMarkerImage(37), 5);
  ASSERT_EQ(tracker.lockedId(), 37);

  const precision_landing::TargetObservation observation =
      tracker.process(makeMarkerImage(12), cameraMatrix(), distortion(), 1.0, -1);
  processFrames(&tracker, makeMarkerImage(12), 5, -1, 1.1);

  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(tracker.lockedId(), 37);
}

TEST(ArucoTracker, ResetClearsLockedId) {
  precision_landing::ArucoTracker tracker(testConfig());
  processFrames(&tracker, makeMarkerImage(37), 5);
  ASSERT_EQ(tracker.lockedId(), 37);

  tracker.reset();

  EXPECT_EQ(tracker.lockedId(), -1);
}

TEST(ArucoTracker, CenteredMarkerPoseIsOnPositiveCameraZAxis) {
  precision_landing::ArucoTracker tracker(testConfig());
  const cv::Mat image = makeMarkerImage(37);
  const cv::Mat k = cameraMatrix();
  const cv::Mat d = distortion();

  precision_landing::TargetObservation observation;
  for (int i = 0; i < 5; ++i) {
    observation = tracker.process(image, k, d, 0.1 * static_cast<double>(i), -1);
  }

  ASSERT_TRUE(observation.valid);
  EXPECT_NEAR(observation.position_camera.x(), 0.0, 0.01);
  EXPECT_NEAR(observation.position_camera.y(), 0.0, 0.01);
  EXPECT_GT(observation.position_camera.z(), 0.0);
}

TEST(ArucoTracker, DetectorOnlyFramesCannotBindMissionLock) {
  precision_landing::ArucoTrackerConfig config = testConfig();
  config.stable_frames = 2;
  precision_landing::ArucoTracker tracker(config);
  const cv::Mat image = makeMarkerImage(37);

  tracker.process(image, cameraMatrix(), distortion(), 0.0, -1, false);
  tracker.process(image, cameraMatrix(), distortion(), 0.1, -1, false);
  EXPECT_EQ(tracker.lockedId(), -1);

  tracker.process(image, cameraMatrix(), distortion(), 0.2, 37, true);
  tracker.process(image, cameraMatrix(), distortion(), 0.3, 37, true);
  EXPECT_EQ(tracker.lockedId(), 37);
}

TEST(ArucoTracker, ReacquisitionRebasesJumpGateForSameLockedId) {
  precision_landing::ArucoTrackerConfig config = testConfig();
  config.stable_frames = 1;
  config.max_position_jump_m = 0.02;
  precision_landing::ArucoTracker tracker(config);
  const cv::Mat centered = makeMarkerImage(37);
  const cv::Mat shifted = makeMarkerImage(37, 430, 240);

  ASSERT_TRUE(
      tracker.process(centered, cameraMatrix(), distortion(), 0.0, -1).valid);
  ASSERT_EQ(tracker.lockedId(), 37);
  EXPECT_FALSE(
      tracker.process(shifted, cameraMatrix(), distortion(), 0.1, -1).valid);

  tracker.beginReacquisition();
  EXPECT_TRUE(
      tracker.process(shifted, cameraMatrix(), distortion(), 0.2, -1).valid);
  EXPECT_FALSE(
      tracker.process(centered, cameraMatrix(), distortion(), 0.3, -1).valid);
}

TEST(ArucoTracker, RejectsInvalidOrExcessiveMarkerTilt) {
  const cv::Vec3d position(0.0, 0.0, 1.0);
  EXPECT_TRUE(precision_landing::markerTiltWithinLimit(
      cv::Vec3d(0.0, CV_PI / 4.0, 0.0), position, 50.0));
  EXPECT_FALSE(precision_landing::markerTiltWithinLimit(
      cv::Vec3d(0.0, 70.0 * CV_PI / 180.0, 0.0), position, 50.0));
  EXPECT_FALSE(precision_landing::markerTiltWithinLimit(
      cv::Vec3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      position, 50.0));
}
