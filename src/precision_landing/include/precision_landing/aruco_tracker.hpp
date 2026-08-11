#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "precision_landing/types.hpp"

namespace precision_landing {

struct ArucoTrackerConfig {
  double marker_size_m{0.60};
  double max_reprojection_error_px{4.0};
  double min_distance_m{0.15};
  double max_distance_m{6.0};
  double max_position_jump_m{0.60};
  double max_tilt_deg{60.0};
  int stable_frames{5};
};

bool markerTiltWithinLimit(const cv::Vec3d& rvec,
                           const cv::Vec3d& tvec,
                           double max_tilt_deg);

class ArucoTracker {
 public:
  explicit ArucoTracker(const ArucoTrackerConfig& config);

  TargetObservation process(const cv::Mat& image,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& distortion,
                            double stamp_sec,
                            int requested_id,
                            bool allow_lock_mutation = true);

  int lockedId() const;
  void reset();
  void beginReacquisition();
  const cv::Mat& debugImage() const;

 private:
  void breakPendingAcquisition();
  void drawLockState();

  ArucoTrackerConfig config_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  int locked_id_{-1};
  int pending_id_{-1};
  int pending_count_{0};
  int last_position_id_{-1};
  bool has_last_position_{false};
  Eigen::Vector3d last_position_camera_{Eigen::Vector3d::Zero()};
  cv::Mat debug_image_;
};

}  // namespace precision_landing
