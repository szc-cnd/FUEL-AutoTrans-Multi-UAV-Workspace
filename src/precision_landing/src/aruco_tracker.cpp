#include "precision_landing/aruco_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace precision_landing {
namespace {

struct Candidate {
  int id{-1};
  std::size_t detection_index{0U};
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double reprojection_error_px{0.0};
  double score{0.0};
};

std::vector<cv::Point3f> markerObjectPoints(double marker_size_m) {
  const float half_size = static_cast<float>(marker_size_m * 0.5);
  std::vector<cv::Point3f> points;
  points.reserve(4U);
  points.emplace_back(-half_size, half_size, 0.0F);
  points.emplace_back(half_size, half_size, 0.0F);
  points.emplace_back(half_size, -half_size, 0.0F);
  points.emplace_back(-half_size, -half_size, 0.0F);
  return points;
}

double reprojectionRms(const std::vector<cv::Point2f>& detected_corners,
                       const cv::Vec3d& rvec,
                       const cv::Vec3d& tvec,
                       double marker_size_m,
                       const cv::Mat& camera_matrix,
                       const cv::Mat& distortion) {
  std::vector<cv::Point2f> projected_corners;
  cv::projectPoints(markerObjectPoints(marker_size_m), rvec, tvec,
                    camera_matrix, distortion, projected_corners);

  double squared_error = 0.0;
  for (std::size_t i = 0U; i < detected_corners.size(); ++i) {
    const cv::Point2f difference = detected_corners[i] - projected_corners[i];
    squared_error += static_cast<double>(difference.dot(difference));
  }
  return std::sqrt(squared_error / static_cast<double>(detected_corners.size()));
}

double selectionScore(const std::vector<cv::Point2f>& corners,
                      const cv::Size& image_size) {
  cv::Point2f center(0.0F, 0.0F);
  for (std::size_t i = 0U; i < corners.size(); ++i) {
    center += corners[i];
  }
  center *= 1.0F / static_cast<float>(corners.size());

  const cv::Point2f image_center(
      0.5F * static_cast<float>(image_size.width),
      0.5F * static_cast<float>(image_size.height));
  const double center_distance_px = cv::norm(center - image_center);
  const double image_diagonal_px =
      std::hypot(static_cast<double>(image_size.width),
                 static_cast<double>(image_size.height));
  const double marker_area_px = std::abs(cv::contourArea(corners));
  const double image_area_px =
      static_cast<double>(image_size.width) * static_cast<double>(image_size.height);
  return center_distance_px / image_diagonal_px -
         0.25 * std::sqrt(marker_area_px / image_area_px);
}

Eigen::Vector3d eigenPosition(const cv::Vec3d& tvec) {
  return Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
}

std::string fixedValue(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

bool finiteVector(const cv::Vec3d& vector) {
  return std::isfinite(vector[0]) && std::isfinite(vector[1]) &&
         std::isfinite(vector[2]);
}

}  // namespace

bool markerTiltWithinLimit(const cv::Vec3d& rvec,
                           const cv::Vec3d& tvec,
                           double max_tilt_deg) {
  if (!finiteVector(rvec) || !finiteVector(tvec) ||
      !std::isfinite(max_tilt_deg) || max_tilt_deg < 0.0 ||
      max_tilt_deg > 90.0) {
    return false;
  }

  const double position_norm = cv::norm(tvec);
  if (!std::isfinite(position_norm) ||
      position_norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  cv::Matx33d marker_to_camera;
  cv::Rodrigues(rvec, marker_to_camera);
  const cv::Vec3d marker_normal_camera =
      marker_to_camera * cv::Vec3d(0.0, 0.0, 1.0);
  if (!finiteVector(marker_normal_camera)) {
    return false;
  }

  const cv::Vec3d camera_direction = -tvec / position_norm;
  const double normal_norm = cv::norm(marker_normal_camera);
  if (!std::isfinite(normal_norm) ||
      normal_norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double cosine =
      std::max(0.0, std::min(
          1.0, std::abs(marker_normal_camera.dot(camera_direction)) /
                   normal_norm));
  const double tilt_deg = std::acos(cosine) * 180.0 / CV_PI;
  return std::isfinite(tilt_deg) && tilt_deg <= max_tilt_deg;
}

ArucoTracker::ArucoTracker(const ArucoTrackerConfig& config)
    : config_(config),
      dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250)) {}

TargetObservation ArucoTracker::process(const cv::Mat& image,
                                        const cv::Mat& camera_matrix,
                                        const cv::Mat& distortion,
                                        double stamp_sec,
                                        int requested_id,
                                        bool allow_lock_mutation,
                                        int excluded_id) {
  TargetObservation observation;
  observation.stamp_sec = stamp_sec;
  detected_targets_.clear();

  if (image.empty()) {
    debug_image_.release();
    breakPendingAcquisition();
    return observation;
  }

  cv::Mat detection_image;
  if (image.channels() == 1) {
    detection_image = image;
    cv::cvtColor(image, debug_image_, cv::COLOR_GRAY2BGR);
  } else if (image.channels() == 3) {
    cv::cvtColor(image, detection_image, cv::COLOR_BGR2GRAY);
    debug_image_ = image.clone();
  } else if (image.channels() == 4) {
    cv::cvtColor(image, detection_image, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(image, debug_image_, cv::COLOR_BGRA2BGR);
  } else {
    image.copyTo(debug_image_);
    breakPendingAcquisition();
    drawLockState();
    return observation;
  }

  if (camera_matrix.rows != 3 || camera_matrix.cols != 3) {
    breakPendingAcquisition();
    drawLockState();
    return observation;
  }

  cv::Mat camera_matrix_64f;
  cv::Mat distortion_64f;
  camera_matrix.convertTo(camera_matrix_64f, CV_64F);
  if (!distortion.empty()) {
    distortion.convertTo(distortion_64f, CV_64F);
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(detection_image, dictionary_, corners, ids);
  if (!ids.empty()) {
    cv::aruco::drawDetectedMarkers(debug_image_, corners, ids);
  }

  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  if (!ids.empty()) {
    cv::aruco::estimatePoseSingleMarkers(
        corners, static_cast<float>(config_.marker_size_m),
        camera_matrix_64f, distortion_64f, rvecs, tvecs);
  }

  Candidate selected;
  bool has_selected = false;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < ids.size(); ++i) {
    if (i >= rvecs.size() || i >= tvecs.size()) {
      continue;
    }
    const int id = ids[i];
    if (id == excluded_id ||
        (requested_id >= 0 && id != requested_id)) {
      continue;
    }

    const Eigen::Vector3d position_camera = eigenPosition(tvecs[i]);
    if (!finiteVector(tvecs[i]) ||
        !markerTiltWithinLimit(rvecs[i], tvecs[i],
                              config_.max_tilt_deg)) {
      continue;
    }
    const double distance_m = position_camera.norm();
    if (distance_m < config_.min_distance_m ||
        distance_m > config_.max_distance_m) {
      continue;
    }

    const double reprojection_error_px =
        reprojectionRms(corners[i], rvecs[i], tvecs[i], config_.marker_size_m,
                        camera_matrix_64f, distortion_64f);
    if (!std::isfinite(reprojection_error_px) ||
        reprojection_error_px > config_.max_reprojection_error_px) {
      continue;
    }

    if (has_last_position_ && last_position_id_ == id &&
        (position_camera - last_position_camera_).norm() >
            config_.max_position_jump_m) {
      continue;
    }

    detected_targets_.push_back(
        DetectedTarget{id, position_camera, selectionScore(corners[i], image.size())});

    // Keep publishing other stable candidates for the dual-platform
    // coordinator, while the mission lock itself remains on one ID.
    if (locked_id_ >= 0 && id != locked_id_) {
      continue;
    }

    const double score = selectionScore(corners[i], image.size());
    const bool better_score = !has_selected || score < best_score;
    const bool deterministic_tie =
        has_selected && score == best_score &&
        (id < selected.id ||
         (id == selected.id && i < selected.detection_index));
    if (better_score || deterministic_tie) {
      selected.id = id;
      selected.detection_index = i;
      selected.rvec = rvecs[i];
      selected.tvec = tvecs[i];
      selected.reprojection_error_px = reprojection_error_px;
      selected.score = score;
      best_score = score;
      has_selected = true;
    }
  }

  if (!has_selected) {
    breakPendingAcquisition();
    drawLockState();
    return observation;
  }

  const Eigen::Vector3d selected_position = eigenPosition(selected.tvec);
  if (allow_lock_mutation) {
    last_position_camera_ = selected_position;
    last_position_id_ = selected.id;
    has_last_position_ = true;

    if (locked_id_ < 0) {
      if (pending_id_ == selected.id) {
        ++pending_count_;
      } else {
        pending_id_ = selected.id;
        pending_count_ = 1;
      }
      if (pending_count_ >= std::max(1, config_.stable_frames)) {
        locked_id_ = selected.id;
      }
    }
  }

  observation.valid = true;
  observation.id = selected.id;
  observation.position_camera = selected_position;
  const std::vector<cv::Point2f>& selected_corners =
      corners[selected.detection_index];
  if (!selected_corners.empty()) {
    cv::Point2f center(0.0F, 0.0F);
    for (const cv::Point2f& corner : selected_corners) {
      center += corner;
    }
    center *= 1.0F / static_cast<float>(selected_corners.size());
    observation.image_center_px = Eigen::Vector2d(center.x, center.y);
  }
  observation.reprojection_error_px = selected.reprojection_error_px;

  // 位姿文字固定在左上角，不能跟随码角点；否则码靠近画面右侧时文字会被裁掉。
  const std::string pose_text =
      "ID " + std::to_string(selected.id) +
      "  XYZ[m] " + fixedValue(selected_position.x()) + " " +
      fixedValue(selected_position.y()) + " " +
      fixedValue(selected_position.z());
  const std::string quality_text =
      "Reprojection error: " + fixedValue(selected.reprojection_error_px) +
      " px";
  cv::putText(debug_image_, pose_text, cv::Point(10, 45),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1,
              cv::LINE_AA);
  cv::putText(debug_image_, quality_text, cv::Point(10, 65),
              cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1,
              cv::LINE_AA);
  drawLockState();
  return observation;
}

int ArucoTracker::lockedId() const {
  return locked_id_;
}

void ArucoTracker::reset() {
  locked_id_ = -1;
  pending_id_ = -1;
  pending_count_ = 0;
  last_position_id_ = -1;
  has_last_position_ = false;
  last_position_camera_.setZero();
}

void ArucoTracker::beginReacquisition() {
  last_position_id_ = -1;
  has_last_position_ = false;
  last_position_camera_.setZero();
}

const cv::Mat& ArucoTracker::debugImage() const {
  return debug_image_;
}

const std::vector<DetectedTarget>& ArucoTracker::detectedTargets() const {
  return detected_targets_;
}

void ArucoTracker::breakPendingAcquisition() {
  if (locked_id_ < 0) {
    pending_id_ = -1;
    pending_count_ = 0;
  }
}

void ArucoTracker::drawLockState() {
  if (debug_image_.empty()) {
    return;
  }

  const std::string lock_text =
      locked_id_ >= 0
          ? "LOCKED ID " + std::to_string(locked_id_)
          : "UNLOCKED candidate " + std::to_string(pending_id_) +
                " " + std::to_string(pending_count_) + "/" +
                std::to_string(std::max(1, config_.stable_frames));
  cv::putText(debug_image_, lock_text, cv::Point(10, 22),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1,
              cv::LINE_AA);
}

}  // namespace precision_landing
