#pragma once

#include <algorithm>
#include <cstdint>

namespace fast_planner {
namespace occupancy_policy {

class StaticEvidence {
public:
  explicit StaticEvidence(int required_hits)
      : required_hits_(std::max(1, required_hits)) {}

  void observe(bool hit) {
    if (locked_) return;
    evidence_ = hit ? evidence_ + 1 : std::max(0, evidence_ - 1);
    locked_ = evidence_ >= required_hits_;
  }

  bool locked() const { return locked_; }
  bool applyMiss() const { return !locked_; }

private:
  int required_hits_{1};
  int evidence_{0};
  bool locked_{false};
};

inline unsigned short updateEvidence(unsigned short evidence, bool hit,
                                     unsigned short required_hits) {
  if (evidence >= required_hits) return required_hits;
  if (hit) return std::min<unsigned short>(required_hits, evidence + 1);
  return evidence > 0 ? evidence - 1 : 0;
}

enum class RetentionState : uint8_t {
  NONE = 0,
  TEMPORARY = 1,
  STATIC = 2,
};

struct StructuredRetentionConfig {
  unsigned short temporary_hits{6};
  unsigned short static_hits{6};
  unsigned short single_view_static_hits{10};
  unsigned short release_misses{12};
};

struct StructuredRetentionStatus {
  RetentionState state{RetentionState::NONE};
  unsigned short hit_evidence{0};
  unsigned short free_evidence{0};
};

struct StructuredRetentionUpdate {
  StructuredRetentionStatus status;
  bool apply_observation{true};
  bool clear_occupancy{false};
};

inline StructuredRetentionUpdate updateStructuredRetention(
    StructuredRetentionStatus status, bool hit, bool has_structure,
    bool hit_multiview_confirmed, bool within_near_field,
    bool free_multiview_confirmed, const StructuredRetentionConfig& config) {
  StructuredRetentionUpdate update;
  update.status = status;
  const unsigned short temporary_hits = std::max<unsigned short>(1, config.temporary_hits);
  const unsigned short static_hits =
      std::max<unsigned short>(temporary_hits, config.static_hits);
  const unsigned short single_view_hits =
      std::max<unsigned short>(static_hits, config.single_view_static_hits);
  const unsigned short release_misses = std::max<unsigned short>(1, config.release_misses);

  if (hit) {
    update.status.hit_evidence =
        std::min<unsigned short>(single_view_hits, update.status.hit_evidence + 1);
    update.status.free_evidence = 0;
    if (has_structure && update.status.hit_evidence >= temporary_hits &&
        update.status.state == RetentionState::NONE) {
      update.status.state = RetentionState::TEMPORARY;
    }
    const unsigned short confirmation_hits =
        hit_multiview_confirmed ? static_hits : single_view_hits;
    if (has_structure && update.status.hit_evidence >= confirmation_hits)
      update.status.state = RetentionState::STATIC;
    return update;
  }

  if (update.status.state == RetentionState::STATIC && within_near_field) {
    // 雷达盲区内的自由射线不足以解除已确认静态障碍；临时障碍仍按miss衰减。
    update.apply_observation = false;
    return update;
  }

  if (update.status.state == RetentionState::STATIC) {
    update.status.free_evidence =
        std::min<unsigned short>(release_misses, update.status.free_evidence + 1);
    update.apply_observation = false;
    if (update.status.free_evidence >= release_misses && free_multiview_confirmed) {
      update.status = StructuredRetentionStatus();
      update.clear_occupancy = true;
    }
    return update;
  }

  update.status.hit_evidence =
      update.status.hit_evidence > 0 ? update.status.hit_evidence - 1 : 0;
  update.status.free_evidence = 0;
  if (update.status.state == RetentionState::TEMPORARY &&
      update.status.hit_evidence < temporary_hits) {
    update.status.state = RetentionState::NONE;
  }
  return update;
}

}  // namespace occupancy_policy
}  // namespace fast_planner
