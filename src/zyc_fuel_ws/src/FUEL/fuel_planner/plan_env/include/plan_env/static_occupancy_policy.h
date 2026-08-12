#pragma once

#include <algorithm>

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

}  // namespace occupancy_policy
}  // namespace fast_planner
