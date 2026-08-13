#pragma once

#include <algorithm>

namespace fast_planner {
namespace vertical_detour {

enum class LowProbePhase {
  IDLE,
  DESCENDING,
  VERIFYING,
  ADVANCING,
  EXIT_VERIFYING,
  ASCENDING
};

struct LowProbeObservation {
  bool at_probe_height{false};
  bool xy_stable{false};
  bool low_corridor_safe{false};
  bool verification_timed_out{false};
  bool advance_complete{false};
  bool ascent_complete{false};
  bool ascent_path_safe{false};
  bool release_corridor_safe{false};
};

inline bool preferDescending(double current_height, double split_height) {
  return current_height > split_height;
}

inline bool inflationEscapeAllowed(LowProbePhase phase) {
  return phase == LowProbePhase::IDLE;
}

inline bool ascentShouldAbort(double xy_error, double xy_tolerance,
                              double elapsed, double timeout) {
  return xy_error > std::max(0.0, xy_tolerance) ||
         elapsed >= std::max(0.1, timeout);
}

inline LowProbePhase advanceLowProbe(LowProbePhase phase,
                                     const LowProbeObservation& observation,
                                     int required_confirmations,
                                     int required_release_confirmations,
                                     int& confirmations,
                                     int& release_confirmations) {
  required_confirmations = std::max(1, required_confirmations);
  required_release_confirmations = std::max(1, required_release_confirmations);
  switch (phase) {
    case LowProbePhase::DESCENDING:
      if (observation.at_probe_height && observation.xy_stable) {
        confirmations = 0;
        release_confirmations = 0;
        return LowProbePhase::VERIFYING;
      }
      return phase;
    case LowProbePhase::VERIFYING:
      confirmations = observation.low_corridor_safe ? confirmations + 1 : 0;
      // 安全确认与超时恰好落在同一周期时优先前进，避免能通过却回升。
      if (confirmations >= required_confirmations) return LowProbePhase::ADVANCING;
      // 已经位于障碍物下方时，只有垂直返回柱仍然安全才允许超时回升。
      if (observation.verification_timed_out && observation.ascent_path_safe) {
        confirmations = 0;
        release_confirmations = 0;
        return LowProbePhase::ASCENDING;
      }
      return phase;
    case LowProbePhase::ADVANCING:
      if (observation.advance_complete) {
        confirmations = 0;
        if (observation.release_corridor_safe) {
          release_confirmations = 1;
          return required_release_confirmations <= 1 ? LowProbePhase::ASCENDING
                                                     : LowProbePhase::EXIT_VERIFYING;
        }
        release_confirmations = 0;
        return LowProbePhase::VERIFYING;
      }
      return phase;
    case LowProbePhase::EXIT_VERIFYING:
      if (observation.release_corridor_safe) {
        ++release_confirmations;
        if (release_confirmations >= required_release_confirmations)
          return LowProbePhase::ASCENDING;
        return phase;
      }
      release_confirmations = 0;
      return LowProbePhase::VERIFYING;
    case LowProbePhase::ASCENDING:
      if (observation.ascent_complete) {
        confirmations = 0;
        release_confirmations = 0;
        return LowProbePhase::IDLE;
      }
      return phase;
    case LowProbePhase::IDLE:
    default:
      confirmations = 0;
      release_confirmations = 0;
      return LowProbePhase::IDLE;
  }
}

inline bool preferPathByMinimumClearance(double candidate_minimum_clearance,
                                         double candidate_endpoint_clearance,
                                         double best_minimum_clearance,
                                         double best_endpoint_clearance,
                                         double candidate_distance,
                                         double best_distance,
                                         double candidate_alignment,
                                         double best_alignment) {
  if (candidate_minimum_clearance > best_minimum_clearance + 0.01) return true;
  if (candidate_minimum_clearance + 0.01 < best_minimum_clearance) return false;
  if (candidate_endpoint_clearance > best_endpoint_clearance + 0.01) return true;
  if (candidate_endpoint_clearance + 0.01 < best_endpoint_clearance) return false;
  if (candidate_distance + 1e-3 < best_distance) return true;
  if (candidate_distance > best_distance + 1e-3) return false;
  return candidate_alignment > best_alignment;
}

}  // namespace vertical_detour
}  // namespace fast_planner
