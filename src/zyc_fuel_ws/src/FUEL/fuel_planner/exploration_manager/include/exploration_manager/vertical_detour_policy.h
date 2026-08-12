#pragma once

#include <algorithm>

namespace fast_planner {
namespace vertical_detour {

enum class LowProbePhase { IDLE, DESCENDING, VERIFYING, ADVANCING, ASCENDING };

inline bool preferDescending(double current_height, double split_height) {
  return current_height > split_height;
}

inline LowProbePhase advanceLowProbe(LowProbePhase phase, bool at_probe_height,
                                     bool xy_stable, bool corridor_safe,
                                     bool verification_timed_out,
                                     bool advance_complete, bool ascent_complete,
                                     int required_confirmations,
                                     int& confirmations) {
  required_confirmations = std::max(1, required_confirmations);
  switch (phase) {
    case LowProbePhase::DESCENDING:
      if (at_probe_height && xy_stable) {
        confirmations = 0;
        return LowProbePhase::VERIFYING;
      }
      return phase;
    case LowProbePhase::VERIFYING:
      confirmations = corridor_safe ? confirmations + 1 : 0;
      // 安全确认与超时恰好落在同一周期时优先前进，避免能通过却回升。
      if (confirmations >= required_confirmations) return LowProbePhase::ADVANCING;
      if (verification_timed_out) {
        confirmations = 0;
        return LowProbePhase::ASCENDING;
      }
      return phase;
    case LowProbePhase::ADVANCING:
      if (advance_complete) {
        confirmations = 0;
        return LowProbePhase::IDLE;
      }
      return phase;
    case LowProbePhase::ASCENDING:
      if (ascent_complete) {
        confirmations = 0;
        return LowProbePhase::IDLE;
      }
      return phase;
    case LowProbePhase::IDLE:
    default:
      confirmations = 0;
      return LowProbePhase::IDLE;
  }
}

}  // namespace vertical_detour
}  // namespace fast_planner
