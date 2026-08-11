#pragma once

#include <chrono>
#include <cstdint>

namespace precision_landing {

using ModeRequestClock = std::chrono::steady_clock;

struct ModeRequestAuthorization {
  bool authorized{false};
  bool state_snapshot_fresh{false};
  bool state_connected{false};
  bool state_armed{false};
  bool state_offboard{false};
  std::uint64_t generation{0U};
  ModeRequestClock::time_point expiry{
      ModeRequestClock::time_point::min()};

  bool validAt(std::uint64_t expected_generation,
               ModeRequestClock::time_point call_start) const noexcept {
    return authorized && state_snapshot_fresh && state_connected &&
           state_armed && state_offboard &&
           generation == expected_generation && call_start < expiry;
  }
};

}  // namespace precision_landing
