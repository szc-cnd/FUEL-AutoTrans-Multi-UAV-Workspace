#include <cassert>

#include "exploration_manager/path_shortening_policy.h"

int main() {
  using fast_planner::path_shortening::preserveWaypointForClearance;

  // A shortcut that gives up more than 5 cm of clearance must keep the A* turn.
  assert(preserveWaypointForClearance(0.35, 0.29, 0.05));

  // Small map/sampling fluctuations must not stop ordinary path shortening.
  assert(!preserveWaypointForClearance(0.35, 0.30, 0.05));
  assert(!preserveWaypointForClearance(0.18, 0.16, 0.05));

  // A shortcut with equal or better clearance is always allowed.
  assert(!preserveWaypointForClearance(0.20, 0.20, 0.05));
  assert(!preserveWaypointForClearance(0.20, 0.28, 0.05));
  return 0;
}
