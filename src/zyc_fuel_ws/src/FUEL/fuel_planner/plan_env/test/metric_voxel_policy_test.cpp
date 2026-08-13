#include <cassert>
#include <cmath>

#include "plan_env/metric_voxel_policy.h"

int main() {
  using fast_planner::metric_voxel::hasMinimumSupport;
  using fast_planner::metric_voxel::radiusInVoxels;

  assert(radiusInVoxels(0.10, 0.10) == 1);
  assert(radiusInVoxels(0.10, 0.05) == 2);
  assert(radiusInVoxels(0.11, 0.05) == 3);
  assert(radiusInVoxels(0.0, 0.05) == 0);
  assert(radiusInVoxels(0.10, 0.0) == 0);

  const auto support_two_cells_away = [](int dx, int dy, int dz) {
    return (dx == 0 && dy == 0 && dz == 0) ||
           (dx == 2 && dy == 0 && dz == 0);
  };
  assert(!hasMinimumSupport(1, 2, true, support_two_cells_away));
  assert(hasMinimumSupport(2, 2, true, support_two_cells_away));

  const auto center_only = [](int dx, int dy, int dz) {
    return dx == 0 && dy == 0 && dz == 0;
  };
  assert(hasMinimumSupport(1, 1, true, center_only));
  assert(!hasMinimumSupport(1, 1, false, center_only));
  return 0;
}
