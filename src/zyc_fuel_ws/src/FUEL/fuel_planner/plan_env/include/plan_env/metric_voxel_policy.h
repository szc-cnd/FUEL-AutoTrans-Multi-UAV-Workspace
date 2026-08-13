#pragma once

#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace metric_voxel {

inline int radiusInVoxels(double radius_m, double resolution_m) {
  if (radius_m <= 0.0 || resolution_m <= 0.0) return 0;
  return std::max(1, static_cast<int>(std::ceil(radius_m / resolution_m - 1e-9)));
}

template <typename IsOccupied>
bool hasMinimumSupport(int radius_voxels, int minimum_support,
                       bool include_center, IsOccupied is_occupied) {
  if (minimum_support <= 0) return true;
  radius_voxels = std::max(0, radius_voxels);
  int support = 0;
  for (int dx = -radius_voxels; dx <= radius_voxels; ++dx) {
    for (int dy = -radius_voxels; dy <= radius_voxels; ++dy) {
      for (int dz = -radius_voxels; dz <= radius_voxels; ++dz) {
        if (!include_center && dx == 0 && dy == 0 && dz == 0) continue;
        if (is_occupied(dx, dy, dz) && ++support >= minimum_support) return true;
      }
    }
  }
  return false;
}

}  // namespace metric_voxel
}  // namespace fast_planner
