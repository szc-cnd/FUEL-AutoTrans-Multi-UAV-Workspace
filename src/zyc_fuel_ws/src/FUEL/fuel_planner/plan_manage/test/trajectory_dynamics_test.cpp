#include <cassert>

#include <bspline/non_uniform_bspline.h>

int main() {
  Eigen::MatrixXd control_points(10, 3);
  control_points.setZero();
  control_points.row(3) << 0.30, 0.15, 0.0;
  control_points.row(4) << 0.60, 0.30, 0.0;
  control_points.row(5) << 0.30, -0.15, 0.0;
  control_points.row(6) << 0.00, 0.00, 0.0;

  fast_planner::NonUniformBspline trajectory(control_points, 3, 0.10);
  const Eigen::Vector3d start_position = trajectory.evaluateDeBoorT(0.0);
  const Eigen::Vector3d start_velocity =
      trajectory.getDerivative().evaluateDeBoorT(0.0);
  const Eigen::Vector3d start_acceleration =
      trajectory.getDerivative().getDerivative().evaluateDeBoorT(0.0);
  const double initial_duration = trajectory.getTimeSum();

  trajectory.setPhysicalLimits(0.20, 0.35);
  assert(!trajectory.checkFeasibility(false));
  int iterations = 0;
  while (!trajectory.checkFeasibility(false) && iterations < 30) {
    trajectory.reallocateTime(false);
    ++iterations;
  }

  assert(trajectory.checkFeasibility(false));
  assert(trajectory.getTimeSum() > initial_duration);
  assert((trajectory.evaluateDeBoorT(0.0) - start_position).norm() < 1e-9);
  assert((trajectory.getDerivative().evaluateDeBoorT(0.0) - start_velocity).norm() < 1e-9);
  assert((trajectory.getDerivative().getDerivative().evaluateDeBoorT(0.0) -
          start_acceleration).norm() < 1e-9);
  return 0;
}
