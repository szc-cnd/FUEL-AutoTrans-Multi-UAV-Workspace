#include <gtest/gtest.h>

#include "precision_landing/landing_controller.hpp"

using precision_landing::ControllerConfig;
using precision_landing::LandingController;

TEST(LandingController, RotatesForwardBodyVelocityIntoEnuAtNinetyDegreeYaw) {
  ControllerConfig cfg;
  cfg.kp_xy = 1.0;
  cfg.max_xy_speed = 2.0;
  cfg.max_xy_accel = 100.0;
  cfg.error_deadband = 0.0;
  cfg.filter_alpha = 1.0;
  LandingController controller(cfg);
  const auto cmd = controller.compute(Eigen::Vector2d(1.0, 0.0), M_PI_2, 0.0, 0.1);
  EXPECT_NEAR(cmd.velocity_enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(cmd.velocity_enu.y(), 1.0, 1e-6);
}

TEST(LandingController, AppliesDeadbandAndDescentVelocity) {
  ControllerConfig cfg;
  cfg.error_deadband = 0.05;
  cfg.filter_alpha = 1.0;
  LandingController controller(cfg);
  const auto cmd = controller.compute(Eigen::Vector2d(0.02, -0.03), 0.0, 0.12, 0.05);
  EXPECT_NEAR(cmd.velocity_enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(cmd.velocity_enu.y(), 0.0, 1e-6);
  EXPECT_NEAR(cmd.velocity_enu.z(), -0.12, 1e-6);
}

TEST(LandingController, LimitsHorizontalSpeedAndSlewRate) {
  ControllerConfig cfg;
  cfg.kp_xy = 10.0;
  cfg.max_xy_speed = 0.6;
  cfg.max_xy_accel = 1.0;
  cfg.error_deadband = 0.0;
  cfg.filter_alpha = 1.0;
  LandingController controller(cfg);
  const auto first = controller.compute(Eigen::Vector2d(2.0, 0.0), 0.0, 0.0, 0.1);
  EXPECT_NEAR(first.velocity_enu.head<2>().norm(), 0.1, 1e-6);
  const auto second = controller.compute(Eigen::Vector2d(2.0, 0.0), 0.0, 0.0, 1.0);
  EXPECT_LE(second.velocity_enu.head<2>().norm(), 0.6 + 1e-6);
}

TEST(LandingController, ResetClearsFilterAndPublishedVelocityHistory) {
  ControllerConfig cfg;
  cfg.kp_xy = 10.0;
  cfg.max_xy_speed = 2.0;
  cfg.max_xy_accel = 1.0;
  cfg.error_deadband = 0.0;
  cfg.filter_alpha = 0.25;
  LandingController controller(cfg);
  const auto before_reset =
      controller.compute(Eigen::Vector2d(1.0, 0.0), 0.0, 0.0, 0.1);
  ASSERT_NEAR(before_reset.velocity_enu.x(), 0.1, 1e-6);

  controller.reset();
  const auto after_reset =
      controller.compute(Eigen::Vector2d(-1.0, 0.0), 0.0, 0.0, 0.1);

  EXPECT_NEAR(after_reset.velocity_enu.x(), -0.1, 1e-6);
  EXPECT_NEAR(after_reset.velocity_enu.y(), 0.0, 1e-6);
}
