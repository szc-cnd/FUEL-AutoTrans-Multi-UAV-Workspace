#include <gtest/gtest.h>

#include "precision_landing/landing_state_machine.hpp"

using precision_landing::LandingState;
using precision_landing::LandingStateMachine;
using precision_landing::StateInput;
using precision_landing::StateMachineConfig;

namespace {

StateInput ActiveInput(double now_sec, double marker_height_m = 2.0,
                       double horizontal_error_m = 0.05, bool target_visible = true) {
  return StateInput{true, true, true, target_visible, false, horizontal_error_m,
                    marker_height_m, now_sec};
}

void ReachHighDescent(LandingStateMachine& machine) {
  machine.update(ActiveInput(0.0));
  machine.update(ActiveInput(0.0));
  machine.update(ActiveInput(0.30));
  machine.update(ActiveInput(0.80));
}

void ReachDescent(LandingStateMachine& machine, double marker_height_m) {
  machine.update(ActiveInput(0.0, marker_height_m));
  machine.update(ActiveInput(0.0, marker_height_m));
  machine.update(ActiveInput(0.30, marker_height_m));
  machine.update(ActiveInput(0.80, marker_height_m));
}

void ReachFinalDescent(LandingStateMachine& machine) {
  ReachDescent(machine, 0.30);
}

}  // namespace

TEST(StateInput, SupportsDefaultAndEightArgumentConstruction) {
  const StateInput default_input;
  EXPECT_FALSE(default_input.trigger);
  EXPECT_TRUE(default_input.critical_inputs_fresh);
  EXPECT_DOUBLE_EQ(default_input.now_sec, 0.0);

  const StateInput explicit_input{true, true, true, true, true, 0.1, 0.2, 0.3};
  EXPECT_TRUE(explicit_input.auto_land_active);
  EXPECT_DOUBLE_EQ(explicit_input.now_sec, 0.3);
}

TEST(LandingStateMachine, NeverDescendsWhenPrecheckFails) {
  LandingStateMachine machine;
  machine.update(StateInput{true, false, true, true, false, 0.0, 2.0, 0.0});

  const auto output = machine.update(
      StateInput{true, false, true, true, false, 0.0, 2.0, 1.0});

  EXPECT_EQ(output.state, LandingState::PRECHECK);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, RequiresStableAcquireAndAlignBeforeDescending) {
  LandingStateMachine machine;
  const StateInput ready{true, true, true, true, false, 0.10, 2.0, 0.0};
  machine.update(ready);
  machine.update(ready);

  auto input = ready;
  input.now_sec = 0.29;
  EXPECT_EQ(machine.update(input).state, LandingState::ACQUIRE);
  input.now_sec = 0.30;
  EXPECT_EQ(machine.update(input).state, LandingState::ALIGN);
  input.now_sec = 0.79;
  EXPECT_DOUBLE_EQ(machine.update(input).descent_speed_mps, 0.0);
  input.now_sec = 0.80;
  const auto output = machine.update(input);
  EXPECT_EQ(output.state, LandingState::DESCEND_HIGH);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.25);
}

TEST(LandingStateMachine, StopsDescendingAfterHighAltitudeTargetLoss) {
  LandingStateMachine machine;
  ReachHighDescent(machine);

  const auto first_lost = machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  EXPECT_EQ(first_lost.state, LandingState::DESCEND_HIGH);
  EXPECT_DOUBLE_EQ(first_lost.descent_speed_mps, 0.0);
  const auto output = machine.update(ActiveInput(1.41, 2.0, 0.05, false));

  EXPECT_EQ(output.state, LandingState::REACQUIRE);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, RequestsAutoLandAfterConfirmedNearGroundLossDuringDescent) {
  LandingStateMachine machine;
  ReachFinalDescent(machine);

  const auto first_lost =
      machine.update(ActiveInput(0.90, 0.30, 0.05, false));
  EXPECT_EQ(first_lost.state, LandingState::DESCEND_FINAL);
  EXPECT_DOUBLE_EQ(first_lost.descent_speed_mps, 0.0);
  EXPECT_FALSE(first_lost.request_auto_land);

  const auto output = machine.update(
      ActiveInput(1.41, 0.30, 0.05, false));

  EXPECT_EQ(output.state, LandingState::REQUEST_AUTO_LAND);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_TRUE(output.request_auto_land);
}

TEST(LandingStateMachine, RequestsAutoLandWhenLossHeightEqualsNearGroundThreshold) {
  StateMachineConfig config;
  config.near_ground_loss_height_m = 1.20;
  LandingStateMachine machine(config);
  ReachHighDescent(machine);

  machine.update(ActiveInput(0.90, 1.20, 0.05, false));
  const auto output = machine.update(ActiveInput(1.41, 1.20, 0.05, false));

  EXPECT_EQ(output.state, LandingState::REQUEST_AUTO_LAND);
  EXPECT_TRUE(output.request_auto_land);
}

TEST(LandingStateMachine, RequestsAutoLandAfterConfirmedNearGroundLossDuringAlign) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0, 0.30));
  machine.update(ActiveInput(0.0, 0.30));
  ASSERT_EQ(machine.update(ActiveInput(0.30, 0.30)).state,
            LandingState::ALIGN);

  const auto first_lost =
      machine.update(ActiveInput(0.31, 0.30, 0.05, false));
  EXPECT_EQ(first_lost.state, LandingState::ALIGN);
  EXPECT_DOUBLE_EQ(first_lost.descent_speed_mps, 0.0);
  EXPECT_FALSE(first_lost.request_auto_land);

  const auto output =
      machine.update(ActiveInput(0.82, 0.30, 0.05, false));
  EXPECT_EQ(output.state, LandingState::REQUEST_AUTO_LAND);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_TRUE(output.request_auto_land);
}

TEST(LandingStateMachine, NeverDescendsDuringNearGroundLossConfirmation) {
  LandingStateMachine machine;
  ReachFinalDescent(machine);

  machine.update(ActiveInput(0.90, 0.30, 0.05, false));
  const auto output = machine.update(
      ActiveInput(1.39, 0.30, 0.05, false));

  EXPECT_EQ(output.state, LandingState::DESCEND_FINAL);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, StaleCriticalInputAbortsInsteadOfConfirmingTargetLoss) {
  LandingStateMachine machine;
  ReachFinalDescent(machine);
  machine.update(ActiveInput(0.90, 0.30, 0.05, false));

  auto stale_input = ActiveInput(1.41, 0.30, 0.05, false);
  stale_input.critical_inputs_fresh = false;
  const auto output = machine.update(stale_input);

  EXPECT_EQ(output.state, LandingState::ABORT_HOLD);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_FALSE(output.request_auto_land);

  stale_input.now_sec = 5.0;
  const auto held = machine.update(stale_input);
  EXPECT_EQ(held.state, LandingState::ABORT_HOLD);
  EXPECT_FALSE(held.request_auto_land);
}

TEST(LandingStateMachine, RequestsAutoLandOnlyAfterStableFinalAlignment) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0, 0.24));
  machine.update(ActiveInput(0.0, 0.24));
  machine.update(ActiveInput(0.30, 0.24));
  EXPECT_EQ(machine.update(ActiveInput(0.80, 0.24)).state, LandingState::DESCEND_FINAL);

  const auto early = machine.update(ActiveInput(1.29, 0.24));
  EXPECT_FALSE(early.request_auto_land);
  const auto output = machine.update(ActiveInput(1.30, 0.24));
  EXPECT_EQ(output.state, LandingState::REQUEST_AUTO_LAND);
  EXPECT_TRUE(output.request_auto_land);

  auto auto_land_active = ActiveInput(1.31, 0.24);
  auto_land_active.auto_land_active = true;
  auto_land_active.offboard = false;
  const auto complete = machine.update(auto_land_active);
  EXPECT_EQ(complete.state, LandingState::DONE);
  EXPECT_TRUE(complete.success);
}

TEST(LandingStateMachine, PassivelyAbortsWhenOffboardIsLost) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0));

  auto input = ActiveInput(0.10);
  input.offboard = false;
  const auto output = machine.update(input);

  EXPECT_EQ(output.state, LandingState::PASSIVE_ABORT);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, ReacquireTimeoutEntersAbortHold) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  EXPECT_EQ(machine.update(ActiveInput(1.41, 2.0, 0.05, false)).state,
            LandingState::REACQUIRE);

  const auto output = machine.update(ActiveInput(6.42, 2.0, 0.05, false));

  EXPECT_EQ(output.state, LandingState::ABORT_HOLD);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, FalseTriggerResetsAbortHoldToIdle) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  machine.update(ActiveInput(1.41, 2.0, 0.05, false));
  ASSERT_EQ(machine.update(ActiveInput(6.42, 2.0, 0.05, false)).state,
            LandingState::ABORT_HOLD);

  auto input = ActiveInput(6.43, 2.0, 0.05, false);
  input.trigger = false;
  const auto output = machine.update(input);

  EXPECT_EQ(output.state, LandingState::IDLE);
  EXPECT_TRUE(output.reset_target_lock);
}

TEST(LandingStateMachine, HoldsDescentWhenHighGateIsExceeded) {
  LandingStateMachine machine;
  ReachHighDescent(machine);

  const auto outside = machine.update(ActiveInput(0.90, 2.0, 0.26));
  EXPECT_EQ(outside.state, LandingState::DESCEND_HIGH);
  EXPECT_DOUBLE_EQ(outside.descent_speed_mps, 0.0);

  const auto first_inside = machine.update(ActiveInput(1.00, 2.0, 0.24));
  EXPECT_DOUBLE_EQ(first_inside.descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.49, 2.0, 0.24)).descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.51, 2.0, 0.24)).descent_speed_mps, 0.25);
}

TEST(LandingStateMachine, HoldsDescentWhenMidGateIsExceeded) {
  LandingStateMachine machine;
  ReachDescent(machine, 0.70);

  const auto outside = machine.update(ActiveInput(0.90, 0.70, 0.16));
  EXPECT_EQ(outside.state, LandingState::DESCEND_MID);
  EXPECT_DOUBLE_EQ(outside.descent_speed_mps, 0.0);

  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.00, 0.70, 0.14)).descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.51, 0.70, 0.14)).descent_speed_mps, 0.12);
}

TEST(LandingStateMachine, HoldsDescentWhenFinalGateIsExceeded) {
  LandingStateMachine machine;
  ReachFinalDescent(machine);

  const auto outside = machine.update(ActiveInput(0.90, 0.30, 0.11));
  EXPECT_EQ(outside.state, LandingState::DESCEND_FINAL);
  EXPECT_DOUBLE_EQ(outside.descent_speed_mps, 0.0);

  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.00, 0.30, 0.09)).descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.51, 0.30, 0.09)).descent_speed_mps, 0.05);
}

TEST(LandingStateMachine, NewDescentStageRequiresItsOwnStableGateWindow) {
  LandingStateMachine machine;
  ReachHighDescent(machine);

  const auto entered_mid = machine.update(ActiveInput(0.90, 0.70, 0.05));
  EXPECT_EQ(entered_mid.state, LandingState::DESCEND_MID);
  EXPECT_DOUBLE_EQ(entered_mid.descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.39, 0.70, 0.05)).descent_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(
      machine.update(ActiveInput(1.41, 0.70, 0.05)).descent_speed_mps, 0.12);
}

TEST(LandingStateMachine, LossAndReacquisitionRequestControllerHistoryReset) {
  LandingStateMachine machine;
  ReachHighDescent(machine);

  const auto first_lost =
      machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  EXPECT_TRUE(first_lost.reset_controller);
  const auto confirmed_lost =
      machine.update(ActiveInput(1.41, 2.0, 0.05, false));
  ASSERT_EQ(confirmed_lost.state, LandingState::REACQUIRE);
  EXPECT_TRUE(confirmed_lost.reset_target_jump_history);

  const auto reacquired = machine.update(ActiveInput(1.50, 2.0, 0.05, true));
  EXPECT_EQ(reacquired.state, LandingState::ALIGN);
  EXPECT_TRUE(reacquired.reset_controller);
  EXPECT_DOUBLE_EQ(reacquired.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, DoesNotRequestAutoLandAboveAutoLandHeight) {
  LandingStateMachine machine;
  ReachFinalDescent(machine);

  const auto output = machine.update(ActiveInput(1.50, 0.30));

  EXPECT_EQ(output.state, LandingState::DESCEND_FINAL);
  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, DoesNotRequestAutoLandAboveAutoLandError) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0, 0.24, 0.09));
  machine.update(ActiveInput(0.0, 0.24, 0.09));
  machine.update(ActiveInput(0.30, 0.24, 0.09));
  ASSERT_EQ(machine.update(ActiveInput(0.80, 0.24, 0.09)).state,
            LandingState::DESCEND_FINAL);

  const auto output = machine.update(ActiveInput(1.50, 0.24, 0.09));

  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, StopsVisualDescentAtFinalFloorUntilAutoLandAlignment) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0, 0.25, 0.09));
  machine.update(ActiveInput(0.0, 0.25, 0.09));
  machine.update(ActiveInput(0.30, 0.25, 0.09));
  ASSERT_EQ(machine.update(ActiveInput(0.80, 0.25, 0.09)).state,
            LandingState::DESCEND_FINAL);

  const auto output = machine.update(ActiveInput(0.90, 0.25, 0.09));

  EXPECT_EQ(output.state, LandingState::DESCEND_FINAL);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, RestartsAutoLandStabilityAfterInterruption) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0, 0.24));
  machine.update(ActiveInput(0.0, 0.24));
  machine.update(ActiveInput(0.30, 0.24));
  ASSERT_EQ(machine.update(ActiveInput(0.80, 0.24)).state, LandingState::DESCEND_FINAL);

  machine.update(ActiveInput(0.90, 0.24, 0.09));
  machine.update(ActiveInput(1.30, 0.24));
  EXPECT_FALSE(machine.update(ActiveInput(1.79, 0.24)).request_auto_land);
  EXPECT_TRUE(machine.update(ActiveInput(1.80, 0.24)).request_auto_land);
}

TEST(LandingStateMachine, EntersAbortHoldAtTotalTimeout) {
  StateMachineConfig config;
  config.total_timeout_sec = 0.50;
  LandingStateMachine machine(config);
  machine.update(ActiveInput(0.0));

  const auto output = machine.update(ActiveInput(0.50));

  EXPECT_EQ(output.state, LandingState::ABORT_HOLD);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, FalseTriggerResetsPassiveAbortToIdle) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0));
  machine.update(ActiveInput(0.1));

  StateInput manual_takeover = ActiveInput(0.2);
  manual_takeover.offboard = false;
  ASSERT_EQ(machine.update(manual_takeover).state,
            LandingState::PASSIVE_ABORT);

  StateInput reset = manual_takeover;
  reset.trigger = false;
  reset.now_sec = 0.3;
  const auto output = machine.update(reset);

  EXPECT_EQ(output.state, LandingState::IDLE);
  EXPECT_TRUE(output.reset_target_lock);
}
