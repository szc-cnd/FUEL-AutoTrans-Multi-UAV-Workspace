#include <gtest/gtest.h>

#include "precision_landing/landing_state_machine.hpp"

using precision_landing::LandingState;
using precision_landing::LandingStateMachine;
using precision_landing::StateInput;

namespace {

StateInput ActiveInput(double now_sec, double marker_height_m = 2.0,
                       double horizontal_error_m = 0.05,
                       bool target_visible = true) {
  return StateInput{true, true, true, target_visible, false,
                    horizontal_error_m, marker_height_m, now_sec};
}

void ReachAlign(LandingStateMachine& machine, double height_m = 2.0) {
  machine.update(ActiveInput(0.0, height_m));
  machine.update(ActiveInput(0.0, height_m));
  ASSERT_EQ(machine.update(ActiveInput(0.30, height_m)).state,
            LandingState::ALIGN);
}

void ReachHighDescent(LandingStateMachine& machine) {
  ReachAlign(machine);
  ASSERT_EQ(machine.update(ActiveInput(0.80)).state,
            LandingState::DESCEND_HIGH);
}

void ReachFixedDescent(LandingStateMachine& machine) {
  ReachAlign(machine, 1.50);
  ASSERT_EQ(machine.update(ActiveInput(0.80, 1.50)).state,
            LandingState::ALIGN);
  ASSERT_EQ(machine.update(ActiveInput(1.31, 1.50)).state,
            LandingState::FIXED_XY_DESCENT);
}

}  // namespace

TEST(StateInput, SupportsDefaultAndLegacyConstruction) {
  const StateInput default_input;
  EXPECT_FALSE(default_input.trigger);
  EXPECT_TRUE(default_input.critical_inputs_fresh);
  EXPECT_FALSE(default_input.landing_contact);

  const StateInput legacy_input{true, true, true, true, false, 0.1, 0.2, 0.3};
  EXPECT_DOUBLE_EQ(legacy_input.now_sec, 0.3);
  EXPECT_TRUE(legacy_input.critical_inputs_fresh);
}

TEST(LandingStateMachine, NeverDescendsWhenPrecheckFails) {
  LandingStateMachine machine;
  machine.update(StateInput{true, false, true, true, false, 0.0, 2.0, 0.0});
  const auto output = machine.update(
      StateInput{true, false, true, true, false, 0.0, 2.0, 1.0});
  EXPECT_EQ(output.state, LandingState::PRECHECK);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, RequiresStableAcquireAndAlignBeforeVisualDescent) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0));
  machine.update(ActiveInput(0.0));
  EXPECT_EQ(machine.update(ActiveInput(0.29)).state, LandingState::ACQUIRE);
  EXPECT_EQ(machine.update(ActiveInput(0.30)).state, LandingState::ALIGN);
  EXPECT_DOUBLE_EQ(machine.update(ActiveInput(0.79)).descent_speed_mps, 0.0);
  const auto output = machine.update(ActiveInput(0.80));
  EXPECT_EQ(output.state, LandingState::DESCEND_HIGH);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.25);
}

TEST(LandingStateMachine, TargetLossStopsVisualDescentAndReacquires) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  const auto first_lost = machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  EXPECT_EQ(first_lost.state, LandingState::DESCEND_HIGH);
  EXPECT_DOUBLE_EQ(first_lost.descent_speed_mps, 0.0);
  EXPECT_TRUE(first_lost.reset_controller);

  const auto confirmed = machine.update(ActiveInput(1.41, 2.0, 0.05, false));
  EXPECT_EQ(confirmed.state, LandingState::REACQUIRE);
  EXPECT_TRUE(confirmed.reset_target_jump_history);
}

TEST(LandingStateMachine, ReacquisitionContinuesWhileInputsRemainHealthy) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  ASSERT_EQ(machine.update(ActiveInput(1.41, 2.0, 0.05, false)).state,
            LandingState::REACQUIRE);
  EXPECT_EQ(machine.update(ActiveInput(20.0, 2.0, 0.05, false)).state,
            LandingState::REACQUIRE);
}

TEST(LandingStateMachine, ReacquiredTargetReturnsToAlignment) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  machine.update(ActiveInput(0.90, 2.0, 0.05, false));
  machine.update(ActiveInput(1.41, 2.0, 0.05, false));
  const auto output = machine.update(ActiveInput(1.50));
  EXPECT_EQ(output.state, LandingState::ALIGN);
  EXPECT_TRUE(output.reset_controller);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, StableFineAlignmentEntersFixedWorldXyDescent) {
  LandingStateMachine machine;
  ReachAlign(machine, 1.50);
  EXPECT_EQ(machine.update(ActiveInput(0.80, 1.50)).state,
            LandingState::ALIGN);
  EXPECT_EQ(machine.update(ActiveInput(1.29, 1.50)).state,
            LandingState::ALIGN);
  const auto output = machine.update(ActiveInput(1.31, 1.50));
  EXPECT_EQ(output.state, LandingState::FIXED_XY_DESCENT);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.10);
}

TEST(LandingStateMachine, FixedWorldXyDescentNoLongerRequiresMarker) {
  LandingStateMachine machine;
  ReachFixedDescent(machine);
  const auto output = machine.update(ActiveInput(1.0, 0.0, 0.0, false));
  EXPECT_EQ(output.state, LandingState::FIXED_XY_DESCENT);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.10);
  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, FixedDescentRequestsAutoLandOnlyAtContactGate) {
  LandingStateMachine machine;
  ReachFixedDescent(machine);
  auto contact = ActiveInput(1.0, 0.0, 0.0, false);
  contact.landing_contact = true;
  const auto output = machine.update(contact);
  EXPECT_EQ(output.state, LandingState::REQUEST_AUTO_LAND);
  EXPECT_TRUE(output.request_auto_land);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
}

TEST(LandingStateMachine, AutoLandModeConfirmationCompletesMission) {
  LandingStateMachine machine;
  ReachFixedDescent(machine);
  auto input = ActiveInput(1.0, 0.0, 0.0, false);
  input.landing_contact = true;
  machine.update(input);
  input.auto_land_active = true;
  input.offboard = false;
  const auto output = machine.update(input);
  EXPECT_EQ(output.state, LandingState::DONE);
  EXPECT_TRUE(output.success);
}

TEST(LandingStateMachine, StaleCriticalInputEntersAbortHold) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  auto input = ActiveInput(0.90);
  input.critical_inputs_fresh = false;
  const auto output = machine.update(input);
  EXPECT_EQ(output.state, LandingState::ABORT_HOLD);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.0);
  EXPECT_FALSE(output.request_auto_land);
}

TEST(LandingStateMachine, OffboardLossPassivelyAborts) {
  LandingStateMachine machine;
  machine.update(ActiveInput(0.0));
  auto input = ActiveInput(0.10);
  input.offboard = false;
  EXPECT_EQ(machine.update(input).state, LandingState::PASSIVE_ABORT);
}

TEST(LandingStateMachine, FallingTriggerDoesNotInterruptFixedDescent) {
  LandingStateMachine machine;
  ReachFixedDescent(machine);
  auto input = ActiveInput(1.0, 0.0, 0.0, false);
  input.trigger = false;
  const auto output = machine.update(input);
  EXPECT_EQ(output.state, LandingState::FIXED_XY_DESCENT);
  EXPECT_DOUBLE_EQ(output.descent_speed_mps, 0.10);
}

TEST(LandingStateMachine, FalseTriggerResetsAbortToIdle) {
  LandingStateMachine machine;
  ReachHighDescent(machine);
  auto stale = ActiveInput(0.90);
  stale.critical_inputs_fresh = false;
  ASSERT_EQ(machine.update(stale).state, LandingState::ABORT_HOLD);
  stale.trigger = false;
  const auto output = machine.update(stale);
  EXPECT_EQ(output.state, LandingState::IDLE);
  EXPECT_TRUE(output.reset_target_lock);
}
