#include "precision_landing/landing_state_machine.hpp"

#include <algorithm>

namespace precision_landing {
namespace {

double initialAlignmentGate(const StateMachineConfig& config,
                            double /*marker_height_m*/) {
  return std::min(config.align_error_m, config.high_align_error_m);
}

bool clearsOnFalseTrigger(LandingState state) {
  return state == LandingState::PRECHECK || state == LandingState::ACQUIRE ||
         state == LandingState::ABORT_HOLD ||
         state == LandingState::PASSIVE_ABORT ||
         state == LandingState::DONE;
}

}  // namespace

LandingStateMachine::LandingStateMachine(const StateMachineConfig& config) : config_(config) {}

StateOutput LandingStateMachine::update(const StateInput& input) {
  bool reset_target_lock = false;
  bool reset_controller = false;
  bool reset_target_jump_history = false;
  bool entered_precheck = false;
  if (!input.trigger && clearsOnFalseTrigger(state_)) {
    state_ = LandingState::IDLE;
    target_visible_since_sec_ = -1.0;
    aligned_since_sec_ = -1.0;
    target_loss_since_sec_ = -1.0;
    descent_gate_since_sec_ = -1.0;
    descent_gate_state_ = LandingState::IDLE;
    reacquire_started_sec_ = -1.0;
    mission_started_sec_ = -1.0;
    reset_target_lock = true;
  } else if (state_ == LandingState::IDLE && input.trigger) {
    state_ = LandingState::PRECHECK;
    mission_started_sec_ = input.now_sec;
    entered_precheck = true;
  }

  if (!entered_precheck && state_ == LandingState::REQUEST_AUTO_LAND &&
             input.auto_land_active) {
    state_ = LandingState::DONE;
  } else if (!entered_precheck && state_ != LandingState::IDLE &&
             state_ != LandingState::PRECHECK &&
             state_ != LandingState::PASSIVE_ABORT &&
             state_ != LandingState::DONE &&
             !input.critical_inputs_fresh) {
    state_ = LandingState::ABORT_HOLD;
  } else if (!entered_precheck && state_ != LandingState::IDLE &&
             state_ != LandingState::PASSIVE_ABORT && state_ != LandingState::DONE &&
             !input.offboard) {
    state_ = LandingState::PASSIVE_ABORT;
  } else if (!entered_precheck && state_ == LandingState::PRECHECK && input.precheck_ok &&
             input.offboard) {
    state_ = LandingState::ACQUIRE;
    target_visible_since_sec_ = input.target_visible ? input.now_sec : -1.0;
  } else if (!entered_precheck && state_ == LandingState::ACQUIRE) {
    if (!input.target_visible) {
      target_visible_since_sec_ = -1.0;
    } else if (target_visible_since_sec_ < 0.0) {
      target_visible_since_sec_ = input.now_sec;
    } else if (input.now_sec - target_visible_since_sec_ >= config_.acquire_stable_sec) {
      state_ = LandingState::ALIGN;
      aligned_since_sec_ =
          input.horizontal_error_m <=
                  initialAlignmentGate(config_, input.marker_height_m)
              ? input.now_sec
              : -1.0;
    }
  } else if (!entered_precheck &&
             state_ == LandingState::FIXED_XY_DESCENT) {
    if (input.landing_contact) {
      state_ = LandingState::REQUEST_AUTO_LAND;
    }
  } else if (!entered_precheck &&
             (state_ == LandingState::ALIGN ||
              state_ == LandingState::DESCEND_HIGH)) {
    if (!input.target_visible) {
      if (target_loss_since_sec_ < 0.0) {
        target_loss_since_sec_ = input.now_sec;
        reset_controller = true;
      }
      descent_gate_since_sec_ = -1.0;
      descent_gate_state_ = LandingState::IDLE;
      if (input.now_sec - target_loss_since_sec_ >=
          config_.target_loss_timeout_sec) {
        state_ = LandingState::REACQUIRE;
        reacquire_started_sec_ = input.now_sec;
        reset_controller = true;
        reset_target_jump_history = true;
      }
    } else {
      target_loss_since_sec_ = -1.0;
      if (input.marker_height_m <= config_.auto_land_height_m) {
        // Once the visual descent reaches the handoff height, freeze the
        // current world-frame X/Y immediately.  Do not wait for a second
        // fine-alignment gate: the fixed-X/Y phase is intentionally
        // independent of subsequent marker visibility.
        state_ = LandingState::FIXED_XY_DESCENT;
        descent_gate_since_sec_ = -1.0;
        descent_gate_state_ = LandingState::IDLE;
      } else if (state_ == LandingState::ALIGN) {
        const double alignment_gate =
            std::min(config_.align_error_m, config_.high_align_error_m);
        if (input.horizontal_error_m > alignment_gate) {
          aligned_since_sec_ = -1.0;
        } else if (aligned_since_sec_ < 0.0) {
          aligned_since_sec_ = input.now_sec;
        } else if (input.now_sec - aligned_since_sec_ >= config_.align_stable_sec) {
          state_ = LandingState::DESCEND_HIGH;
          descent_gate_state_ = state_;
          descent_gate_since_sec_ = aligned_since_sec_;
        }
      } else {
        if (input.horizontal_error_m > config_.high_align_error_m) {
          descent_gate_since_sec_ = -1.0;
        } else if (descent_gate_state_ != state_ ||
                   descent_gate_since_sec_ < 0.0) {
          descent_gate_state_ = state_;
          descent_gate_since_sec_ = input.now_sec;
        }
      }
    }
  } else if (!entered_precheck && state_ == LandingState::REACQUIRE) {
    if (input.target_visible) {
      state_ = LandingState::ALIGN;
      target_loss_since_sec_ = -1.0;
      aligned_since_sec_ =
          input.horizontal_error_m <=
                  initialAlignmentGate(config_, input.marker_height_m)
              ? input.now_sec
              : -1.0;
      descent_gate_since_sec_ = -1.0;
      descent_gate_state_ = LandingState::IDLE;
      reset_controller = true;
    }
  }

  StateOutput output;
  output.state = state_;
  output.reset_target_lock = reset_target_lock;
  output.reset_controller = reset_controller;
  output.reset_target_jump_history = reset_target_jump_history;
  output.reason = toString(state_);
  const bool above_visual_descent_floor =
      input.marker_height_m > config_.auto_land_height_m;
  if (state_ == LandingState::FIXED_XY_DESCENT) {
    output.descent_speed_mps = config_.fixed_descent_mps;
  } else if (state_ == LandingState::DESCEND_HIGH && input.target_visible &&
      above_visual_descent_floor &&
      input.horizontal_error_m <= config_.high_align_error_m &&
      descent_gate_state_ == state_ &&
      descent_gate_since_sec_ >= 0.0 &&
      input.now_sec - descent_gate_since_sec_ >=
          config_.align_stable_sec) {
    output.descent_speed_mps = config_.high_descent_mps;
  } else if (state_ == LandingState::REQUEST_AUTO_LAND) {
    output.request_auto_land = true;
  } else if (state_ == LandingState::DONE) {
    output.success = true;
  }
  return output;
}

}  // namespace precision_landing
