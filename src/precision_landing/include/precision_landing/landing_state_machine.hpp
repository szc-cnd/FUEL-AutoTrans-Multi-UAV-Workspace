#pragma once

#include <string>

#include "precision_landing/types.hpp"

namespace precision_landing {

struct StateMachineConfig {
  double acquire_stable_sec{0.30};
  double align_stable_sec{0.50};
  double align_error_m{0.20};
  double high_height_m{1.50};
  double auto_land_height_m{1.50};
  double high_align_error_m{0.25};
  double auto_land_error_m{0.08};
  double high_descent_mps{0.25};
  double fixed_descent_mps{0.10};
  double target_loss_timeout_sec{0.50};
  double reacquire_timeout_sec{5.0};
  double total_timeout_sec{30.0};
};

struct StateInput {
  StateInput(bool trigger_value = false, bool precheck_ok_value = false,
             bool offboard_value = false, bool target_visible_value = false,
             bool auto_land_active_value = false, double horizontal_error_m_value = 0.0,
             double marker_height_m_value = 0.0, double now_sec_value = 0.0,
             bool critical_inputs_fresh_value = true,
             bool landing_contact_value = false)
      : trigger(trigger_value),
        precheck_ok(precheck_ok_value),
        offboard(offboard_value),
        target_visible(target_visible_value),
        auto_land_active(auto_land_active_value),
        horizontal_error_m(horizontal_error_m_value),
        marker_height_m(marker_height_m_value),
        now_sec(now_sec_value),
        critical_inputs_fresh(critical_inputs_fresh_value),
        landing_contact(landing_contact_value) {}

  bool trigger;
  bool precheck_ok;
  bool offboard;
  bool target_visible;
  bool auto_land_active;
  double horizontal_error_m;
  double marker_height_m;
  double now_sec;
  bool critical_inputs_fresh;
  bool landing_contact;
};

struct StateOutput {
  LandingState state{LandingState::IDLE};
  double descent_speed_mps{0.0};
  bool request_auto_land{false};
  bool reset_target_lock{false};
  bool reset_controller{false};
  bool reset_target_jump_history{false};
  bool success{false};
  std::string reason;
};

class LandingStateMachine {
 public:
  explicit LandingStateMachine(const StateMachineConfig& config = StateMachineConfig());

  StateOutput update(const StateInput& input);

 private:
  StateMachineConfig config_;
  LandingState state_{LandingState::IDLE};
  double target_visible_since_sec_{-1.0};
  double aligned_since_sec_{-1.0};
  double target_loss_since_sec_{-1.0};
  double auto_land_aligned_since_sec_{-1.0};
  double descent_gate_since_sec_{-1.0};
  LandingState descent_gate_state_{LandingState::IDLE};
  double reacquire_started_sec_{-1.0};
  double mission_started_sec_{-1.0};
};

}  // namespace precision_landing
