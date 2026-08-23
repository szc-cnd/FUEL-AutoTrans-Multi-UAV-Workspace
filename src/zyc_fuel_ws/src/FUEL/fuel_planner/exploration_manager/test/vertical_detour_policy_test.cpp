#include <cassert>

#include "exploration_manager/vertical_detour_policy.h"

int main() {
  using fast_planner::vertical_detour::LowProbePhase;
  using fast_planner::vertical_detour::LowProbeObservation;
  using fast_planner::vertical_detour::advanceLowProbe;
  using fast_planner::vertical_detour::ascentShouldAbort;
  using fast_planner::vertical_detour::inflationEscapeAllowed;
  using fast_planner::vertical_detour::groundAscentRecoveryNeeded;
  using fast_planner::vertical_detour::preferPathByMinimumClearance;
  using fast_planner::vertical_detour::preferDescending;

  assert(!preferDescending(0.74, 0.75));
  assert(!preferDescending(0.75, 0.75));
  assert(preferDescending(0.76, 0.75));
  assert(inflationEscapeAllowed(LowProbePhase::IDLE));
  assert(!inflationEscapeAllowed(LowProbePhase::DESCENDING));
  assert(!inflationEscapeAllowed(LowProbePhase::ASCENDING));
  assert(!ascentShouldAbort(0.08, 0.08, 3.9, 4.0));
  assert(ascentShouldAbort(0.081, 0.08, 3.9, 4.0));
  assert(ascentShouldAbort(0.0, 0.08, 4.0, 4.0));

  // 只有低于巡航层、下方存在足够占据支撑且整根上升柱安全时才抢占水平逃逸。
  assert(groundAscentRecoveryNeeded(0.22, 0.60, 0.06, 3, 2, true));
  assert(!groundAscentRecoveryNeeded(0.55, 0.60, 0.06, 3, 2, true));
  assert(!groundAscentRecoveryNeeded(0.22, 0.60, 0.06, 1, 2, true));
  assert(!groundAscentRecoveryNeeded(0.22, 0.60, 0.06, 3, 2, false));

  int confirmations = 0;
  int release_confirmations = 0;
  auto phase = LowProbePhase::DESCENDING;
  LowProbeObservation observation;

  // 未原地到达低位前，绝不能进入前进确认。
  observation.xy_stable = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::DESCENDING);
  observation.at_probe_height = true;
  observation.xy_stable = false;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::DESCENDING);

  // 原地到达后进入确认；必须连续三次安全，任意一次失败都会清零。
  observation.xy_stable = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::VERIFYING);
  observation.low_corridor_safe = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::VERIFYING && confirmations == 1);
  observation.low_corridor_safe = false;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::VERIFYING && confirmations == 0);
  observation.low_corridor_safe = true;
  for (int i = 0; i < 3; ++i) {
    phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                            release_confirmations);
  }
  assert(phase == LowProbePhase::ADVANCING);

  // 低位走完一个短步但上升柱仍被悬空障碍挡住时，必须继续低位确认，不能退出。
  observation.advance_complete = true;
  observation.ascent_path_safe = false;
  observation.release_corridor_safe = false;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::VERIFYING);

  // 下一段通过后，只有上升柱及恢复高度前方都连续安全两次才允许抬升。
  observation.advance_complete = false;
  observation.low_corridor_safe = true;
  for (int i = 0; i < 3; ++i) {
    phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                            release_confirmations);
  }
  assert(phase == LowProbePhase::ADVANCING);
  observation.advance_complete = true;
  observation.ascent_path_safe = true;
  observation.release_corridor_safe = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::EXIT_VERIFYING && release_confirmations == 1);
  observation.advance_complete = false;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::ASCENDING && release_confirmations == 2);
  observation.ascent_complete = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::IDLE);

  // 无法确认时只能沿已知安全的垂直柱回升；球仍在头顶时必须留在低位。
  confirmations = 2;
  release_confirmations = 0;
  observation = LowProbeObservation{};
  observation.verification_timed_out = true;
  phase = advanceLowProbe(LowProbePhase::VERIFYING, observation, 3, 2,
                          confirmations, release_confirmations);
  assert(phase == LowProbePhase::VERIFYING);
  observation.ascent_path_safe = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::ASCENDING && confirmations == 0);
  observation = LowProbeObservation{};
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::ASCENDING);
  observation.ascent_complete = true;
  phase = advanceLowProbe(phase, observation, 3, 2, confirmations,
                          release_confirmations);
  assert(phase == LowProbePhase::IDLE);

  // 安全确认与超时同周期发生时，已经确认的低位短通道应当优先放行。
  confirmations = 0;
  release_confirmations = 0;
  observation = LowProbeObservation{};
  observation.low_corridor_safe = true;
  observation.verification_timed_out = true;
  phase = advanceLowProbe(LowProbePhase::VERIFYING, observation, 1, 2,
                          confirmations, release_confirmations);
  assert(phase == LowProbePhase::ADVANCING && confirmations == 1);

  // 侧绕排序看整条路径最小净空，不能被终点处的虚高净空诱导去贴墙。
  assert(preferPathByMinimumClearance(0.35, 0.45, 0.30, 0.60, 0.30, 0.20,
                                     0.5, 0.8));
  assert(!preferPathByMinimumClearance(0.20, 0.60, 0.35, 0.45, 0.20, 0.30,
                                      0.8, 0.5));
  return 0;
}
