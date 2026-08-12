#include <cassert>

#include "exploration_manager/vertical_detour_policy.h"

int main() {
  using fast_planner::vertical_detour::LowProbePhase;
  using fast_planner::vertical_detour::advanceLowProbe;
  using fast_planner::vertical_detour::preferDescending;

  assert(!preferDescending(0.74, 0.75));
  assert(!preferDescending(0.75, 0.75));
  assert(preferDescending(0.76, 0.75));

  int confirmations = 0;
  auto phase = LowProbePhase::DESCENDING;

  // 未原地到达低位前，绝不能进入前进确认。
  phase = advanceLowProbe(phase, false, true, false, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::DESCENDING);
  phase = advanceLowProbe(phase, true, false, false, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::DESCENDING);

  // 原地到达后进入确认；必须连续三次安全，任意一次失败都会清零。
  phase = advanceLowProbe(phase, true, true, false, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::VERIFYING);
  phase = advanceLowProbe(phase, true, true, true, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::VERIFYING && confirmations == 1);
  phase = advanceLowProbe(phase, true, true, false, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::VERIFYING && confirmations == 0);
  for (int i = 0; i < 3; ++i)
    phase = advanceLowProbe(phase, true, true, true, false, false, false, 3,
                            confirmations);
  assert(phase == LowProbePhase::ADVANCING);

  // 低位前进结束后退出专用状态。
  phase = advanceLowProbe(phase, true, true, true, false, true, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::IDLE);

  // 无法确认时必须回升，回到原高度后才退出。
  confirmations = 2;
  phase = advanceLowProbe(LowProbePhase::VERIFYING, true, true, false, true,
                          false, false, 3, confirmations);
  assert(phase == LowProbePhase::ASCENDING && confirmations == 0);
  phase = advanceLowProbe(phase, false, true, false, false, false, false, 3,
                          confirmations);
  assert(phase == LowProbePhase::ASCENDING);
  phase = advanceLowProbe(phase, false, true, false, false, false, true, 3,
                          confirmations);
  assert(phase == LowProbePhase::IDLE);

  // 安全确认与超时同周期发生时，已经确认的低位短通道应当优先放行。
  confirmations = 0;
  phase = advanceLowProbe(LowProbePhase::VERIFYING, true, true, true, true,
                          false, false, 1, confirmations);
  assert(phase == LowProbePhase::ADVANCING && confirmations == 1);
  return 0;
}
