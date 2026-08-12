#include <cassert>

#include "plan_env/static_occupancy_policy.h"

int main() {
  using fast_planner::occupancy_policy::StaticEvidence;

  StaticEvidence evidence(/*required_hits=*/3);
  evidence.observe(true);
  evidence.observe(true);
  assert(!evidence.locked());
  evidence.observe(false);
  assert(!evidence.locked());
  evidence.observe(true);
  assert(!evidence.locked());
  evidence.observe(true);
  assert(evidence.locked());

  // 锁存后普通漏扫不再删除；未达到门槛的临时点仍可被miss清除。
  assert(!evidence.applyMiss());
  StaticEvidence transient(/*required_hits=*/3);
  transient.observe(true);
  assert(transient.applyMiss());
  return 0;
}
