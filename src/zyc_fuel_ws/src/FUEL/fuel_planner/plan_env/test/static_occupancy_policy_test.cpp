#include <cassert>

#include "plan_env/static_occupancy_policy.h"

int main() {
  using fast_planner::occupancy_policy::StaticEvidence;
  using fast_planner::occupancy_policy::RetentionState;
  using fast_planner::occupancy_policy::StructuredRetentionConfig;
  using fast_planner::occupancy_policy::StructuredRetentionStatus;
  using fast_planner::occupancy_policy::updateStructuredRetention;

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

  const StructuredRetentionConfig config{/*temporary_hits=*/6,
                                         /*static_hits=*/6,
                                         /*single_view_static_hits=*/10,
                                         /*release_misses=*/12};

  // 孤立噪点即使多帧命中，没有邻域/竖直结构也不能进入保护状态。
  StructuredRetentionStatus isolated;
  for (int i = 0; i < 10; ++i)
    isolated = updateStructuredRetention(isolated, true, false, false, false,
                                         false, config).status;
  assert(isolated.state == RetentionState::NONE);

  // 六帧结构化命中才进入临时保护；近场临时障碍仍按miss衰减。
  StructuredRetentionStatus structured;
  for (int i = 0; i < 5; ++i)
    structured = updateStructuredRetention(structured, true, true, false, false,
                                           false, config).status;
  assert(structured.state == RetentionState::NONE);
  structured = updateStructuredRetention(structured, true, true, false, false,
                                         false, config).status;
  assert(structured.state == RetentionState::TEMPORARY);
  const auto near_miss = updateStructuredRetention(
      structured, false, true, false, true, false, config);
  assert(near_miss.apply_observation);
  assert(near_miss.status.state == RetentionState::NONE);
  assert(near_miss.status.hit_evidence == structured.hit_evidence - 1);

  // 离开盲区后的自由观测正常降级临时障碍，避免噪声永久残留。
  const auto far_miss = updateStructuredRetention(
      structured, false, true, false, false, false, config);
  assert(far_miss.apply_observation);
  assert(far_miss.status.state == RetentionState::NONE);

  // 多视角在6帧确认静态；单视角必须达到更保守的10帧。
  StructuredRetentionStatus multiview;
  for (int i = 0; i < 6; ++i)
    multiview = updateStructuredRetention(multiview, true, true, i >= 1, false,
                                          false, config).status;
  assert(multiview.state == RetentionState::STATIC);

  // 只有已确认的静态障碍在0.35m近场忽略miss。
  const auto static_near_miss = updateStructuredRetention(
      multiview, false, true, true, true, false, config);
  assert(!static_near_miss.apply_observation);
  assert(static_near_miss.status.state == RetentionState::STATIC);

  StructuredRetentionStatus single_view;
  for (int i = 0; i < 6; ++i)
    single_view = updateStructuredRetention(single_view, true, true, false, false,
                                            false, config).status;
  assert(single_view.state == RetentionState::TEMPORARY);
  for (int i = 6; i < 10; ++i)
    single_view = updateStructuredRetention(single_view, true, true, false, false,
                                            false, config).status;
  assert(single_view.state == RetentionState::STATIC);

  // 静态障碍只有在盲区外累计足够自由证据且自由观测具备多视角基线时才解除。
  StructuredRetentionStatus clearing = multiview;
  for (int i = 0; i < 12; ++i)
    clearing = updateStructuredRetention(clearing, false, true, true, false,
                                         false, config).status;
  assert(clearing.state == RetentionState::STATIC);
  const auto released = updateStructuredRetention(
      clearing, false, true, true, false, true, config);
  assert(released.clear_occupancy);
  assert(released.status.state == RetentionState::NONE);
  return 0;
}
