# RAL-25 意图预测改进的轨迹预测实现方案

## 目标范围

本方案只讨论动态目标短时轨迹预测，不包含规划控制或 MPC 设计。输入为动态目标的过往轨迹、当前位置速度及其不确定性，输出为物体中心未来轨迹、trajectory-level GMM 概率分布及其闭环校准结果，并通过闭环指标评估预测误差、不确定性校准和耗时。

对比对象是 **Intent Prediction-Driven Model Predictive Control for UAV Planning and Navigation in Dynamic Environments**（RAL-25）。该方法使用离散意图 MDP 估计 `forward/left/right/stop` 等意图，对每个意图采样轨迹后，通过每个时间步所有采样轨迹位置的均值确定预测轨迹，并用位置标准差膨胀风险规模。其主要限制是：多意图结果被均值化后丢失多峰性，目标运动假设主要在二维平面内，交互主要通过碰撞约束体现，上一轮预测的不确定性没有闭环回馈到下一轮意图概率和轨迹协方差。

## Selected Opportunity

将 RAL-25 的 `MDP intent + sampled trajectory mean/std` 改为 **闭环不确定性交互意图混合预测器**，即 `Closed-loop Uncertainty and Interaction-aware Intent Mixture Predictor`。

该方法的核心输出包括：

- 物体中心轨迹：同时提供 `MAP mode trajectory` 与 `mixture expected trajectory`，避免把多峰预测强行压成单一均值。
- 多轨迹概率分布：每个 intent 或 motion mode 对应一条时序 Gaussian，整体构成 trajectory-level GMM。
- 分布本体：保留可被下游直接消费的 GMM components；不把仅用于可视化的额外采样轨迹作为预测输出。
- 闭环评估：预测残差反过来更新 mode probability、过程噪声和不确定性校准。

当前 LDOP 实现切分如下：P2 已发布 branch-level trajectory-level GMM，P3 已加入目标间/地图交互能量、free corridor 方向先验和轻量 rerollout，P4 已在 predictor 内部用上一轮预测 residual / NIS / likelihood 闭环更新下一帧同语义分支概率先验和未来过程噪声尺度。P3 同时已把 UFOMap 应用层的显式 occupancy state、coarse/fine query depth 和最近 occupied 查询作为跨阶段基础接入；`BoundingVolume-lite` 几何查询抽象、swept-volume 辅助线段检查、完整/局部地图发布后置增强仍留给 LDOP 第四阶段。预测话题和 RViz 只表达真实保留的 GMM branch；此前仅用于可视化的额外采样轨迹已删除，因为它不服务实际规划或评估契约。后续若某个算法内部确实需要 Monte Carlo samples，应由该消费者从 GMM 临时派生，不进入 predictor 默认消息或 RViz marker。离线多 horizon evaluator 和 LDOP 第四阶段 planner-facing 风险地图仍作为后续工作独立推进。

## Worth-Doing Gate

Decision: **proceed**。

Reason: 该方案不是对 RAL-25 的参数微调，而是改变预测表示和更新机制：从离散 MDP 后处理，升级为 `intent-conditioned Bayesian filtering + interaction energy + calibrated GMM`。

Maturity: **可实现，适合作为 LDOP 第三阶段轨迹预测模块的主方法**。它不强依赖学习数据，可先实现模型驱动版本；若后续有数据，再将 map/interaction cost 的权重学习化。

## Contribution Positioning

Primary contribution type: **新预测表示 + 新闭环更新机制**。

What is actually new:

- 将 tracker 输出的状态协方差、历史预测残差、动态目标间交互、目标-地图交互共同用于 intent probability 与 trajectory covariance 更新。
- 保留 trajectory-level GMM，使多峰轨迹分布成为一等输出，而不是只输出均值轨迹和风险膨胀半径。
- 用预测残差的统计一致性闭环调整各 mode 的置信度和过程噪声。

What is borrowed:

- Trajectron++、AgentFormer、EvolveGraph 等工作证明了多模态、交互和地图上下文对轨迹预测的重要性。
- Bayesian intention inference 与 IMM-KF 类方法证明了轻量递归意图更新可在线运行。

What must be proven:

- 相比 RAL-25 式 MDP，GMM 不确定性更校准。
- `NLL`、`coverage error`、`minADE`、`minFDE` 等指标改善。
- 在线耗时满足 UAV 动态环境预测需求。

## Research Question

如何在不引入规划控制器的前提下，利用动态目标历史状态、不确定性、运动学意图和目标间/地图交互，生成在线可更新、校准良好的多模态轨迹概率分布？

## Hypotheses

H1: 用预测残差的 NIS/likelihood 闭环更新 intent probability，会降低 mode collapse，并改善 `NLL`、`coverage error` 和 `minFDE`。

H2: 在每个 intent mode 内加入轻量交互能量项，会在会车、穿越、遮挡恢复等场景降低 `minADE/minFDE`，尤其减少不合理穿越和碰撞式预测。

H3: 保留 trajectory-level GMM，而不是输出所有样本均值轨迹，可提高多峰场景的概率校准；若只看 ADE 可能提升有限，但 `NLL/CRPS/ECE` 应明显改善。

## Mechanism Causal Chain

Failure mechanism:

RAL-25 的多轨迹采样结果最终被压缩为每个时间步的位置均值和标准差，导致左右绕行、停止/继续、避让/不避让等多峰意图被平均成一条实际不可行的中间轨迹。同时，预测误差没有闭环影响下一帧的意图概率和不确定性。

Intervention:

维护 intent-conditioned GMM，并用预测残差、目标间交互代价和地图可达性共同更新每个 mode 的概率与协方差。

Changed variable or representation:

从单均值轨迹 `mean trajectory + inflated radius` 改为 `trajectory-level Gaussian mixture`：

```text
p(Y_i | history, map, agents) = Σ_z π_z · N(Y_i ; μ_z, Σ_z)
```

Expected intermediate effect:

多峰结构被保留；最近预测不准的 mode 权重下降或协方差增大；与地图和邻近目标明显冲突的 mode 被软惩罚。

Expected final metric improvement:

`NLL`、`coverage error`、`minADE/minFDE`、`collision-like prediction rate` 和在线稳定性改善。

Falsifying observation:

若闭环 GMM 相比 RAL-25-style MDP 不能改善 `NLL` 或 coverage，且只通过调大协方差获得看似安全的覆盖率，则主机制不成立。

## Method Design

定义每个目标 `i` 的状态：

```text
x_i = [p_x, p_y, p_z, v_x, v_y, v_z]^T
```

定义离散 mode 或 intent：

```text
z_i ∈ {CV, CA, turn-left, turn-right, stop, yield, avoid-left, avoid-right, climb, descend, hold-z}
```

每个 mode 有自己的运动模型：

```text
x_{k+1}^{i,z} = f_z(x_k^{i,z}, Δt) + w_z
w_z ~ N(0, Q_z)
```

mode probability 递归更新：

```text
π_z(t|t-1) = Σ_z' T_{z'z}(history, map, interaction) π_z'(t-1)

L_z = N(y_t ; H μ_z^-(t), S_z)
E_z = λ_map E_map + λ_pair E_pair + λ_heading E_heading + λ_dyn E_dyn

π_z(t) = normalize(π_z(t|t-1) · L_z · exp(-E_z / τ))
```

闭环不确定性来自预测残差：

```text
ν_z = y_t - H μ_z^-(t)
NIS_z = ν_z^T S_z^{-1} ν_z
Q_z(t) = α_z(t) Q_z_base
```

其中 `α_z(t)` 由最近窗口内的 NIS 校准，不是无限膨胀。它表达“该 mode 最近是否解释得了真实观测”。

## Interaction Mechanism

目标间交互不只做碰撞约束，而是让动态目标具有“主观能动性”的轻量近似：

```text
E_pair(i,j,z_i,z_j) =
  w_ttc · softplus(TTC_safe - TTC_ij)
+ w_cone · collision_cone_overlap
+ w_yield · priority_inconsistency
```

目标-地图交互：

```text
E_map =
  w_occ · occupied_or_near_obstacle_cost
+ w_esdf · exp(-distance_to_obstacle / σ)
+ w_free · heading_to_free_corridor_penalty
```

这样预测器会倾向于生成“避让、减速、绕行、继续前进”等多条可解释轨迹，而不是只在风险层面膨胀障碍物。

## Retained Variants

1. Variant: 学习式 interaction encoder。
   Why it is not the main method: 需要足够多标注或自监督轨迹数据，当前阶段会显著提高实现和验证成本。
   Missing mechanism or evidence: 需要证明学习特征比手工交互能量在 LDOP 场景中更稳健。
   Possible ablation or minimum test: 用轻量 MLP 替代 `E_pair` 权重函数，对比固定权重版本。

2. Variant: 全 3D intent set。
   Why it is not the main method: UAV 场景确实有 z 方向运动，但多数动态目标可能仍以地面或近似平面运动为主。
   Missing mechanism or evidence: 需要数据证明 z 方向意图对预测误差有显著贡献。
   Possible ablation or minimum test: 先做 `2.5D`，即 xy intent 加 `climb/descend/hold-z` 三类 z 模式。

## Information Flow

```text
tracker history + covariance
        ↓
motion feature / heading feature / map feature / pairwise interaction feature
        ↓
intent-conditioned Bayesian mode update
        ↓
per-mode Gaussian rollout with covariance propagation
        ↓
trajectory-level GMM + MAP/expected center trajectory
        ↓
next frame residual feedback updates π_z and Q_z
```

## Algorithm Sketch

```text
Input:
  tracked objects with IDs, class labels, state means, covariances
  history window H_past
  local static/dynamic map representation
  prediction horizon H_future

For each object i:
  1. Build kinematic features from history:
       velocity trend, acceleration trend, yaw/heading change, stop likelihood

  2. Build map features:
       distance to occupied space, free corridor direction, height constraint

  3. Build pairwise features:
       relative position, relative velocity, TTC, collision cone overlap

  4. For each mode z:
       propagate μ_z and Σ_z by f_z
       compute observation likelihood L_z from current measurement
       compute interaction energy E_z
       update π_z by likelihood and energy
       update Q_z scale using recent NIS consistency

  5. Roll out each mode over future horizon:
       produce μ_z(1:H), Σ_z(1:H)

  6. Normalize mixture:
       p(Y_i) = Σ_z π_z N(Y_i; μ_z, Σ_z)

  7. Keep the GMM components as the prediction distribution; do not add visual-only sampled trajectories.

Output:
  MAP trajectory
  mixture expected trajectory
  per-mode Gaussian trajectory distribution
```

## Output Format 建议

每个动态目标输出：

```text
object_id
class_label
horizon_dt
components[K]:
  weight
  intent_label
  states[H]:
    mean_position[3]
    mean_velocity[3]
    covariance_6x6
```

这里的 `components[K]` 对应当前 LDOP 的 GMM 分支。每个 component 内部是一条中心状态序列和对应协方差序列，不是多条原始轨迹样本；分支数量裁剪参数作用在 `components[K]` 上。单条“中心预测轨迹”建议默认用 `MAP component`，同时另给 `mixture mean`。不要用 mixture mean 替代概率分布本体，否则多峰预测会被平均成一条不可实现轨迹。当前 LDOP 不把 `samples[N]` 列为 predictor 默认输出；仅可视化采样轨迹已删除，后续也不作为默认接口恢复。

## Verification Plan

Primary metrics:

- `minADE / minFDE`: 多模态预测必须用 minimum-over-modes。
- `MAP ADE/FDE`: 检查最高概率轨迹是否可信。
- `NLL`: 地面真值在 GMM 下的负对数似然。
- `coverage error`: 置信椭球覆盖率是否接近标称概率。
- `ECE/Brier`: intent probability 是否校准。
- `runtime mean/p95`: 随目标数量和 mixture 数增长的耗时。
- `NIS consistency`: 闭环不确定性是否过度自信或过度保守。

Stress tests:

- 突然停止、突然转向。
- 两目标交叉穿越。
- 狭窄通道会车。
- 遮挡后重现。
- 靠近静态障碍物绕行。
- 目标数量增加时的耗时曲线。

## Baseline and Ablation Matrix

| Purpose | Method | Why Needed | Expected Outcome |
|---|---|---|---|
| RAL-25 对齐 | MDP intent + sample mean/std | 直接证明相对改进 | GMM 在 NLL/coverage 更好 |
| 最弱基线 | CV/CA Kalman extrapolation | 检查复杂度是否必要 | 直线场景接近，交互场景变差 |
| 运动模型基线 | IMM-KF without interaction | 分离意图更新贡献 | 比 CV 好，但交互场景不足 |
| 去闭环 | proposed without NIS feedback | 验证不确定性反馈 | 校准与 abrupt change 变差 |
| 去交互 | proposed without pairwise energy | 验证主观能动性建模 | 会车/穿越场景变差 |
| 去地图 | proposed without map prior | 验证地图约束 | 靠墙/狭窄通道预测变差 |
| 可选强基线 | Trajectron++/AgentFormer 类学习模型 | 若有数据，可作上界参考 | 精度可能强，但部署成本高 |

## Expected Contributions

1. 提出一种面向 UAV 动态环境的轻量 trajectory-level GMM 预测表示，避免 RAL-25 式均值轨迹压缩造成多峰丢失。
2. 提出一种基于预测残差的闭环不确定性更新机制，将上一轮预测误差反馈到下一轮 mode probability 和 process noise。
3. 提出一种非学习式、可在线运行的交互意图能量项，同时考虑目标间交互和目标-地图交互。

## Reviewer Attack Simulation

| Potential Attack | Defense Needed | Experiment or Evidence |
|---|---|---|
| “只是 IMM + cost 的组合” | 强调闭环 residual-to-probability/covariance 更新和 trajectory-level GMM 输出 | 去闭环、去 GMM、去交互 ablation |
| “交互能量是手工规则” | 说明其定位是无训练、可解释、低时延在线预测 | 不同目标密度、会车、穿越压力测试 |
| “比学习方法弱” | 定位为轻量、无训练、可解释、适合 UAV 在线预测 | 与学习模型作参考上界，而非唯一对手 |
| “只改善 NLL，不改善 ADE” | 说明多模态任务不能只看均值误差，必须看概率校准和 coverage | 同时报告 `minADE/FDE`、`NLL`、coverage、ECE |
| “3D 意图太复杂” | 先实现 `2.5D`: xy intent + z climb/descend/hold 三类 | 做是否启用 z mode 的 ablation |

## Risks and Fallbacks

- 若交互能量调参困难，可先保留 `TTC + collision cone + map distance` 三个最小项。
- 若 GMM 的协方差传播过慢，可先使用 block-diagonal covariance，仅保留位置-速度关键相关项。
- 若多目标联合预测复杂度过高，可采用 pairwise energy 的 mean-field 近似，只迭代 1-2 次。
- 若闭环不确定性导致过度保守，可限制 `α_z(t)` 的上下界，并用 coverage error 选择阈值。

## Minimum Viable Experiment

1. 用现有 tracker 输出录制若干动态目标轨迹片段。
2. 对每一帧用过去 `1-2s` 预测未来 `1-3s`。
3. 比较 `CV/CA`、RAL-25-style MDP、IMM-KF、proposed。
4. 若 proposed 的 `NLL` 和 `coverage error` 没有改善，只保留交互模块为 ablation，不作为主贡献。

## Paper-Ready Methodology Draft

本文提出一种闭环不确定性交互意图混合预测方法，用于动态环境中 UAV 感知到的目标短时轨迹预测。与 RAL-25 Intent-MPC 中基于 MDP 的离散意图预测不同，本方法不将多条采样轨迹压缩为单一均值轨迹，而是维护 intent-conditioned trajectory-level Gaussian mixture。每个 mixture component 表示一种可解释运动意图及其时序状态协方差，component weight 由历史运动特征、地图可达性、目标间交互代价和上一轮预测残差共同递归更新。该设计使预测器能够在每一帧利用真实观测对不同意图的概率和不确定性进行闭环校准，从而同时输出可用于可视化的真实分支轨迹、可直接消费的多模态概率分布，以及可量化的预测置信度。

## 参考来源

- Intent Prediction-Driven Model Predictive Control for UAV Planning and Navigation in Dynamic Environments: <https://arxiv.org/abs/2409.15633>
- Intent-MPC GitHub repository: <https://github.com/Zhefan-Xu/Intent-MPC>
- Trajectron++: <https://arxiv.org/abs/2001.03093>
- AgentFormer: <https://arxiv.org/abs/2103.14023>
- EvolveGraph: <https://arxiv.org/abs/2003.13924>
- Bayesian intention inference and trajectory prediction: <https://arxiv.org/abs/2509.24928>
- Gaussian trajectory uncertainty calibration: <https://arxiv.org/abs/2603.10407>
