# 动态障碍物重复轨迹合并 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 合并同一小球被点云分裂后形成的近邻重复轨迹，保持一个稳定的对外 ID。

**Architecture:** 在现有 `DynamicObjectTracker` 的当前帧更新、创建和 coast 流程之后增加一次轨迹级去重。两条三维中心距离不超过可配置阈值的轨迹只保留质量更高的一条，随后沿用现有过期删除、对外发布和预测输入流程。

**Tech Stack:** C++17、ROS Noetic、Eigen、Catkin、YAML 参数。

## Global Constraints

- 本次只修改跟踪器头文件、跟踪器实现和 `config/ldop.yaml`，不改地面滤波、点云聚类和地图逻辑。
- 新参数 `tracking_duplicate_merge_distance` 默认值为 `0.45 m`；设为非正数时关闭合并。
- 每次源码或配置修改同时同步到 `/home/oem/ldop_ws/src/LDOP`。
- 用户明确要求不运行自动测试；使用远程实时 ROS 话题进行验证。
- Git 提交备注使用中文。

---

### Task 1: 增加重复轨迹合并配置接口

**Files:**
- Modify: `include/ldop/dynamic_object_tracker.h`
- Modify: `src/dynamic_object_tracker.cpp`
- Modify: `config/ldop.yaml`

**Interfaces:**
- `DynamicObjectTrackerConfig::duplicate_merge_distance`：运行时三维中心合并距离，非正数表示关闭。
- `DynamicObjectTrackerParams::duplicate_merge_distance`：ROS 参数快照，默认 `0.45`。
- ROS 私有参数：`tracking_duplicate_merge_distance`。

- [ ] **Step 1: 在配置结构体中加入参数字段**

在 `DynamicObjectTrackerConfig` 和 `DynamicObjectTrackerParams` 中加入同名字段，默认值为 `0.45`，并写明该参数只用于近邻重复轨迹合并。

- [ ] **Step 2: 读取并构建参数**

在 `buildTrackerConfig()` 中保留正数配置，非正数转为关闭值；在 `loadParameters()` 中读取 `tracking_duplicate_merge_distance`。

- [ ] **Step 3: 写入 YAML 默认参数**

在跟踪参数区域增加：

```yaml
tracking_duplicate_merge_distance: 0.45  # 同一目标碎片轨迹的三维中心合并距离，<=0 关闭
```

- [ ] **Step 4: 同步本次配置接口到远程**

使用 SSH/SCP 将三个修改文件同步到远程源码目录，并用 SHA256 对比确认内容一致。

### Task 2: 实现轨迹质量比较与重复轨迹合并

**Files:**
- Modify: `include/ldop/dynamic_object_tracker.h`
- Modify: `src/dynamic_object_tracker.cpp`

**Interfaces:**
- 新增私有方法 `mergeDuplicateTracks()`，无参数、直接处理成员 `tracks_`。
- 方法在当前帧所有轨迹 update/create/coast 完成后调用，在 `deleteExpiredTracks()` 前执行。

- [ ] **Step 1: 定义保留轨迹规则**

对候选轨迹比较以下字段，按顺序优先保留：

1. `hits >= min_hits_to_publish` 的轨迹；
2. `hits` 更大的轨迹；
3. `missed_frames` 更少的轨迹；
4. `age` 更大的轨迹；
5. `id` 更小的轨迹。

- [ ] **Step 2: 实现近邻合并循环**

当合并距离大于零时，两两计算 `distance3D(track.bbox.center, other.bbox.center)`。距离不超过阈值时，按保留规则删除较弱轨迹；重复扫描直到本轮没有可合并轨迹，避免一次删除后留下新的可合并组合。

- [ ] **Step 3: 接入单帧处理流程**

在所有 detection 已关联、未关联 detection 已建轨迹、未匹配旧轨迹已 coast 后调用 `mergeDuplicateTracks()`，然后执行已有的过期删除和输出流程。若合并距离关闭，方法立即返回。

- [ ] **Step 4: 保持稳定 ID 和预测输入一致**

不复制或重置保留轨迹的 Kalman 状态、历史、尺寸平滑状态和 ID；被删除轨迹不得进入 `buildOutput()` 或 `buildPredictionInputs()`。

- [ ] **Step 5: 同步实现文件到远程**

同步头文件和实现文件，重新核对三类文件的 SHA256。

### Task 3: 远程构建和现场验证

**Files:**
- No additional source files.

**Interfaces:**
- 运行参数：`tracking_duplicate_merge_distance=0.45`。
- 对外话题：`/ldop/dynamic_objects`。

- [ ] **Step 1: 检查差异和编译前状态**

执行 `git diff --check`，确认只包含本次跟踪器修改；不运行自动单元测试。

- [ ] **Step 2: 远程编译**

执行：

```bash
source /home/oem/ldop_ws/ldop_env.sh
cd /home/oem/ldop_ws
catkin build ldop --no-status
```

预期：`ldop` 包构建成功。

- [ ] **Step 3: 重启单一 LDOP 实例并核对参数**

停止旧 LDOP 进程，启动一份新的 LDOP 检测节点，确认 `tracking_duplicate_merge_distance` 为 `0.45`，不保留重复启动实例。

- [ ] **Step 4: 现场观察 ID**

让小球运动并经历点数下降、短暂遮挡和重新出现，观察 `/ldop/dynamic_objects`：同一小球附近只保留一个轨迹 ID，重新出现时继续使用原 ID；目标离开后不长期输出预测残影。

- [ ] **Step 5: 提交实现**

```bash
git add config/ldop.yaml include/ldop/dynamic_object_tracker.h src/dynamic_object_tracker.cpp
git commit -m "修复：合并动态障碍物重复轨迹"
```
