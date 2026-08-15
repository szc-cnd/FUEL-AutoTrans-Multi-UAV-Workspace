# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

LDOP（Local Dynamic Object Processing）是 UAV 自主飞行系统的动态环境感知模块，基于 ROS Noetic。核心职责：从 LiDAR 点云中分离静态/动态点，聚类并跟踪动态目标，预测其短时轨迹。与 `rog_map`（静态地图）和规划器协同工作。

## 构建与测试

工作区根目录：`/home/st/ldop_ws`

```bash
# 构建
cd /home/st/ldop_ws
catkin build ldop --no-status

# 运行全部测试
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop

# 运行单个测试（以 predictor 为例）
rosrun ldop test_dynamic_object_predictor
```

完整 Gazebo + PX4 联合仿真仅用于最终验证，不作为日常测试：
```bash
./src/LDOP/launch/gazebo_px4_LDOP.sh
```

## 技术栈

- **C++20**（严格模式，无 GNU 扩展），编译选项 `-Wall -Wextra -Wpedantic`
- **ROS Noetic**（catkin 构建系统）
- **外部依赖**：Eigen3、UFOMap（位于 `${WORKSPACE_ROOT}/external/install/ufomap`）、OpenMP、TBB、GTest
- **clangd**：`.clangd` 配置已移除 `-fopenmp` 和 `-DUFO_PARALLEL=1` 以避免误报

## 架构

### 数据流管线

```
LiDAR 点云 + 里程计
  → UfomapMapper（静态/动态点分离，UFOMap 维护）
  → DynamicObjectClusterer（动态点聚类为检测框）
  → DynamicObjectTracker（跨帧关联、跟踪、分类、多模型 KF/EKF）
  → DynamicObjectPredictor（GMM 多分支轨迹预测 + 交互感知 + 在线反馈）
  → DynamicObjectPoseFusion（动捕-LiDAR 融合，B-spline 插值，虚拟 IMU）
```

### 运行时结构

- `Ldop` 类（`ldop.h/cpp`）是核心调度器，拥有所有子模块的 `unique_ptr`
- ROS 回调 → 输入队列（mutex + condition variable）→ 单处理线程 FIFO 消费 → 同线程发布
- `AsyncSpinner` 处理 ROS 回调

### 模块文件映射

| 模块 | 头文件 | 实现 |
|------|--------|------|
| 核心调度 | `include/ldop/ldop.h` | `src/ldop.cpp` |
| UFOMap 建图 | `include/ldop/ufomap_mapper.h` | `src/ufomap_mapper.cpp` |
| 动态点聚类 | `include/ldop/dynamic_object_clusterer.h` | `src/dynamic_object_clusterer.cpp` |
| 目标跟踪 | `include/ldop/dynamic_object_tracker.h` | `src/dynamic_object_tracker.cpp` |
| 轨迹预测 | `include/ldop/dynamic_object_predictor.h` | `src/dynamic_object_predictor.cpp` |
| 交互感知 | `include/ldop/prediction_interaction_context.h` | `src/prediction_interaction_context.cpp` |
| 动捕融合 | `include/ldop/dynamic_object_pose_fusion.h` | `src/dynamic_object_pose_fusion.cpp` |
| 卡尔曼滤波 | `include/ldop/multi_model_kalman_filter.h` | `src/multi_model_kalman_filter.cpp` |
| 运动模型 | `include/ldop/motion_model.h` | `src/motion_model.cpp` |
| 公共类型 | `include/ldop/utils.h` | — |

所有代码在 `ldopcore` 命名空间下。

### 关键设计模式

- **Params → Config**：每个模块有 `*Params` 结构体（ROS 参数快照含默认值）和 `*Config` 结构体（运行时校验），通过 `build*Config()` 转换
- **FrameResult**：每个模块的主函数返回 `*FrameResult` 结构体，包含所有输出（消息、标记、计时统计）
- **计时埋点**：每个模块有 `*TimingStats` 结构体，记录每帧毫秒级耗时
- **运动模型**：`MotionModel` 抽象基类，子类 `CAMotionModel`、`CVMotionModel`、`CTRAMotionModel`；`model_state` 布局随 `motion_model_type` 变化（CA2D/CA3D/CV3D/CTRA）
- **线程安全**：输入队列用 mutex + condition variable；UFOMap 用 `shared_mutex`（读写锁）；`DynamicObjectPoseFusion` 有独立内部 mutex

### 运动模型状态布局

`DynamicObject.msg` 中 `model_state` 前三维固定为 `x/y/z`，其余字段按 `motion_model_type` 解释：
- `CA2D`：`[x,y,z,vx,vy,ax,ay]`
- `CA3D`：`[x,y,z,vx,vy,vz,ax,ay,az]`
- `CV3D`：`[x,y,z,vx,vy,vz]`
- `CTRA`：`[x,y,z,v,a,yaw,yaw_rate]`

## 配置

运行时参数集中在 `config/ldop.yaml`，主要分组：输入输出话题、UFOMap、聚类、跟踪、预测、分类、动捕融合、卡尔曼滤波器参数。

## 文档

- `README.md`：设计总结、阶段状态、模块边界、对外接口
- `docs/design/`：设计文档（四阶段路线图等）
- `docs/explanntion/`：数学/算法解释文档
- `docs/superpowers/`：已完成设计、实施计划和规格说明

## 语言约定

代码注释和文档使用中文，技术术语、库 API、公式变量名保留英文。

## Agent skills

### Issue tracker

Local markdown — issues are tracked as files under `.scratch/<feature>/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Five canonical roles: needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context — one `CONTEXT.md` + `docs/adr/` at repo root. See `docs/agents/domain.md`.
