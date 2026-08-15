# 动态目标 ID 稳定跟踪优化实施计划

> **For agentic workers:** 本计划在当前会话内执行；用户已明确要求直接调整并现场复测。步骤使用复选框记录。

**目标：** 让小球在遮挡和局部点云碎片下继续复用原轨迹 ID，并过滤固定杆的静止抖动。

**架构：** 在 tracker 内加入一次性动态确认状态；确认后的轨迹使用近距离碎片回关联和受限重复合并保持 ID。RViz 改为默认显示 tracker 确认后的稳定框，原始候选框话题继续保留用于排查。

**技术栈：** C++、ROS、Eigen、RViz、YAML、Catkin。

## 全局约束

- 不安装新的依赖。
- 动态避障接口继续使用 `/ldop/dynamic_objects`。
- 本地 `E:/比赛功能包/LDOP` 与远程 `/home/oem/ldop_ws/src/LDOP` 同步修改。
- Git 提交备注使用中文。
- 按用户要求不运行完整单元测试，执行远程 Catkin 编译和参数/差异核对。

## 任务 1：跟踪器动态确认与碎片回关联

**文件：**
- 修改：`include/ldop/dynamic_object_tracker.h`
- 修改：`src/dynamic_object_tracker.cpp`
- 修改：`config/ldop.yaml`

- [x] 根据现场日志确认 ID5/12/13 是主轨迹，ID10/15/17/19 是近邻局部碎片，ID7 是低速固定候选。
- [x] 增加 `motion_confirmed` 状态和运动速度参数。
- [x] 已确认动态轨迹不再因局部框尺寸过小而新建 ID。
- [x] 将重复合并距离提高到覆盖 0.5～0.7 m 现场碎片，并限制合并条件。
- [x] 统一 `/ldop/dynamic_objects` 和预测输入的发布过滤条件。

## 任务 2：稳定轨迹可视化

**文件：**
- 修改：`src/dynamic_object_tracker.cpp`
- 修改：`launch/rviz/ldop_rviz.rviz`

- [x] 为已确认动态轨迹发布稳定框 Marker。
- [x] 默认关闭原始候选框显示，避免固定杆继续显示红色原始框。
- [x] 默认保留 ID、轨迹头和预测显示。

## 任务 3：同步与验证

- [x] 将修改文件复制到远程源码对应路径。
- [x] 执行 `catkin build ldop --no-status`。
- [x] 核对远程参数、构建结果和本地/远程文件哈希。
- [ ] 提交本地中文 Git 记录，并把启动复测交给用户。
