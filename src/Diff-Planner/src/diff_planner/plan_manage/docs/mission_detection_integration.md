# 三类目标检测接入规划器

## 目标

比赛挑战赛道要求无人机在狭窄通道内完成彩色标签、二维码和热异常点检测，并给出目标在赛道坐标系中的位置。检测算法和坐标转换由 `db_ws/target_reporting` 独立完成，本工作空间的 `mission_detection_bridge` 将其 `channel` 坐标观测分成“候选观察”和“稳定确认”两条规划器接口。

桥接关系如下：

| 类型 | `target_reporting` 来源字段 | 消息类型 | 规划器话题 |
| --- | --- | --- | --- |
| 彩色标签 | `target_type=color_tag` | `std_msgs/String` `/target_reporting/observation` | 候选 `/UAV0/mission/detection/candidate/color`；确认 `/UAV0/mission/detection/color` |
| 二维码 | `target_type=qr_code` | `std_msgs/String` `/target_reporting/observation` | 候选 `/UAV0/mission/detection/candidate/qrcode`；确认 `/UAV0/mission/detection/qrcode` |
| 热异常点 | `target_type=thermal_source` | `std_msgs/String` `/target_reporting/observation` | 候选 `/UAV0/mission/detection/candidate/thermal`；确认 `/UAV0/mission/detection/thermal` |

三个检测包先分别发布首个有效的原始候选；`target_reporting` 负责候选坐标的 TF 转换、实时远程上报和空间连续命中确认。桥接节点只消费 `/target_reporting/observation`，要求 `position.frame_id=channel`、`position.unit=m`。`confirmed=false` 的首帧/后续帧只发布候选点，`confirmed=true` 后再发布锁存的确认点。

桥接后的规范化记录发布到 `/UAV0/mission/detection/report`，类型为 `std_msgs/String`。本工程默认 `world` 就是比赛通道坐标系，因此 JSON 中的 `corridor_x/y/z` 与 `world x/y/z` 完全相同，不再做入口原点平移或二次旋转。

## 坐标和抗误检处理

坐标转换和相机外参只在 `target_reporting` 中配置；规划器侧不再配置相机平移、旋转或里程计同步参数。桥接节点仅检查有限数值、`channel` 帧和稳定命中次数，并将结果锁存为 `world` 帧的 `PoseStamped`。

## 任务状态约束

三类检测结果用于比赛记分、目标位置登记和 RViz 标记，但默认不作为出口硬门槛：`require_stage2_detections=false`。即使有目标漏检，只要搜索覆盖和出口几何证据满足条件，规划器仍会正常出通道。最终降落二维码和精确降落仍按原有独立流程执行。

规划器同时启用了相机视角航向和小幅机头扫视，以提高走廊搜索时三个检测器看到目标的概率，不执行原地整周旋转。

## 检测—规划闭环

收到首个候选点后，FUEL 在 `SEARCH_CORRIDOR` 阶段暂停当前普通 frontier 目标，围绕候选点按 0.90--1.80 m 半径采样观察位姿，并让机头朝向目标。候选目标本身不会直接作为飞行点，避免撞向目标或墙面；无人机移动到安全观察位姿后，检测包继续提供后续帧供 `target_reporting` 确认。

观察位姿必须依次通过累计 SDF 的自由/膨胀占据检查、机体足迹检查、任务单向通道约束、A* 可达性和最终轨迹安全检查；这些检查任一失败，就进入失败冷却并尝试另一个环位置，最多三次后放弃该候选，恢复普通搜索。候选超过 2.5 s 未更新也会自动失效。

确认话题一旦到达，候选观察任务立即清除并登记目标，不再为该目标生成任何飞行目标；三个目标不全时仍允许继续搜索、确认出口和出通道，漏检只影响比赛得分，不会把规划器锁死。

## 启动

先在 `db_ws` 启动三个检测节点、FAST-LIO 和 `target_reporting`，再启动规划器：

```bash
cd /home/oem/db_ws
source devel/setup.bash
# 按现有设备启动流程启动 color_tag_detector、qr_detector、uvc_ubuntu、FAST-LIO
roslaunch target_reporting target_reporting.launch

cd /home/oem/match_ws
source devel/setup.bash
roslaunch diff_planner run_swarm_indoor1_fuel_exploration.launch
```

如果通过其他入口 include 此文件，保证 `enable_mission_detection_bridge=true`。如需改桥接输入，只需覆盖 `detection_observation_input`，默认是 `/target_reporting/observation`。

## 现场检查

```bash
rostopic echo /target_reporting/observation
rostopic echo /UAV0/mission/detection/candidate/color
rostopic echo /UAV0/mission/detection/candidate/qrcode
rostopic echo /UAV0/mission/detection/candidate/thermal
rostopic echo /UAV0/mission/detection/report
rostopic echo /mission/task_status
```

每种目标被 `target_reporting` 观测后，桥接节点会先在对应 `candidate` 话题输出；累计确认后输出 `CONFIRMED`，对应规划器确认话题会保留最后一次结果。若没有进入规划器，先检查 `/target_reporting/observation` 的 `position.frame_id` 是否为 `channel`、`unit` 是否为 `m`，以及 `candidate` 话题是否有消息。

精确降落仍由 `/home/oem/db_ws/src/precision_landing` 的现有链路执行，本次接入没有修改其控制接口。
