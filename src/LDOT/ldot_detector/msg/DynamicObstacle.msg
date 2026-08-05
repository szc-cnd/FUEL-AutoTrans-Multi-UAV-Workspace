# 2026-07-27: 新增稳定的结构化动态障碍物消息，供检测/预测上层使用，不包含控制或避障指令。
uint32 id
string obstacle_type
# 2026-07-27: 暴露确认/coasting状态和跟踪质量，后续规划器无需猜测目标可靠性。
uint8 tracking_state
float32 dynamic_confidence
uint32 age_frames
uint32 missed_frames
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 size
uint32 state_dim
float64[] state
float64[] covariance
geometry_msgs/Point[] predicted_positions
geometry_msgs/Vector3[] predicted_velocities
geometry_msgs/Vector3[] predicted_position_covariances
