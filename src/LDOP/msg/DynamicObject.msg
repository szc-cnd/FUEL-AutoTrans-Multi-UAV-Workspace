uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint8 MOTION_MODEL_CA2D=0
uint8 MOTION_MODEL_CA3D=1
uint8 MOTION_MODEL_CV3D=2
uint8 MOTION_MODEL_CTRA=3

uint32 id
# 当前 bbox 尺寸；中心位置在 model_state 的前三维中。
geometry_msgs/Vector3 size
# 当前运动模型的完整状态向量；不同 motion_model_type 使用各自模型布局。
float64[] model_state
# 当前运动模型状态协方差矩阵 P，按 row-major 展开，维度为 len(model_state)^2。
float64[] model_covariance
uint8 object_class
uint8 motion_model_type
