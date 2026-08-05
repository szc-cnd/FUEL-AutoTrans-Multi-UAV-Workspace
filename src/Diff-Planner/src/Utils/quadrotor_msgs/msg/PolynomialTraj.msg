Header header

# AutoTrans 使用的轨迹编号，从 1 开始。
uint32 trajectory_id

# 轨迹动作命令。
uint32 ACTION_ADD           =   1
uint32 ACTION_ABORT         =   2
uint32 ACTION_WARN_START    =   3
uint32 ACTION_WARN_FINAL    =   4
uint32 ACTION_WARN_IMPOSSIBLE = 5
uint32 action

# 每个矩阵是一段三维多项式，系数按 Eigen 列主序存储。
quadrotor_msgs/PolynomialMatrix[] trajectory
