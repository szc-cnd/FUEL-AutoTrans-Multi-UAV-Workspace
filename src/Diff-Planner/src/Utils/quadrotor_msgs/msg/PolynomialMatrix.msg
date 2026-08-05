# 一段三维多项式轨迹的系数矩阵。
uint32 num_order
uint32 num_dim

# 按 Eigen 列主序存储：每一列对应一个多项式阶次，每一行对应一个空间维度。
float64[] data
float64 duration
