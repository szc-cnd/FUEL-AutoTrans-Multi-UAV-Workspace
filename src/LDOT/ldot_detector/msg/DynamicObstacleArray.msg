# 2026-07-27: 按传感器帧发布全部动态目标及短期预测，作为后续规划模块的只读数据接口。
std_msgs/Header header
float64 prediction_horizon
float64 prediction_dt
DynamicObstacle[] obstacles
