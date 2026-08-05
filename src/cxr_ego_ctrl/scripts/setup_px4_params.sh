#!/bin/bash

# PX4参数配置脚本
# 用于配置PX4参数以支持OFFBOARD模式和自动起飞

echo "等待PX4启动..."
sleep 5

echo "配置PX4参数..."

# 配置手动控制参数
# 允许在没有GPS的情况下解锁
px4param set COM_ARM_WO_GPS 1

# 配置RC输入参数
# 允许RC覆盖
px4param set COM_RCL_EXCEPT 4

# 配置安全模式参数
# 设置OFFBOARD模式为可用
px4param set COM_FLTMODE1 1
px4param set COM_FLTMODE2 2
px4param set COM_FLTMODE3 4
px4param set COM_FLTMODE4 6

# 配置起飞参数
# 设置最大上升速度
px4param set MPC_Z_VEL_MAX_UP 3.0

# 设置最大下降速度
px4param set MPC_Z_VEL_MAX_DN 2.0

# 设置位置控制P增益
px4param set MPC_XY_P 2.0

# 设置高度控制P增益
px4param set MPC_Z_P 2.0

# 设置最小油门值
px4param set MPC_MANTHR_MIN 0.2

# 设置最大油门值
px4param set MPC_MANTHR_MAX 0.9

# 设置悬停油门值
px4param set MPC_THR_HOVER 0.5

# 配置电池参数
# 设置电池容量
px4param set BAT_CAPACITY 2000

# 设置电池低电量警告阈值
px4param set BAT_CRIT_THR 0.15

# 设置电池低电量警告阈值
px4param set BAT_LOW_THR 0.20

# 配置预飞检查参数
# 禁用某些预飞检查，避免自动解除解锁
px4param set COM_PREARM_MODE 1

# 配置位置估计参数
# 使用GPS和视觉里程计
px4param set EKF2_AID_MASK 24

# 设置气压计使用
px4param set EKF2_BARO_CTRL 1

# 配置电机参数
# 设置电机最小PWM值
px4param set PWM_MIN 1000

# 设置电机最大PWM值
px4param set PWM_MAX 2000

echo "PX4参数配置完成！"