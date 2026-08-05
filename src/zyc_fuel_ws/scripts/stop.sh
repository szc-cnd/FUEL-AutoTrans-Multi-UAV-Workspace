#!/bin/sh

# 杀死与指定命令关联的进程
for pattern in \
    "sh shfiles/rspx4.sh" \
    "vins_to_mavros_node" \
    "pose_to_odom_converter_node" \
    "roslaunch exploration_manager exploration.launch" \
    "roslaunch yolo_detector yolo_ros.launch" \
    "roslaunch sort_ros sort_ros.launch" \
    "python3 marker_wenzi_jisuan.py" \
    "rosrun exploration_manager fuel_nav"
do
    if pgrep -f "$pattern" > /dev/null; then
        pkill -9 -f "$pattern" 2>/dev/null || true
        echo "已终止进程：$pattern"
    else
        echo "未找到进程：$pattern"
    fi
done

# 清理 ROS 节点
if rosnode list > /dev/null 2>&1; then
    rosnode kill --all 2>/dev/null || true
    echo "已清理所有 ROS 节点。"
else
    echo "未检测到活跃的 ROS 节点。"
fi

echo "所有指定进程已关闭。"