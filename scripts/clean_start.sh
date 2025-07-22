#!/bin/bash

echo "正在清理之前的进程..."

# 关闭所有ROS相关进程
pkill -f "roslaunch\|roscore\|rosmaster\|rosout" 2>/dev/null
pkill -f "robot_state_publisher\|thruster_allocator" 2>/dev/null
pkill -f "publish_world_models\|ground_truth_to_tf" 2>/dev/null

# 关闭Gazebo进程
pkill -f "gazebo\|gzserver\|gzclient" 2>/dev/null

# 关闭RViz进程
pkill -f "rviz" 2>/dev/null

# 等待进程完全关闭
sleep 3

echo "进程清理完成！"

# 检查是否还有残留进程
remaining=$(ps aux | grep -E "(ros|gazebo|rviz)" | grep -v grep | wc -l)
if [ $remaining -gt 0 ]; then
    echo "警告：还有 $remaining 个进程未关闭，强制关闭..."
    ps aux | grep -E "(ros|gazebo|rviz)" | grep -v grep | awk '{print $2}' | xargs -r kill -9
    sleep 2
fi

echo "现在启动新的仿真环境..."

# 启动仿真
roslaunch underwater_coverage_planning complete_rexrov_control.launch &

# 等待系统启动
sleep 10

echo "启动完成！"
echo "使用键盘控制机器人："
echo "  W/S: 前进/后退"
echo "  A/D: 左转/右转"
echo "  X/Z: 上浮/下潜"
echo "  Q/E: 偏航"
echo "  I/K: 俯仰"
echo "  J/L: 横滚"
echo "  1/2: 调整速度"
echo "  Ctrl+C: 停止" 