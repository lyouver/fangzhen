# 🚀 Quick Start Guide - Underwater Coverage Planning

## ⚡ 5分钟快速上手

### 前置条件检查

```bash
# 检查ROS安装
echo $ROS_DISTRO

# 检查UUV Simulator
rospack find uuv_simulator

# 检查Gazebo版本
gazebo --version
```

### 一键启动脚本

创建启动脚本 `start_system.sh`：

```bash
#!/bin/bash
echo "🌊 启动水下覆盖规划系统..."

# 设置环境
source ~/catkin_ws/devel/setup.bash

# 启动基础系统
echo "📡 启动Gazebo环境和RexROV机器人..."
gnome-terminal --tab --title="Base System" -- bash -c "
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch;
exec bash"

# 等待系统初始化
echo "⏳ 等待系统初始化 (30秒)..."
sleep 30

# 启动轨迹控制
echo "🎯 启动轨迹控制系统..."
gnome-terminal --tab --title="Trajectory Control" -- bash -c "
roslaunch underwater_coverage_planning trajectory_20250720_121855_launch.launch;
exec bash"

echo "✅ 系统启动完成！"
echo "📺 请查看Gazebo和RViz窗口"
```

使用方法：
```bash
chmod +x start_system.sh
./start_system.sh
```

## 🎮 基本操作

### 1. 系统状态检查

```bash
# 检查所有节点是否运行
rosnode list

# 应该看到以下关键节点：
# /gazebo
# /rexrov/thruster_manager  
# /trajectory_publisher
# /trajectory_controller
# /rviz
```

### 2. 监控机器人状态

```bash
# 查看机器人位置
rostopic echo /rexrov/pose_gt

# 查看控制命令
rostopic echo /rexrov/cmd_vel

# 查看轨迹数据
rostopic echo /rexrov/trajectory
```

### 3. 可视化界面

#### Gazebo窗口
- 🌊 水下环境仿真
- 🤖 RexROV机器人模型
- 🏔️ 地形显示

#### RViz窗口  
- 📍 机器人实时位置
- 📈 轨迹路径显示
- 📊 传感器数据

## 🛠️ 常用命令

### 系统控制

```bash
# 暂停仿真
rosservice call /gazebo/pause_physics

# 恢复仿真  
rosservice call /gazebo/unpause_physics

# 重置仿真
rosservice call /gazebo/reset_simulation

# 停止所有节点
rosnode kill -a
```

### 数据记录

```bash
# 记录所有话题数据
rosbag record -a -O mission_$(date +%Y%m%d_%H%M%S).bag

# 记录特定话题
rosbag record /rexrov/pose_gt /rexrov/trajectory -O trajectory_data.bag

# 播放记录的数据
rosbag play mission_data.bag
```

### 参数调整

```bash
# 查看所有参数
rosparam list

# 修改最大速度
rosparam set /trajectory_controller/max_forward_speed 1.5

# 保存参数到文件
rosparam dump params.yaml

# 从文件加载参数
rosparam load params.yaml
```

## 🔧 快速故障排除

### 问题1: Gazebo启动失败

```bash
# 解决方案
killall gzserver gzclient
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find uuv_gazebo_worlds)/models
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch
```

### 问题2: 机器人不移动

```bash
# 检查推进器管理器
rosnode info /rexrov/thruster_manager

# 重启轨迹控制器
rosnode kill /trajectory_controller
roslaunch underwater_coverage_planning trajectory_20250720_121855_launch.launch
```

### 问题3: RViz显示异常

```bash
# 重置RViz配置
rosrun rviz rviz -d $(rospack find underwater_coverage_planning)/rviz/rexrov_default.rviz
```

## 📊 性能监控

### 实时监控脚本

创建 `monitor.sh`：

```bash
#!/bin/bash
echo "🔍 系统性能监控"

while true; do
    clear
    echo "=== 系统状态 ==="
    echo "时间: $(date)"
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "内存: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
    
    echo -e "\n=== ROS节点状态 ==="
    rosnode list | wc -l | xargs echo "活跃节点数:"
    
    echo -e "\n=== 话题频率 ==="
    timeout 3 rostopic hz /rexrov/pose_gt 2>/dev/null | grep "average rate" || echo "位置更新: 无数据"
    timeout 3 rostopic hz /rexrov/trajectory 2>/dev/null | grep "average rate" || echo "轨迹更新: 无数据"
    
    echo -e "\n按Ctrl+C退出监控"
    sleep 5
done
```

## 🎯 任务示例

### 示例1: 简单路径跟踪

```bash
# 1. 启动系统
./start_system.sh

# 2. 等待初始化完成
sleep 30

# 3. 开始执行轨迹
echo "开始执行预定义轨迹..."

# 4. 监控执行状态
rostopic echo /rexrov/pose_gt | head -20
```

### 示例2: 自定义轨迹

```python
#!/usr/bin/env python3
# custom_trajectory.py

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def create_simple_trajectory():
    """创建简单的方形轨迹"""
    traj = JointTrajectory()
    traj.header.frame_id = "world_ned"
    traj.joint_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    
    # 方形路径的四个角点
    waypoints = [
        [-200, -200, -22, 0, 0, 0],      # 起点
        [-150, -200, -22, 0, 0, 0],      # 右
        [-150, -150, -22, 0, 0, 0],      # 上
        [-200, -150, -22, 0, 0, 0],      # 左
        [-200, -200, -22, 0, 0, 0],      # 回到起点
    ]
    
    for i, wp in enumerate(waypoints):
        point = JointTrajectoryPoint()
        point.positions = wp
        point.velocities = [1.0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration(i * 10)  # 每10秒一个点
        traj.points.append(point)
    
    return traj

if __name__ == '__main__':
    rospy.init_node('custom_trajectory_publisher')
    pub = rospy.Publisher('/rexrov/trajectory', JointTrajectory, queue_size=1)
    
    rate = rospy.Rate(1)  # 1Hz
    traj = create_simple_trajectory()
    
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pub.publish(traj)
        rate.sleep()
```

运行自定义轨迹：
```bash
python custom_trajectory.py
```

## 📱 移动端监控

### 使用rqt进行远程监控

```bash
# 启动rqt图形界面
rqt

# 或者启动特定插件
rqt_plot /rexrov/pose_gt/pose/position/x:y:z
rqt_image_view /rexrov/camera/image_raw
```

### Web界面监控

```bash
# 安装rosbridge
sudo apt-get install ros-$ROS_DISTRO-rosbridge-suite

# 启动web服务器
roslaunch rosbridge_server rosbridge_websocket.launch

# 在浏览器中访问: http://localhost:9090
```

## 🎓 学习资源

### 推荐教程

1. **ROS基础**: http://wiki.ros.org/ROS/Tutorials
2. **Gazebo仿真**: http://gazebosim.org/tutorials
3. **UUV Simulator**: https://uuvsimulator.github.io/
4. **RViz可视化**: http://wiki.ros.org/rviz/Tutorials

### 实践项目

1. **修改轨迹参数**: 调整速度、路径间距
2. **添加新传感器**: 集成声纳、激光雷达
3. **优化控制算法**: 改进PID参数
4. **扩展环境**: 添加障碍物、海流

## 📞 获取帮助

### 社区支持

- **ROS Answers**: https://answers.ros.org/
- **Gazebo Community**: https://community.gazebosim.org/
- **GitHub Issues**: 项目仓库的Issues页面

### 调试技巧

```bash
# 详细日志输出
export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}'
roslaunch --screen underwater_coverage_planning compact_terrain_rexrov.launch

# 图形化调试
rqt_console  # 查看日志
rqt_graph    # 查看节点关系图
rqt_tf_tree  # 查看坐标变换树
```

---

🎉 **恭喜！你已经掌握了水下覆盖规划系统的基本使用方法。现在可以开始你的水下探索之旅了！**