# 🔧 Troubleshooting Guide - 故障排除指南

## 🚨 常见问题快速解决

### 📋 问题分类索引

- [🚀 启动问题](#启动问题)
- [🤖 机器人控制问题](#机器人控制问题)  
- [🎮 仿真环境问题](#仿真环境问题)
- [📡 通信问题](#通信问题)
- [💻 性能问题](#性能问题)
- [🔧 配置问题](#配置问题)

---

## 🚀 启动问题

### ❌ 问题1: Gazebo启动失败

**症状**:
```
[ERROR] [gazebo]: process has died
[ERROR] Could not load library: libgazebo_ros_api_plugin.so
```

**解决方案**:
```bash
# 1. 检查Gazebo安装
gazebo --version

# 2. 重新安装Gazebo ROS包
sudo apt-get install --reinstall ros-$ROS_DISTRO-gazebo-ros-pkgs

# 3. 清理Gazebo缓存
rm -rf ~/.gazebo/log
rm -rf ~/.gazebo/models/.database

# 4. 设置环境变量
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find uuv_gazebo_worlds)/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(rospack find underwater_coverage_planning)

# 5. 重新启动
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch
```

### ❌ 问题2: ROS Master连接失败

**症状**:
```
[ERROR] Unable to contact ROS master at [http://localhost:11311]
```

**解决方案**:
```bash
# 1. 检查roscore是否运行
ps aux | grep roscore

# 2. 启动roscore
roscore &

# 3. 检查网络配置
echo $ROS_MASTER_URI
echo $ROS_IP

# 4. 重置网络配置
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# 5. 重新启动系统
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch
```

### ❌ 问题3: 包找不到错误

**症状**:
```
[rospack] Error: package 'underwater_coverage_planning' not found
```

**解决方案**:
```bash
# 1. 检查包路径
rospack find underwater_coverage_planning

# 2. 重新编译工作空间
cd ~/catkin_ws
catkin_make

# 3. 重新source环境
source devel/setup.bash

# 4. 检查包是否在ROS_PACKAGE_PATH中
echo $ROS_PACKAGE_PATH | grep underwater_coverage_planning

# 5. 如果仍然失败，手动添加路径
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src
```

---

## 🤖 机器人控制问题

### ❌ 问题4: 机器人不响应控制命令

**症状**:
- 机器人模型加载成功但不移动
- 控制命令发布但无效果

**诊断步骤**:
```bash
# 1. 检查推进器管理器状态
rosnode info /rexrov/thruster_manager

# 2. 检查控制话题
rostopic list | grep cmd_vel
rostopic echo /rexrov/cmd_vel

# 3. 检查推进器输出
rostopic echo /rexrov/thrusters/0/input
```

**解决方案**:
```bash
# 1. 重启推进器管理器
rosnode kill /rexrov/thruster_manager
roslaunch uuv_thruster_manager thruster_manager.launch uuv_name:=rexrov model_name:=rexrov

# 2. 检查机器人模型配置
rosrun xacro xacro $(rospack find underwater_coverage_planning)/robots/rexrov_default.xacro

# 3. 手动发送测试命令
rostopic pub /rexrov/cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}" -r 10
```

### ❌ 问题5: 轨迹跟踪精度差

**症状**:
- 机器人偏离预定轨迹
- 位置误差过大

**解决方案**:
```bash
# 1. 调整PID参数
rosparam set /trajectory_controller/position/kp "[15.0, 15.0, 15.0]"
rosparam set /trajectory_controller/position/ki "[0.2, 0.2, 0.2]"
rosparam set /trajectory_controller/position/kd "[2.0, 2.0, 2.0]"

# 2. 降低最大速度
rosparam set /trajectory_controller/max_forward_speed 1.0

# 3. 增加控制频率
# 修改trajectory_publisher.py中的publish_rate参数
```

**PID调参指南**:
```python
# PID参数调整原则
# Kp (比例): 增加响应速度，但过大会震荡
# Ki (积分): 消除稳态误差，但过大会不稳定  
# Kd (微分): 减少超调，提高稳定性

# 推荐起始值
position_pid = {
    'kp': [10.0, 10.0, 10.0],  # 位置控制
    'ki': [0.1, 0.1, 0.1],
    'kd': [1.0, 1.0, 1.0]
}

orientation_pid = {
    'kp': [5.0, 5.0, 5.0],     # 姿态控制
    'ki': [0.05, 0.05, 0.05],
    'kd': [0.5, 0.5, 0.5]
}
```

---

## 🎮 仿真环境问题

### ❌ 问题6: 地形显示异常

**症状**:
- 地形模型不显示
- 高度图加载失败

**解决方案**:
```bash
# 1. 检查地形模型路径
ls $(rospack find uuv_gazebo_worlds)/models/sand_heightmap/

# 2. 重新下载地形模型
cd ~/.gazebo/models
wget -r -np -nH --cut-dirs=2 http://models.gazebosim.org/sand_heightmap/

# 3. 检查world文件配置
rosrun xacro xacro $(rospack find underwater_coverage_planning)/worlds/compact_underwater_terrain.world

# 4. 手动加载地形
rosservice call /gazebo/spawn_sdf_model "model_name: 'terrain'
model_xml: '$(cat $(rospack find uuv_gazebo_worlds)/models/sand_heightmap/model.sdf)'
robot_namespace: ''
initial_pose: {position: {x: 0, y: 0, z: -25}}"
```

### ❌ 问题7: 水下物理效果异常

**症状**:
- 机器人浮力异常
- 水阻力效果不正确

**解决方案**:
```bash
# 1. 检查UUV插件加载
rosservice call /gazebo/get_world_properties

# 2. 重新加载物理参数
rosparam load $(rospack find uuv_descriptions)/config/rexrov_default.yaml

# 3. 调整浮力参数
rosparam set /rexrov/buoyancy/fluid_density 1028.0  # 海水密度
rosparam set /rexrov/buoyancy/volume 1.83           # 机器人体积

# 4. 重启仿真
rosservice call /gazebo/reset_simulation
```

---

## 📡 通信问题

### ❌ 问题8: 话题数据丢失

**症状**:
```bash
rostopic echo /rexrov/pose_gt
# 无输出或数据不连续
```

**解决方案**:
```bash
# 1. 检查话题发布者
rostopic info /rexrov/pose_gt

# 2. 检查消息队列大小
rostopic echo /rexrov/pose_gt --queue-size=100

# 3. 检查网络延迟
rostopic delay /rexrov/pose_gt

# 4. 增加缓冲区大小
# 在launch文件中添加：
# <param name="queue_size" value="100"/>
```

### ❌ 问题9: 坐标系变换错误

**症状**:
- TF变换失败
- 坐标系不一致

**解决方案**:
```bash
# 1. 查看TF树
rosrun tf view_frames
evince frames.pdf

# 2. 检查特定变换
rosrun tf tf_echo world_ned rexrov/base_link

# 3. 发布缺失的变换
rosrun tf static_transform_publisher 0 0 0 0 0 0 world world_ned 100

# 4. 重新启动TF发布器
rosnode kill /robot_state_publisher
rosrun robot_state_publisher robot_state_publisher
```

---

## 💻 性能问题

### ❌ 问题10: 仿真运行缓慢

**症状**:
- 实时因子 < 0.5
- 画面卡顿严重

**解决方案**:
```bash
# 1. 降低物理更新频率
rosparam set /gazebo/max_step_size 0.02
rosparam set /gazebo/real_time_update_rate 50

# 2. 关闭不必要的传感器
# 在launch文件中注释掉相机等高负载传感器

# 3. 减少模型复杂度
# 使用简化的机器人模型
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch model_name:=rexrov_simple

# 4. 优化系统资源
# 关闭其他应用程序
# 增加虚拟内存
sudo sysctl vm.swappiness=10
```

### ❌ 问题11: 内存泄漏

**症状**:
- 内存使用持续增长
- 系统变慢或崩溃

**解决方案**:
```bash
# 1. 监控内存使用
watch -n 1 'ps aux | grep -E "(gazebo|ros)" | head -10'

# 2. 重启高内存使用的节点
rosnode kill /gazebo
roslaunch gazebo_ros empty_world.launch

# 3. 清理ROS日志
rosclean purge

# 4. 限制日志大小
export ROSCONSOLE_CONFIG_FILE=$(rospack find underwater_coverage_planning)/config/rosconsole.conf
```

---

## 🔧 配置问题

### ❌ 问题12: 参数加载失败

**症状**:
```
[ERROR] Could not load parameter /trajectory_planner/max_velocity
```

**解决方案**:
```bash
# 1. 检查参数文件路径
ls $(rospack find underwater_coverage_planning)/config/

# 2. 手动加载参数
rosparam load $(rospack find underwater_coverage_planning)/config/trajectory_params.yaml

# 3. 验证参数加载
rosparam list | grep trajectory_planner

# 4. 设置默认值
rosparam set /trajectory_planner/max_velocity 2.0
```

### ❌ 问题13: 轨迹文件格式错误

**症状**:
```
[ERROR] YAML parsing error in trajectory file
```

**解决方案**:
```bash
# 1. 验证YAML格式
python -c "import yaml; yaml.safe_load(open('trajectory.yaml'))"

# 2. 检查文件编码
file trajectory.yaml

# 3. 转换文件格式
dos2unix trajectory.yaml

# 4. 使用示例文件
cp $(rospack find underwater_coverage_planning)/scripts/guiji/trajectory_20250720_121855_ros.yaml my_trajectory.yaml
```

---

## 🛠️ 高级调试工具

### 系统诊断脚本

创建 `diagnose_system.sh`:
```bash
#!/bin/bash
echo "🔍 系统诊断报告"
echo "=================="

echo "📅 时间: $(date)"
echo "🖥️  系统: $(uname -a)"
echo "🐧 ROS版本: $ROS_DISTRO"

echo -e "\n📦 包状态:"
rospack find underwater_coverage_planning && echo "✅ 主包已找到" || echo "❌ 主包未找到"
rospack find uuv_simulator && echo "✅ UUV Simulator已安装" || echo "❌ UUV Simulator未安装"

echo -e "\n🔗 网络配置:"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"

echo -e "\n🤖 节点状态:"
rosnode list 2>/dev/null | wc -l | xargs echo "活跃节点数:"
rosnode list 2>/dev/null | grep -E "(gazebo|rexrov|trajectory)" || echo "关键节点未运行"

echo -e "\n📡 话题状态:"
rostopic list 2>/dev/null | grep -E "(pose_gt|trajectory|cmd_vel)" | wc -l | xargs echo "关键话题数:"

echo -e "\n💾 资源使用:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "内存: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
echo "磁盘: $(df -h / | awk 'NR==2{printf "%s", $5}')"

echo -e "\n🔧 建议操作:"
if ! pgrep -f roscore > /dev/null; then
    echo "❗ 启动roscore: roscore &"
fi

if ! pgrep -f gazebo > /dev/null; then
    echo "❗ 启动Gazebo: roslaunch underwater_coverage_planning compact_terrain_rexrov.launch"
fi

echo -e "\n✅ 诊断完成"
```

### 自动修复脚本

创建 `auto_fix.sh`:
```bash
#!/bin/bash
echo "🔧 自动修复系统问题..."

# 清理进程
echo "🧹 清理旧进程..."
killall -9 gzserver gzclient 2>/dev/null
rosnode kill -a 2>/dev/null

# 清理缓存
echo "🗑️  清理缓存..."
rm -rf ~/.gazebo/log/*
rosclean purge -y

# 重置环境
echo "🔄 重置环境变量..."
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find uuv_gazebo_worlds)/models
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# 重新编译
echo "🔨 重新编译工作空间..."
cd ~/catkin_ws
catkin_make

echo "✅ 自动修复完成，请重新启动系统"
```

---

## 📞 获取更多帮助

### 🌐 在线资源

- **官方文档**: https://uuvsimulator.github.io/
- **ROS Wiki**: http://wiki.ros.org/
- **Gazebo教程**: http://gazebosim.org/tutorials
- **社区论坛**: https://answers.ros.org/

### 📧 报告问题

提交问题时请包含以下信息：

1. **系统信息**:
   ```bash
   uname -a
   echo $ROS_DISTRO
   gazebo --version
   ```

2. **错误日志**:
   ```bash
   tail -50 ~/.ros/log/latest/rosout.log
   ```

3. **节点状态**:
   ```bash
   rosnode list
   rostopic list
   ```

4. **重现步骤**: 详细描述如何重现问题

### 🔧 自助调试清单

- [ ] 检查ROS环境变量设置
- [ ] 验证所有依赖包已安装
- [ ] 确认Gazebo版本兼容性
- [ ] 检查系统资源使用情况
- [ ] 查看ROS日志文件
- [ ] 尝试重启相关节点
- [ ] 使用诊断脚本检查系统状态

---

**💡 提示**: 大多数问题都可以通过重启相关节点或重置环境变量来解决。如果问题持续存在，请参考上述详细解决方案或寻求社区帮助。