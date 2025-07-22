# 🌊 Underwater Coverage Planning System

## 📋 项目概述

水下覆盖路径规划系统是一个基于ROS的完整水下机器人仿真平台，专为ROV（遥控水下机器人）任务设计。该系统集成了RexROV机器人模型、复杂水下环境仿真、智能路径规划算法和实时轨迹控制功能。

### 🎯 主要功能

- **完整的水下仿真环境**：基于Gazebo的真实水下物理仿真
- **RexROV机器人模型**：配备多传感器的专业水下机器人
- **智能路径规划**：自动生成覆盖路径和轨迹优化
- **实时轨迹控制**：PID控制器实现精确路径跟踪
- **可视化界面**：RViz实时显示和HTML交互式规划
- **模块化设计**：组件分离，易于扩展和维护

## 🏗️ 系统架构

```
underwater_coverage_planning/
├── launch/                     # 启动文件
│   └── compact_terrain_rexrov.launch
├── worlds/                     # Gazebo世界文件
│   ├── compact_underwater_terrain.world
│   └── underwater_heightmap.world
├── robots/                     # 机器人模型
│   ├── rexrov_default.xacro
│   └── rexrov_fixed.xacro
├── rviz/                      # RViz配置
│   ├── rexrov_default.rviz
│   └── rexrov_sensors.rviz
├── scripts/                   # Python脚本
│   ├── guiji/                 # 轨迹系统
│   ├── interactive_coverage_planner.py
│   ├── trajectory_to_ros.py
│   └── monitor_movement.py
└── README.md                  # 本文档
```

## 🚀 快速开始

### 环境要求

- **操作系统**: Ubuntu 18.04/20.04
- **ROS版本**: Melodic/Noetic
- **Python**: 3.6+
- **依赖包**: UUV Simulator, Gazebo, RViz

### 安装步骤

1. **克隆项目**
```bash
cd ~/catkin_ws/src
git clone <repository_url> underwater_coverage_planning
```

2. **安装依赖**
```bash
# 安装UUV Simulator
sudo apt-get install ros-$ROS_DISTRO-uuv-simulator

# 安装其他依赖
rosdep install --from-paths . --ignore-src -r -y
```

3. **编译项目**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 基本使用

#### 方法一：分步启动（推荐）

1. **启动基础环境**
```bash
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch
```

2. **启动轨迹控制**
```bash
roslaunch underwater_coverage_planning trajectory_20250720_121855_launch.launch
```



## 🎮 操作指南

### 系统启动流程

1. **环境初始化**（15-30秒）
   - Gazebo加载水下环境
   - RexROV机器人生成
   - 传感器系统激活

2. **轨迹系统启动**
   - 推进器管理器初始化
   - PID控制器配置
   - 轨迹发布器启动

3. **可视化界面**
   - RViz显示机器人状态
   - 路径可视化
   - 传感器数据显示

### 控制模式

#### 自动轨迹跟踪
- 系统自动执行预定义轨迹
- PID控制器确保精确跟踪
- 实时监控执行状态

#### 手动控制（可选）
- 键盘控制接口
- 实时姿态调整
- 紧急停止功能

## 📊 技术规格

### 机器人参数
- **模型**: RexROV
- **尺寸**: 1.4m × 0.9m × 0.6m
- **最大速度**: 2.0 m/s
- **工作深度**: 0-50m
- **传感器**: IMU, 压力传感器, 摄像头

### 环境参数
- **仿真区域**: 444m × 444m
- **水深范围**: 0-50m
- **地形**: 高度图地形
- **物理**: 真实水下动力学

### 控制参数
- **控制器**: PID控制器
- **更新频率**: 10Hz
- **路径精度**: ±0.5m
- **响应时间**: <1s

## 🔧 配置说明

### 坐标系统

系统使用统一的NED（North-East-Down）坐标系：
- **X轴**: 北方向（正值向北）
- **Y轴**: 东方向（正值向东）
- **Z轴**: 下方向（正值向下）

### 参数配置

#### 机器人参数
```xml
<arg name="uuv_name" default="rexrov"/>
<arg name="model_name" default="rexrov"/>
<arg name="use_ned_frame" default="true"/>
```

#### 初始位置
```xml
<arg name="x" value="-200"/>
<arg name="y" value="-200"/>
<arg name="z" value="-22"/>
```

#### 控制参数
```xml
<param name="max_forward_speed" value="2.0"/>
<param name="timeout" value="10.0"/>
<param name="publish_rate" value="10.0"/>
```

## 📈 轨迹系统

### 轨迹格式

系统使用YAML格式存储轨迹数据：

```yaml
header:
  frame_id: world_ned
joint_names: [x, y, z, roll, pitch, yaw]
points:
  - positions: [-200.0, -200.0, -22.0, 0.0, 0.0, 0.0]
    velocities: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
```

### 轨迹生成

1. **路径规划**
   - 覆盖路径算法
   - 地形适应性调整
   - 安全约束检查

2. **轨迹优化**
   - 速度平滑处理
   - 加速度限制
   - 时间参数化

3. **格式转换**
   - JSON到YAML转换
   - ROS消息格式适配
   - 可视化数据生成

## 🎨 可视化功能

### RViz显示

- **机器人模型**: 实时3D模型显示
- **传感器数据**: 点云、图像数据
- **路径可视化**: 规划路径和执行轨迹
- **状态信息**: 速度、姿态、位置

### HTML交互界面

- **3D地形显示**: 基于Plotly的交互式地形
- **路径编辑**: 拖拽式路径规划
- **参数调整**: 实时参数修改
- **结果导出**: 轨迹文件生成

## 🔍 监控与调试

### 系统监控

```bash
# 查看话题列表
rostopic list

# 监控机器人状态
rostopic echo /rexrov/pose_gt

# 查看轨迹数据
rostopic echo /rexrov/trajectory

# 监控控制命令
rostopic echo /rexrov/cmd_vel
```

### 调试工具

```bash
# 启动系统监控
rosrun underwater_coverage_planning monitor_movement.py

# 清理环境
./scripts/clean_start.sh

# 检查节点状态
rosnode list
rosnode info /trajectory_publisher
```

## 🛠️ 故障排除

### 常见问题

#### 1. Gazebo启动失败
```bash
# 检查Gazebo版本
gazebo --version

# 重置Gazebo
killall gzserver gzclient
rosclean purge
```

#### 2. 机器人不响应控制
```bash
# 检查推进器管理器
rosnode info /rexrov/thruster_manager

# 重启控制器
rosnode kill /trajectory_controller
roslaunch underwater_coverage_planning trajectory_20250720_121855_launch.launch
```

#### 3. 轨迹文件加载失败
```bash
# 检查文件路径
rospack find underwater_coverage_planning

# 验证YAML格式
python -c "import yaml; yaml.safe_load(open('trajectory.yaml'))"
```

### 性能优化

#### 1. 提高仿真性能
- 降低Gazebo物理更新频率
- 减少传感器数据发布频率
- 关闭不必要的可视化

#### 2. 优化控制响应
- 调整PID参数
- 增加控制器更新频率
- 优化轨迹平滑度

## 📚 API参考

### ROS话题

#### 发布话题
- `/rexrov/trajectory` - 轨迹数据
- `/rexrov/path_visualization` - 路径可视化
- `/rexrov/cmd_vel` - 控制命令

#### 订阅话题
- `/rexrov/pose_gt` - 机器人位姿
- `/rexrov/odom` - 里程计数据
- `/clock` - 仿真时间

### ROS服务

- `/gazebo/spawn_urdf_model` - 生成机器人模型
- `/gazebo/delete_model` - 删除模型
- `/rexrov/thruster_manager/get_thrusters_info` - 推进器信息

### Python API

```python
from underwater_coverage_planning import TrajectoryPublisher

# 创建轨迹发布器
publisher = TrajectoryPublisher()

# 加载轨迹文件
publisher.load_trajectory('trajectory.yaml')

# 开始发布
publisher.start_publishing()
```

## 🤝 贡献指南

### 开发环境设置

1. Fork项目仓库
2. 创建功能分支
3. 遵循代码规范
4. 添加测试用例
5. 提交Pull Request

### 代码规范

- **Python**: PEP 8标准
- **C++**: Google C++风格
- **XML**: 4空格缩进
- **注释**: 中英文混合，关键部分英文

### 测试要求

- 单元测试覆盖率 > 80%
- 集成测试通过
- 性能测试达标
- 文档更新完整

## 📄 许可证

本项目采用MIT许可证，详见[LICENSE](LICENSE)文件。

## 👥 维护团队

- **主要维护者**: tb
- **邮箱**: tb@todo.todo
- **版本**: 1.0.0

## 🔗 相关链接

- [UUV Simulator](https://uuvsimulator.github.io/)
- [ROS Wiki](http://wiki.ros.org/)
- [Gazebo Documentation](http://gazebosim.org/documentation)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)

## 📝 更新日志

### v1.0.0 (2025-01-21)
- 初始版本发布
- 完整的水下仿真环境
- RexROV机器人集成
- 轨迹规划和控制系统
- 可视化界面完善

---

**🌊 让我们一起探索水下世界的奥秘！**