#!/usr/bin/env python3
"""
轨迹格式转换器
将主机生成的JSON轨迹转换为ROS格式，供虚拟机中的uuv_trajectory_control使用
增强版：包含四元数验证和错误处理
"""

import json
import yaml
import numpy as np
from typing import List, Dict, Any
from dataclasses import dataclass
import os
from datetime import datetime

@dataclass
class ROSWaypoint:
    """ROS格式的航点"""
    position: List[float]  # [x, y, z]
    orientation: List[float]  # [x, y, z, w] (quaternion)
    velocity: List[float]  # [vx, vy, vz]
    timestamp: float

class TrajectoryConverter:
    """轨迹格式转换器"""
    
    def __init__(self):
        self.default_velocity = 2.0  # 默认速度 m/s
        
    def load_json_trajectory(self, json_file: str) -> Dict[str, Any]:
        """加载JSON格式的轨迹文件"""
        print(f"加载轨迹文件: {json_file}")
        
        if not os.path.exists(json_file):
            raise FileNotFoundError(f"轨迹文件不存在: {json_file}")
        
        with open(json_file, 'r') as f:
            trajectory_data = json.load(f)
        
        print(f"轨迹包含 {len(trajectory_data['waypoints'])} 个航点")
        return trajectory_data
    
    def validate_quaternion(self, quat: List[float]) -> List[float]:
        """验证并修正四元数"""
        # 检查是否全零
        if all(abs(q) < 1e-6 for q in quat):
            print("警告：检测到全零四元数，使用单位四元数代替")
            return [0.0, 0.0, 0.0, 1.0]
        
        # 归一化四元数
        norm = np.sqrt(sum(q**2 for q in quat))
        if abs(norm - 1.0) > 1e-6:
            print(f"警告：四元数未归一化 (norm={norm:.6f})，进行归一化")
            if norm > 1e-6:
                return [q/norm for q in quat]
            else:
                print("警告：四元数模长接近零，使用单位四元数")
                return [0.0, 0.0, 0.0, 1.0]
        
        return quat
    
    def yaw_to_quaternion(self, yaw: float) -> List[float]:
        """将偏航角转换为四元数并验证"""
        # 只考虑Z轴旋转（偏航）
        half_yaw = yaw * 0.5
        quat = [0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)]
        return self.validate_quaternion(quat)
    
    def convert_to_ros_trajectory(self, json_trajectory: Dict[str, Any]) -> Dict[str, Any]:
        """转换为ROS轨迹格式"""
        print("转换为ROS轨迹格式...")
        
        waypoints = json_trajectory['waypoints']
        ros_waypoints = []
        
        # 修复时间戳 - 确保严格递增，使用更大的时间间隔
        current_time = 0.0
        time_step = 5.0  # 每个点间隔5秒，确保有足够时间执行
        
        for i, wp in enumerate(waypoints):
            # 计算速度
            if i < len(waypoints) - 1:
                next_wp = waypoints[i + 1]
                # 使用固定时间步长计算速度
                dt = time_step
                vx = (next_wp['x'] - wp['x']) / dt
                vy = (next_wp['y'] - wp['y']) / dt
                vz = (next_wp['z'] - wp['z']) / dt
            else:
                vx = vy = vz = 0.0
            
            ros_wp = ROSWaypoint(
                position=[wp['x'], wp['y'], wp['z']],
                orientation=self.yaw_to_quaternion(wp['yaw']),
                velocity=[vx, vy, vz],
                timestamp=current_time  # 使用递增的时间戳
            )
            ros_waypoints.append(ros_wp)
            current_time += time_step  # 递增时间
        
        # 创建ROS轨迹消息格式
        ros_trajectory = {
            'header': {
                'seq': 0,
                'stamp': {
                    'secs': int(datetime.now().timestamp()),
                    'nsecs': int((datetime.now().timestamp() % 1) * 1e9)
                },
                'frame_id': 'world_ned'
            },
            'points': []  # 移除joint_names，这不是Trajectory消息的标准字段
        }
        
        for ros_wp in ros_waypoints:
            # 从四元数提取欧拉角
            qx, qy, qz, qw = ros_wp.orientation
            # 简化：只考虑偏航角
            yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            point = {
                'positions': ros_wp.position + [0.0, 0.0, yaw],  # [x, y, z, roll, pitch, yaw]
                'velocities': ros_wp.velocity + [0.0, 0.0, 0.0],  # [vx, vy, vz, vroll, vpitch, vyaw]
                'accelerations': [0.0] * 6,  # 加速度设为0
                'effort': [0.0] * 6,  # 力/力矩设为0
                'time_from_start': {
                    'secs': int(ros_wp.timestamp),
                    'nsecs': int((ros_wp.timestamp % 1) * 1e9)
                }
            }
            ros_trajectory['points'].append(point)
        
        return ros_trajectory
    
    def save_ros_trajectory(self, ros_trajectory: Dict[str, Any], output_file: str):
        """保存ROS格式的轨迹"""
        print(f"保存ROS轨迹到: {output_file}")
        
        # 转换numpy对象为Python原生类型
        def convert_numpy(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, dict):
                return {key: convert_numpy(value) for key, value in obj.items()}
            elif isinstance(obj, list):
                return [convert_numpy(item) for item in obj]
            else:
                return obj
        
        # 转换所有numpy对象
        converted_trajectory = convert_numpy(ros_trajectory)
        
        with open(output_file, 'w') as f:
            yaml.dump(converted_trajectory, f, default_flow_style=False)
        
        print("ROS轨迹保存完成")
    
    def convert_trajectory(self, input_file: str, output_file: str):
        """转换轨迹文件"""
        print(f"转换轨迹: {input_file} -> {output_file}")
        
        # 加载JSON轨迹
        json_trajectory = self.load_json_trajectory(input_file)
        
        # 转换为ROS格式
        ros_trajectory = self.convert_to_ros_trajectory(json_trajectory)
        
        # 保存ROS轨迹
        self.save_ros_trajectory(ros_trajectory, output_file)
        
        print("轨迹转换完成")

    def create_launch_file(self, trajectory_file: str, launch_file: str):
        """创建ROS launch文件"""
        print(f"创建launch文件: {launch_file}")
        
        # 获取相对路径（相对于underwater_coverage_planning包）
        rel_trajectory_path = os.path.relpath(
            trajectory_file, 
            start=os.path.join(os.path.dirname(__file__), '../../..')
        )
        
        # 修正路径：确保不包含src/underwater_coverage_planning前缀
        if rel_trajectory_path.startswith('src/underwater_coverage_planning/'):
            rel_trajectory_path = rel_trajectory_path.replace('src/underwater_coverage_planning/', '', 1)
        
        # 获取发布器脚本的文件名（不包含路径）
        base_name = os.path.basename(trajectory_file)
        if base_name.endswith('_ros_fixed.yaml'):
            base_name = base_name.replace('_ros_fixed.yaml', '')
            publisher_script_name = f"{base_name}_publisher_fixed.py"
        elif base_name.endswith('_ros.yaml'):
            base_name = base_name.replace('_ros.yaml', '')
            publisher_script_name = f"{base_name}_publisher.py"
        else:
            # 处理其他情况
            base_name = base_name.replace('.yaml', '').replace('.json', '')
            publisher_script_name = f"{base_name}_publisher.py"
        
        launch_content = f'''<?xml version="1.0"?>
<launch>
    <!-- 
        UUV Coverage Path Execution - 兼容版本
        此launch文件设计为与compact_terrain_rexrov.launch一起使用
        不包含重复的Gazebo、RViz或机器人模型启动
    -->
    
    <!-- 参数设置 -->
    <arg name="uuv_name" default="rexrov"/>
    <arg name="model_name" default="rexrov"/>
    <arg name="use_ned_frame" default="true"/>
    <arg name="model_params_file" default="$(find uuv_trajectory_control)/config/models/rexrov/params.yaml"/>
    
    <!-- 推进器管理器由compact_terrain_rexrov_with_trajectory.launch提供 -->
    
    <!-- 加载轨迹参数 -->
    <rosparam file="$(find underwater_coverage_planning)/{rel_trajectory_path}" command="load" ns="trajectory_planner"/>
    
    <!-- 使用本地轨迹发布器 -->
    <node name="trajectory_publisher" 
          pkg="underwater_coverage_planning"
          type="{publisher_script_name}"
          output="screen"
          launch-prefix="bash -c 'sleep 3; $0 $@' ">
        <param name="trajectory_file" value="$(find underwater_coverage_planning)/{rel_trajectory_path}"/>
        <param name="publish_rate" value="1.0"/>
    </node>
    
    <!-- 使用UUV轨迹控制器组合 -->
    <group ns="$(arg uuv_name)">
        <!-- 启动轨迹规划器 -->
        <node name="dp_controller_local_planner" 
              pkg="uuv_trajectory_control"
              type="dp_controller_local_planner.py"
              output="screen">
            <rosparam subst_value="true">
                inertial_frame_id: world_ned
                max_forward_speed: 2.0
                idle_radius: 10.0
                timeout_idle_mode: 5.0
                look_ahead_delay: 0.0
                is_underactuated: false
                stamped_pose_only: false
                thrusters_only: true
                # 插值器参数
                lipb:
                    max_forward_speed: 2.0
                    interpolation_method: lipb
                cubic:
                    max_forward_speed: 2.0
                    interpolation_method: cubic
                linear:
                    max_forward_speed: 2.0
                    interpolation_method: linear
            </rosparam>
        </node>
        
        <!-- 使用无奇点轨迹控制器 -->
        <node name="rov_sf_controller" 
              pkg="uuv_trajectory_control"
              type="rov_sf_controller.py"
              output="screen">
            <!-- Remap necessary topics -->
            <remap from="odom" to="pose_gt"/>
            <remap from="trajectory" to="dp_controller/trajectory"/>
            <remap from="input_trajectory" to="dp_controller/input_trajectory"/>
            <remap from="waypoints" to="dp_controller/waypoints"/>
            <remap from="error" to="dp_controller/error"/>
            <remap from="reference" to="dp_controller/reference"/>
            <remap from="thruster_output" to="thruster_manager/input_stamped"/>
            
            <!-- 加载机器人模型参数 -->
            <rosparam file="$(arg model_params_file)" command="load"/>
            
            <!-- 控制器参数 -->
            <rosparam subst_value="true">
              saturation: 5000
              Kd: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
              lambda: [1.0]
              c: [1.0]
              inertial_frame_id: world_ned
            </rosparam>
        </node>
    </group>
    
    <!-- 使用说明 -->
    <!-- 
        使用方法:
        1. 先启动环境和机器人:
           roslaunch underwater_coverage_planning compact_terrain_rexrov.launch
           
        2. 然后在新终端启动此文件:
           roslaunch underwater_coverage_planning {os.path.basename(launch_file)}
           
        注意:
        - 此launch文件不会启动Gazebo或RViz，依赖compact_terrain_rexrov.launch提供环境
        - 如果您需要单独运行，请取消注释下面的环境启动部分
    -->
    
    <!-- 
    取消注释以独立运行（不推荐）
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="world_name" value="$(find underwater_coverage_planning)/worlds/compact_underwater_terrain.world"/>
      <arg name="paused" value="false"/>
      <arg name="use_sim_time" value="true"/>
      <arg name="gui" value="true"/>
    </include>
    
    <include file="$(find uuv_assistants)/launch/publish_world_ned_frame.launch"/>
    
    <include file="$(find uuv_descriptions)/launch/upload_rexrov_default.launch">
      <arg name="x" value="-200"/>
      <arg name="y" value="-200"/>
      <arg name="z" value="-22"/>
      <arg name="namespace" value="$(arg uuv_name)"/>
      <arg name="use_ned_frame" value="true"/>
    </include>
    -->
</launch>'''
        
        with open(launch_file, 'w') as f:
            f.write(launch_content)
        
        print("Launch文件创建完成")
    
    def create_trajectory_publisher_script(self, trajectory_file: str, publisher_file: str):
        """创建轨迹发布器脚本"""
        print(f"创建发布器脚本: {publisher_file}")
        
        # 获取相对路径（相对于underwater_coverage_planning包）
        rel_trajectory_path = os.path.relpath(
            trajectory_file, 
            start=os.path.join(os.path.dirname(__file__), '../../..')
        )
        
        # 修正路径：确保不包含src/underwater_coverage_planning前缀
        if rel_trajectory_path.startswith('src/underwater_coverage_planning/'):
            rel_trajectory_path = rel_trajectory_path.replace('src/underwater_coverage_planning/', '', 1)
            
        # 创建发布器脚本
        with open(publisher_file, 'w') as f:
            f.write(f"""#!/usr/bin/env python3
\"\"\"
ROS轨迹发布节点 - 增强版
读取YAML轨迹文件并发布到ROS话题
支持命名空间参数，与compact_terrain_rexrov.launch兼容
修复时间戳问题，确保轨迹点时间戳严格递增
增加四元数验证和错误处理
\"\"\"

import rospy
import yaml
import numpy as np
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, Vector3, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as trans
import sys

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        
        # 等待机器人初始化
        rospy.loginfo("等待机器人初始化...")
        rospy.sleep(5.0)  # 延迟5秒确保机器人完全初始化
        
        # 参数
        trajectory_file = rospy.get_param('~trajectory_file', 'trajectory.yaml')
        publish_rate = rospy.get_param('~publish_rate', 1.0)  # 降低发布频率，避免重复发送
        
        # 获取机器人命名空间参数
        self.uuv_name = rospy.get_param('/uuv_name', 'rexrov')
        rospy.loginfo(f"使用机器人命名空间: {{self.uuv_name}}")
        
        # 发布器 - 使用命名空间
        self.traj_pub = rospy.Publisher(f'{{self.uuv_name}}/dp_controller/input_trajectory', Trajectory, queue_size=1)
        self.traj_pub_vis = rospy.Publisher(f'{{self.uuv_name}}/dp_controller/trajectory', Trajectory, queue_size=1)
        self.path_pub = rospy.Publisher(f'{{self.uuv_name}}/path_visualization', Path, queue_size=1)
        self.marker_pub = rospy.Publisher(f'{{self.uuv_name}}/trajectory_marker', Marker, queue_size=1)
        self.waypoints_pub = rospy.Publisher(f'{{self.uuv_name}}/waypoints_markers', MarkerArray, queue_size=1)
        
        # 加载轨迹
        self.trajectory = self.load_trajectory(trajectory_file)
        
        # 等待订阅者连接
        rospy.sleep(2.0)
        
        # 发布一次轨迹，然后定期重发
        self.publish_trajectory_once()
        
        # 定时器 - 定期重发轨迹
        self.timer = rospy.Timer(rospy.Duration(1.0/publish_rate), self.republish_trajectory)
        
        rospy.loginfo("轨迹发布器启动完成")
    
    def load_trajectory(self, filename):
        \"\"\"加载轨迹文件\"\"\"
        try:
            with open(filename, 'r') as f:
                trajectory = yaml.safe_load(f)
            rospy.loginfo(f"加载轨迹文件: {{filename}}")
            rospy.loginfo(f"轨迹包含 {{len(trajectory['points'])}} 个航点")
            return trajectory
        except Exception as e:
            rospy.logerr(f"加载轨迹文件失败: {{e}}")
            sys.exit(1)
    
    def validate_quaternion(self, quat):
        \"\"\"验证并修正四元数\"\"\"
        # 检查是否全零
        if all(abs(q) < 1e-6 for q in quat):
            rospy.logwarn("检测到全零四元数，使用单位四元数代替")
            return [0.0, 0.0, 0.0, 1.0]
        
        # 归一化四元数
        norm = np.sqrt(sum(q**2 for q in quat))
        if abs(norm - 1.0) > 1e-6:
            rospy.logwarn(f"四元数未归一化 (norm={{norm:.6f}})，进行归一化")
            if norm > 1e-6:
                return [q/norm for q in quat]
            else:
                rospy.logwarn("四元数模长接近零，使用单位四元数")
                return [0.0, 0.0, 0.0, 1.0]
        
        return quat
    
    def publish_trajectory_once(self):
        \"\"\"发布轨迹一次\"\"\"
        if not self.trajectory:
            return
        
        # 发布uuv_control_msgs/Trajectory消息
        traj_msg = Trajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = 'world_ned'
        
        # 获取当前时间作为基准时间
        base_time = rospy.Time.now()
        
        # 转换每个轨迹点，确保时间戳严格递增
        for i, point_data in enumerate(self.trajectory['points']):
            point = TrajectoryPoint()
            
            # 关键修复：设置严格递增的时间戳
            time_from_start_secs = point_data['time_from_start']['secs']
            time_from_start_nsecs = point_data['time_from_start']['nsecs']
            point_time = base_time + rospy.Duration(time_from_start_secs, time_from_start_nsecs)
            
            point.header.stamp = point_time
            point.header.frame_id = 'world_ned'
            
            # 位置
            point.pose.position.x = point_data['positions'][0]
            point.pose.position.y = point_data['positions'][1]
            point.pose.position.z = point_data['positions'][2]
            
            # 姿态 - 从欧拉角转换为四元数，并验证
            roll = point_data['positions'][3]
            pitch = point_data['positions'][4]
            yaw = point_data['positions'][5]
            quat = trans.quaternion_from_euler(roll, pitch, yaw)
            quat = self.validate_quaternion(quat)
            point.pose.orientation.x = quat[0]
            point.pose.orientation.y = quat[1]
            point.pose.orientation.z = quat[2]
            point.pose.orientation.w = quat[3]
            
            # 速度
            if 'velocities' in point_data and len(point_data['velocities']) >= 6:
                point.velocity.linear.x = point_data['velocities'][0]
                point.velocity.linear.y = point_data['velocities'][1]
                point.velocity.linear.z = point_data['velocities'][2]
                point.velocity.angular.x = point_data['velocities'][3]
                point.velocity.angular.y = point_data['velocities'][4]
                point.velocity.angular.z = point_data['velocities'][5]
            
            # 加速度
            if 'accelerations' in point_data and len(point_data['accelerations']) >= 6:
                point.acceleration.linear.x = point_data['accelerations'][0]
                point.acceleration.linear.y = point_data['accelerations'][1]
                point.acceleration.linear.z = point_data['accelerations'][2]
                point.acceleration.angular.x = point_data['accelerations'][3]
                point.acceleration.angular.y = point_data['accelerations'][4]
                point.acceleration.angular.z = point_data['accelerations'][5]
            else:
                # 如果没有加速度数据，设为0
                point.acceleration.linear.x = 0.0
                point.acceleration.linear.y = 0.0
                point.acceleration.linear.z = 0.0
                point.acceleration.angular.x = 0.0
                point.acceleration.angular.y = 0.0
                point.acceleration.angular.z = 0.0
            
            traj_msg.points.append(point)
        
        # 发布轨迹消息
        self.traj_pub.publish(traj_msg)
        rospy.loginfo(f"发布轨迹消息，包含 {{len(traj_msg.points)}} 个点")
        
        # 同时发布到trajectory话题，用于可视化
        self.traj_pub_vis.publish(traj_msg)
        
        # 发布可视化
        self.publish_visualizations()
    
    def republish_trajectory(self, event):
        \"\"\"定期重发轨迹\"\"\"
        self.publish_trajectory_once()
    
    def publish_visualizations(self):
        \"\"\"发布所有可视化\"\"\"
        self.publish_path_visualization()
        self.publish_trajectory_marker()
        self.publish_waypoints_markers()
    
    def publish_path_visualization(self):
        \"\"\"发布路径可视化\"\"\"
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        
        # 始终使用world_ned坐标系
        path_msg.header.frame_id = 'world_ned'
        
        for point_data in self.trajectory['points']:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point_data['positions'][0]
            pose.pose.position.y = point_data['positions'][1]
            pose.pose.position.z = point_data['positions'][2]
            
            # 从欧拉角转换为四元数
            roll = point_data['positions'][3]
            pitch = point_data['positions'][4]
            yaw = point_data['positions'][5]
            quat = trans.quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_trajectory_marker(self):
        \"\"\"发布轨迹线可视化\"\"\"
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 线宽
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)  # 永久显示
        
        # 添加所有轨迹点
        for point_data in self.trajectory['points']:
            p = Vector3()
            p.x = point_data['positions'][0]
            p.y = point_data['positions'][1]
            p.z = point_data['positions'][2]
            marker.points.append(p)
        
        self.marker_pub.publish(marker)
    
    def publish_waypoints_markers(self):
        \"\"\"发布航点标记可视化\"\"\"
        marker_array = MarkerArray()
        
        # 创建航点标记
        for i, point_data in enumerate(self.trajectory['points']):
            # 每10个点创建一个航点标记
            if i % 10 != 0 and i != len(self.trajectory['points']) - 1:
                continue
                
            marker = Marker()
            marker.header.frame_id = "world_ned"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 位置
            marker.pose.position.x = point_data['positions'][0]
            marker.pose.position.y = point_data['positions'][1]
            marker.pose.position.z = point_data['positions'][2]
            
            # 姿态
            roll = point_data['positions'][3]
            pitch = point_data['positions'][4]
            yaw = point_data['positions'][5]
            quat = trans.quaternion_from_euler(roll, pitch, yaw)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            
            # 大小
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # 颜色 - 起点绿色，终点红色，中间点蓝色
            if i == 0:  # 起点
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == len(self.trajectory['points']) - 1:  # 终点
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:  # 中间点
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)  # 永久显示
            
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        publisher = TrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
""")
        
        # 设置执行权限
        os.chmod(publisher_file, os.stat(publisher_file).st_mode | 0o111)

def main():
    """主函数"""
    print("=== 轨迹格式转换器 ===")
    
    # 确保输出目录存在
    output_dir = os.path.join(os.path.dirname(__file__), 'guiji')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录: {output_dir}")
    
    # 查找最新的轨迹文件
    guiji_dir = os.path.join(os.path.dirname(__file__), 'guiji')
    if os.path.exists(guiji_dir):
        trajectory_files = [f for f in os.listdir(guiji_dir) if f.startswith('trajectory_') and f.endswith('.json')]
        # 添加完整路径
        trajectory_files = [os.path.join(guiji_dir, f) for f in trajectory_files]
    else:
        # 如果guiji目录不存在，则在当前目录和工作区根目录查找
        current_dir_files = [f for f in os.listdir('.') if f.startswith('trajectory_') and f.endswith('.json')]
        workspace_root = os.path.join(os.path.dirname(__file__), '../../..')
        workspace_files = []
        if os.path.exists(workspace_root):
            workspace_files = [f for f in os.listdir(workspace_root) if f.startswith('trajectory_') and f.endswith('.json')]
            workspace_files = [os.path.join(workspace_root, f) for f in workspace_files]
        trajectory_files = [os.path.join('.', f) for f in current_dir_files] + workspace_files
    
    if not trajectory_files:
        print("未找到轨迹文件，请将轨迹文件放在guiji目录下，或当前目录，或工作区根目录")
        return
    
    # 按修改时间排序，获取最新的文件
    trajectory_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
    trajectory_file = trajectory_files[0]
    print(f"使用最新的轨迹文件: {trajectory_file}")
    
    # 生成输出文件名
    base_name = os.path.basename(trajectory_file).replace('.json', '')
    ros_trajectory_file = os.path.join(output_dir, f"{base_name}_ros.yaml")
    launch_file = os.path.join(output_dir, f"{base_name}_launch.launch")
    publisher_script = os.path.join(output_dir, f"{base_name}_publisher.py")
    
    # 创建转换器实例
    converter = TrajectoryConverter()
    
    # 转换轨迹
    converter.convert_trajectory(trajectory_file, ros_trajectory_file)
    
    # 创建launch文件
    converter.create_launch_file(ros_trajectory_file, launch_file)
    
    # 创建发布脚本
    converter.create_trajectory_publisher_script(ros_trajectory_file, publisher_script)
    
    print(f"\n=== 转换完成 ===")
    print(f"轨迹文件: {ros_trajectory_file}")
    print(f"Launch文件: {launch_file}")
    print(f"发布脚本: {publisher_script}")
    print("\n使用方法:")
    print(f"1. 首先启动环境和机器人 (带轨迹可视化):")
    print(f"   roslaunch underwater_coverage_planning compact_terrain_rexrov_with_trajectory.launch")
    print(f"2. 然后在新终端中启动轨迹控制器:")
    print(f"   roslaunch {os.path.relpath(launch_file, start=os.path.join(os.path.dirname(__file__), '../../..'))}")
    print("\n可视化说明:")
    print("- 绿色球体：起点")
    print("- 红色球体：终点")
    print("- 蓝色球体：中间航点")
    print("- 绿色线条：路径可视化")
    print("- 蓝色线条：轨迹线可视化")
    print("\n如果您看到TF警告，可以使用以下命令隐藏它们:")
    print("rosservice call /rosout/set_logger_level \"logger: 'ros.tf2' level: 'error'\"")
    
if __name__ == "__main__":
    main()