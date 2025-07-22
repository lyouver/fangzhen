#!/usr/bin/env python3
"""
ROS轨迹发布节点 - 增强版
读取YAML轨迹文件并发布到ROS话题
支持命名空间参数，与compact_terrain_rexrov.launch兼容
修复时间戳问题，确保轨迹点时间戳严格递增
增加四元数验证和错误处理
"""

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
        rospy.loginfo(f"使用机器人命名空间: {self.uuv_name}")
        
        # 发布器 - 使用命名空间
        self.traj_pub = rospy.Publisher(f'{self.uuv_name}/dp_controller/input_trajectory', Trajectory, queue_size=1)
        self.traj_pub_vis = rospy.Publisher(f'{self.uuv_name}/dp_controller/trajectory', Trajectory, queue_size=1)
        self.path_pub = rospy.Publisher(f'{self.uuv_name}/path_visualization', Path, queue_size=1)
        self.marker_pub = rospy.Publisher(f'{self.uuv_name}/trajectory_marker', Marker, queue_size=1)
        self.waypoints_pub = rospy.Publisher(f'{self.uuv_name}/waypoints_markers', MarkerArray, queue_size=1)
        
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
        """加载轨迹文件"""
        try:
            with open(filename, 'r') as f:
                trajectory = yaml.safe_load(f)
            rospy.loginfo(f"加载轨迹文件: {filename}")
            rospy.loginfo(f"轨迹包含 {len(trajectory['points'])} 个航点")
            return trajectory
        except Exception as e:
            rospy.logerr(f"加载轨迹文件失败: {e}")
            sys.exit(1)
    
    def validate_quaternion(self, quat):
        """验证并修正四元数"""
        # 检查是否全零
        if all(abs(q) < 1e-6 for q in quat):
            rospy.logwarn("检测到全零四元数，使用单位四元数代替")
            return [0.0, 0.0, 0.0, 1.0]
        
        # 归一化四元数
        norm = np.sqrt(sum(q**2 for q in quat))
        if abs(norm - 1.0) > 1e-6:
            rospy.logwarn(f"四元数未归一化 (norm={norm:.6f})，进行归一化")
            if norm > 1e-6:
                return [q/norm for q in quat]
            else:
                rospy.logwarn("四元数模长接近零，使用单位四元数")
                return [0.0, 0.0, 0.0, 1.0]
        
        return quat
    
    def publish_trajectory_once(self):
        """发布轨迹一次"""
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
        rospy.loginfo(f"发布轨迹消息，包含 {len(traj_msg.points)} 个点")
        
        # 同时发布到trajectory话题，用于可视化
        self.traj_pub_vis.publish(traj_msg)
        
        # 发布可视化
        self.publish_visualizations()
    
    def republish_trajectory(self, event):
        """定期重发轨迹"""
        self.publish_trajectory_once()
    
    def publish_visualizations(self):
        """发布所有可视化"""
        self.publish_path_visualization()
        self.publish_trajectory_marker()
        self.publish_waypoints_markers()
    
    def publish_path_visualization(self):
        """发布路径可视化"""
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
        """发布轨迹线可视化"""
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
        """发布航点标记可视化"""
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
