#!/usr/bin/env python3
"""
简单的轨迹控制器
接收轨迹消息并直接发送推进器命令
"""

import rospy
import numpy as np
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import WrenchStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as trans

class SimpleTrajectoryController:
    def __init__(self):
        rospy.init_node('simple_trajectory_controller')
        
        # 参数
        self.Kp = 200.0  # 位置比例增益（降低以减少振荡）
        self.Kd = 50.0   # 速度比例增益
        self.max_force = 800.0  # 最大推力限制
        
        # 状态变量
        self.current_pose = None
        self.target_pose = None
        self.trajectory = None
        self.current_point_index = 0
        
        # 订阅器
        self.pose_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.pose_callback)
        self.trajectory_sub = rospy.Subscriber('/rexrov/dp_controller/input_trajectory', Trajectory, self.trajectory_callback)
        
        # 发布器
        self.thruster_pub = rospy.Publisher('/rexrov/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.reference_pub = rospy.Publisher('/rexrov/dp_controller/reference', TrajectoryPoint, queue_size=1)
        self.error_pub = rospy.Publisher('/rexrov/dp_controller/error', TrajectoryPoint, queue_size=1)
        
        # 等待初始化
        rospy.loginfo("等待初始化...")
        rospy.sleep(2.0)
        
        # 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("简单轨迹控制器已启动")
    
    def pose_callback(self, msg):
        """接收机器人位姿"""
        self.current_pose = msg
    
    def trajectory_callback(self, msg):
        """接收轨迹消息"""
        self.trajectory = msg
        self.current_point_index = 0
        rospy.loginfo(f"接收到新轨迹，包含 {len(msg.points)} 个点")
    
    def control_loop(self, event):
        """控制循环"""
        if self.current_pose is None or self.trajectory is None:
            return
        
        # 获取当前目标点
        if self.current_point_index >= len(self.trajectory.points):
            return
        
        target_point = self.trajectory.points[self.current_point_index]
        
        # 计算位置误差（在world_ned坐标系中）
        # 注意：这里假设current_pose是在world坐标系中，需要转换到world_ned
        # 在NED中：North=X, East=Y, Down=Z
        # 在ENU中：East=X, North=Y, Up=Z
        # 所以转换是：X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
        
        # 获取当前位置（ENU）
        current_x_enu = self.current_pose.pose.pose.position.x
        current_y_enu = self.current_pose.pose.pose.position.y
        current_z_enu = self.current_pose.pose.pose.position.z
        
        # 转换到NED
        current_x_ned = current_y_enu
        current_y_ned = current_x_enu
        current_z_ned = -current_z_enu
        
        # 获取目标位置（NED）
        target_x_ned = target_point.pose.position.x
        target_y_ned = target_point.pose.position.y
        target_z_ned = target_point.pose.position.z
        
        # 计算误差（NED）
        error_x = target_x_ned - current_x_ned
        error_y = target_y_ned - current_y_ned
        error_z = target_z_ned - current_z_ned
        
        # 计算距离
        distance = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # 如果到达目标点，移动到下一个点
        if distance < 1.0:  # 1米的接受半径
            self.current_point_index += 1
            rospy.loginfo(f"到达目标点 {self.current_point_index-1}，移动到下一个点")
            return
        
        # 计算控制力（简单的PD控制器）
        force_x = self.Kp * error_x
        force_y = self.Kp * error_y
        force_z = self.Kp * error_z
        
        # 添加速度控制（如果有速度信息）
        if hasattr(target_point, 'velocity') and hasattr(target_point.velocity, 'linear'):
            force_x += self.Kd * target_point.velocity.linear.x
            force_y += self.Kd * target_point.velocity.linear.y
            force_z += self.Kd * target_point.velocity.linear.z
        
        # 限制最大力
        force_x = np.clip(force_x, -self.max_force, self.max_force)
        force_y = np.clip(force_y, -self.max_force, self.max_force)
        force_z = np.clip(force_z, -self.max_force, self.max_force)
        
        # 转换力从NED到ENU
        force_x_enu = force_y
        force_y_enu = force_x
        force_z_enu = -force_z
        
        # 创建推进器命令
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = 'rexrov/base_link'
        
        wrench_msg.wrench.force.x = force_x_enu
        wrench_msg.wrench.force.y = force_y_enu
        wrench_msg.wrench.force.z = force_z_enu
        
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        
        # 发布推进器命令
        self.thruster_pub.publish(wrench_msg)
        
        # 发布参考和误差（用于可视化和调试）
        self.publish_reference(target_point)
        self.publish_error(error_x, error_y, error_z)
        
        rospy.loginfo(f"目标: ({target_x_ned:.2f}, {target_y_ned:.2f}, {target_z_ned:.2f}), " +
                     f"当前: ({current_x_ned:.2f}, {current_y_ned:.2f}, {current_z_ned:.2f}), " +
                     f"误差: ({error_x:.2f}, {error_y:.2f}, {error_z:.2f}), " +
                     f"力: ({force_x_enu:.2f}, {force_y_enu:.2f}, {force_z_enu:.2f})")
    
    def publish_reference(self, target_point):
        """发布参考点"""
        self.reference_pub.publish(target_point)
    
    def publish_error(self, error_x, error_y, error_z):
        """发布误差"""
        error_msg = TrajectoryPoint()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = 'world_ned'
        
        error_msg.pose.position.x = error_x
        error_msg.pose.position.y = error_y
        error_msg.pose.position.z = error_z
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        controller = SimpleTrajectoryController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass