#!/usr/bin/env python3
"""
调试ROV_SFController的bad callback错误
分析并修复控制器回调函数中的异常
"""

import rospy
import traceback
import sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped

class DebugROVSFController:
    """调试版本的ROV_SFController"""
    
    def __init__(self):
        rospy.init_node('debug_rov_sf_controller', log_level=rospy.DEBUG)
        
        # 订阅odometry话题进行测试
        self.odom_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.debug_odom_callback)
        
        # 发布推力输出
        self.thrust_pub = rospy.Publisher('/rexrov/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        
        rospy.loginfo("调试控制器启动")
    
    def debug_odom_callback(self, msg):
        """调试版本的odometry回调函数"""
        try:
            rospy.logdebug(f"收到odometry消息: {msg.header.stamp}")
            
            # 检查消息的完整性
            if not hasattr(msg, 'pose') or not hasattr(msg.pose, 'pose'):
                rospy.logerr("Odometry消息格式错误: 缺少pose信息")
                return
            
            if not hasattr(msg, 'twist') or not hasattr(msg.twist, 'twist'):
                rospy.logerr("Odometry消息格式错误: 缺少twist信息")
                return
            
            # 提取位置信息
            pos = msg.pose.pose.position
            rospy.logdebug(f"位置: x={pos.x}, y={pos.y}, z={pos.z}")
            
            # 提取姿态信息
            orient = msg.pose.pose.orientation
            rospy.logdebug(f"姿态: x={orient.x}, y={orient.y}, z={orient.z}, w={orient.w}")
            
            # 提取速度信息
            linear_vel = msg.twist.twist.linear
            angular_vel = msg.twist.twist.angular
            rospy.logdebug(f"线速度: x={linear_vel.x}, y={linear_vel.y}, z={linear_vel.z}")
            rospy.logdebug(f"角速度: x={angular_vel.x}, y={angular_vel.y}, z={angular_vel.z}")
            
            # 检查数值是否有效
            values_to_check = [
                pos.x, pos.y, pos.z,
                orient.x, orient.y, orient.z, orient.w,
                linear_vel.x, linear_vel.y, linear_vel.z,
                angular_vel.x, angular_vel.y, angular_vel.z
            ]
            
            for i, val in enumerate(values_to_check):
                if np.isnan(val) or np.isinf(val):
                    rospy.logerr(f"检测到无效数值: index={i}, value={val}")
                    return
            
            # 模拟控制器计算
            self.simulate_controller_update(msg)
            
        except Exception as e:
            rospy.logerr(f"Odometry回调函数异常: {e}")
            rospy.logerr(f"异常详情: {traceback.format_exc()}")
    
    def simulate_controller_update(self, odom_msg):
        """模拟控制器更新过程"""
        try:
            # 模拟基本的控制计算
            # 这里只是一个简单的P控制器示例
            
            # 目标位置 (示例)
            target_x, target_y, target_z = -200.0, -200.0, -22.0
            
            # 当前位置
            current_x = odom_msg.pose.pose.position.x
            current_y = odom_msg.pose.pose.position.y
            current_z = odom_msg.pose.pose.position.z
            
            # 位置误差
            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = target_z - current_z
            
            # 简单P控制
            kp = 100.0  # 比例增益
            force_x = kp * error_x
            force_y = kp * error_y
            force_z = kp * error_z
            
            # 限制力的大小
            max_force = 1000.0
            force_x = np.clip(force_x, -max_force, max_force)
            force_y = np.clip(force_y, -max_force, max_force)
            force_z = np.clip(force_z, -max_force, max_force)
            
            # 发布控制输出
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = 'rexrov/base_link'
            
            wrench_msg.wrench.force.x = force_x
            wrench_msg.wrench.force.y = force_y
            wrench_msg.wrench.force.z = force_z
            wrench_msg.wrench.torque.x = 0.0
            wrench_msg.wrench.torque.y = 0.0
            wrench_msg.wrench.torque.z = 0.0
            
            self.thrust_pub.publish(wrench_msg)
            
            rospy.logdebug(f"发布控制输出: fx={force_x:.2f}, fy={force_y:.2f}, fz={force_z:.2f}")
            
        except Exception as e:
            rospy.logerr(f"控制器计算异常: {e}")
            rospy.logerr(f"异常详情: {traceback.format_exc()}")

def check_ros_parameters():
    """检查必需的ROS参数"""
    rospy.loginfo("=== ROS参数检查 ===")
    
    # 检查常见的控制器参数
    param_checks = [
        '/rexrov/rov_sf_controller/saturation',
        '/rexrov/rov_sf_controller/Kd',
        '/rexrov/rov_sf_controller/lambda',
        '/rexrov/rov_sf_controller/c',
        '/rexrov/rov_sf_controller/inertial_frame_id',
        # 车辆模型参数
        '/rexrov/mass',
        '/rexrov/volume',
        '/rexrov/center_of_mass',
        '/rexrov/center_of_buoyancy'
    ]
    
    for param in param_checks:
        if rospy.has_param(param):
            value = rospy.get_param(param)
            rospy.loginfo(f"✓ {param} = {value}")
        else:
            rospy.logwarn(f"✗ 缺少参数: {param}")

def check_ros_topics():
    """检查ROS话题"""
    rospy.loginfo("=== ROS话题检查 ===")
    
    import rostopic
    
    # 检查关键话题
    topic_checks = [
        '/rexrov/pose_gt',
        '/rexrov/thruster_manager/input_stamped',
        '/rexrov/dp_controller/input_trajectory',
        '/rexrov/dp_controller/trajectory',
        '/tf'
    ]
    
    for topic in topic_checks:
        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
            if msg_class:
                rospy.loginfo(f"✓ {topic} -> {msg_class.__name__}")
            else:
                rospy.logwarn(f"✗ 话题不存在: {topic}")
        except Exception as e:
            rospy.logwarn(f"✗ 检查话题失败 {topic}: {e}")

def main():
    """主函数"""
    print("=== ROV_SFController 调试工具 ===")
    print("此工具将诊断和测试ROV控制器的回调函数问题")
    
    try:
        # 检查ROS环境
        check_ros_parameters()
        check_ros_topics()
        
        # 启动调试控制器
        debug_controller = DebugROVSFController()
        
        rospy.loginfo("调试控制器运行中... 按Ctrl+C停止")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("调试控制器停止")
    except Exception as e:
        rospy.logerr(f"调试工具异常: {e}")
        rospy.logerr(f"异常详情: {traceback.format_exc()}")

if __name__ == '__main__':
    main() 