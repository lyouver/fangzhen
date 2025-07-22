#!/usr/bin/env python3
"""
简单的测试控制器
直接发送推进器命令，不依赖轨迹
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped

def main():
    rospy.init_node('test_controller')
    
    # 创建推进器命令发布器
    thruster_pub = rospy.Publisher('/rexrov/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
    
    # 等待发布器连接
    rospy.loginfo("等待发布器连接...")
    rospy.sleep(2.0)
    
    # 创建推进器命令消息
    wrench_msg = WrenchStamped()
    wrench_msg.header.frame_id = 'rexrov/base_link'
    
    # 设置前进力（Y轴正方向）
    force_magnitude = 500.0  # 500N的推力
    
    rate = rospy.Rate(10)  # 10Hz
    
    rospy.loginfo("开始发送推进器命令...")
    
    while not rospy.is_shutdown():
        # 更新时间戳
        wrench_msg.header.stamp = rospy.Time.now()
        
        # 设置力和力矩
        wrench_msg.wrench.force.x = 0.0
        wrench_msg.wrench.force.y = force_magnitude  # 向前的力
        wrench_msg.wrench.force.z = 0.0
        
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        
        # 发布命令
        thruster_pub.publish(wrench_msg)
        rospy.loginfo(f"发送推力: {force_magnitude}N")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass