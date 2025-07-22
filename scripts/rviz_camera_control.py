#!/usr/bin/env python3
"""
RViz相机控制脚本
发布一个大标记在轨迹位置，并提供视角调整指导
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node('rviz_camera_control')
    
    marker_pub = rospy.Publisher('/big_trajectory_marker', Marker, queue_size=1)
    
    rospy.sleep(1.0)
    
    rate = rospy.Rate(2)  # 2Hz
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("RViz视角调整指导")
    rospy.loginfo("=" * 60)
    rospy.loginfo("1. 在RViz中添加Marker显示，Topic设为: /big_trajectory_marker")
    rospy.loginfo("2. 你会看到一个巨大的红色球在轨迹位置")
    rospy.loginfo("3. 右键点击红球，选择'Focus Camera Here'")
    rospy.loginfo("4. 或者手动设置Views面板中的Focal Point:")
    rospy.loginfo("   X: -200, Y: -200, Z: -22, Distance: 1000")
    rospy.loginfo("=" * 60)
    
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "big_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 轨迹位置
        marker.pose.position.x = -200.0
        marker.pose.position.y = -200.0
        marker.pose.position.z = -22.0
        marker.pose.orientation.w = 1.0
        
        # 巨大的球，不可能看不到
        marker.scale.x = 100.0
        marker.scale.y = 100.0
        marker.scale.z = 100.0
        
        # 闪烁的红色
        import time
        if int(time.time() * 2) % 2 == 0:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(1.0)
        
        marker_pub.publish(marker)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass