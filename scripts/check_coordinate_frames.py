#!/usr/bin/env python3
"""
检查坐标系状态的脚本
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def check_frames():
    rospy.init_node('frame_checker')
    
    # 创建TF缓冲区和监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(2.0)  # 等待TF数据
    
    frames_to_check = ['world', 'world_ned', 'rexrov/base_link']
    
    print("=== 坐标系检查 ===")
    
    # 检查各个坐标系是否存在
    for frame in frames_to_check:
        try:
            # 尝试获取从world到该坐标系的变换
            transform = tf_buffer.lookup_transform('world', frame, rospy.Time(), rospy.Duration(1.0))
            print(f"✓ 坐标系 '{frame}' 存在")
            print(f"  位置: ({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f"✗ 坐标系 '{frame}' 不存在或无法访问: {e}")
    
    # 检查world_ned到world的变换
    try:
        transform = tf_buffer.lookup_transform('world', 'world_ned', rospy.Time(), rospy.Duration(1.0))
        print(f"\n=== world_ned 到 world 的变换 ===")
        print(f"平移: ({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})")
        print(f"旋转: ({transform.transform.rotation.x:.3f}, {transform.transform.rotation.y:.3f}, {transform.transform.rotation.z:.3f}, {transform.transform.rotation.w:.3f})")
    except Exception as e:
        print(f"✗ 无法获取world_ned到world的变换: {e}")
    
    print("\n=== 所有可用的坐标系 ===")
    try:
        frames = tf_buffer.all_frames_as_string()
        print(frames)
    except Exception as e:
        print(f"无法获取坐标系列表: {e}")

if __name__ == '__main__':
    try:
        check_frames()
    except rospy.ROSInterruptException:
        pass