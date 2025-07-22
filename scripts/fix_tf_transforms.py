#!/usr/bin/env python3
"""
TF转换修复脚本
用于添加缺失的TF转换，确保轨迹跟踪系统正常工作
"""

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as trans
import numpy as np
from threading import Thread
import time

class TFTransformFixer:
    def __init__(self):
        rospy.init_node('tf_transform_fixer', anonymous=True)
        
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布频率
        self.rate = rospy.Rate(50)  # 50Hz
        
        print("TF转换修复器已启动")
        
    def create_transform(self, parent_frame, child_frame, translation, rotation):
        """创建TF转换消息"""
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # 设置平移
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        
        # 设置旋转（四元数）
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        
        return t
    
    def publish_static_transforms(self):
        """发布静态转换"""
        transforms = []
        
        # 1. world -> world_ned 转换 (ENU到NED的转换)
        # NED: North(X), East(Y), Down(Z)
        # ENU: East(X), North(Y), Up(Z)
        # 旋转: 绕Z轴旋转90度，然后绕X轴旋转180度
        enu_to_ned_quat = trans.quaternion_from_euler(np.pi, 0, np.pi/2)
        world_to_world_ned = self.create_transform(
            "world", "world_ned",
            [0.0, 0.0, 0.0],  # 无平移
            enu_to_ned_quat
        )
        transforms.append(world_to_world_ned)
        
        # 2. rexrov/base_link -> rexrov/base_link_ned 转换
        # 这是机器人本地坐标系的ENU到NED转换
        base_link_to_ned_quat = trans.quaternion_from_euler(np.pi, 0, np.pi/2)
        base_link_to_ned = self.create_transform(
            "rexrov/base_link", "rexrov/base_link_ned",
            [0.0, 0.0, 0.0],  # 无平移
            base_link_to_ned_quat
        )
        transforms.append(base_link_to_ned)
        
        return transforms
    
    def check_existing_transforms(self):
        """检查现有的转换"""
        transforms_to_check = [
            ('world', 'rexrov/base_link'),
            ('world', 'rexrov/base_footprint'),
            ('rexrov/base_footprint', 'rexrov/base_stabilized'),
            ('rexrov/base_stabilized', 'rexrov/base_link')
        ]
        
        existing_transforms = []
        for source, target in transforms_to_check:
            try:
                # 等待转换可用
                self.tf_buffer.can_transform(target, source, rospy.Time(), rospy.Duration(1.0))
                existing_transforms.append((source, target))
                print(f"✓ 发现现有转换: {source} -> {target}")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print(f"✗ 缺失转换: {source} -> {target}")
        
        return existing_transforms
    
    def publish_dynamic_world_to_base_link(self):
        """发布动态的world到base_link转换"""
        # 如果没有现有的world到base_link转换，我们需要创建一个
        # 这通常由定位系统提供，但在仿真中可能需要手动添加
        
        try:
            # 尝试获取机器人的当前位置（从pose_gt话题）
            # 这里我们先提供一个静态的转换作为示例
            world_to_base_link = self.create_transform(
                "world", "rexrov/base_link",
                [-200.0, -200.0, -22.0],  # 机器人初始位置
                [0.0, 0.0, 0.0, 1.0]      # 无旋转
            )
            return world_to_base_link
        except Exception as e:
            print(f"创建world到base_link转换时出错: {e}")
            return None
    
    def run(self):
        """运行TF修复器"""
        print("开始发布TF转换...")
        
        # 检查现有转换
        print("检查现有转换...")
        existing_transforms = self.check_existing_transforms()
        
        # 等待一下让系统稳定
        time.sleep(2)
        
        while not rospy.is_shutdown():
            try:
                # 获取静态转换
                static_transforms = self.publish_static_transforms()
                
                # 检查是否需要world到base_link的转换
                world_to_base_link = None
                try:
                    self.tf_buffer.can_transform('rexrov/base_link', 'world', rospy.Time(), rospy.Duration(0.1))
                except:
                    # 如果没有world到base_link的转换，创建一个
                    world_to_base_link = self.publish_dynamic_world_to_base_link()
                
                # 发布所有转换
                all_transforms = static_transforms
                if world_to_base_link:
                    all_transforms.append(world_to_base_link)
                
                for transform in all_transforms:
                    transform.header.stamp = rospy.Time.now()
                    self.tf_broadcaster.sendTransform(transform)
                
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                print(f"发布TF转换时出错: {e}")
                self.rate.sleep()

def main():
    print("=== TF转换修复器 ===")
    print("此脚本将添加缺失的TF转换以支持轨迹跟踪")
    print()
    
    try:
        fixer = TFTransformFixer()
        fixer.run()
    except rospy.ROSInterruptException:
        print("TF转换修复器已停止")
    except Exception as e:
        print(f"启动TF转换修复器时出错: {e}")

if __name__ == "__main__":
    main()