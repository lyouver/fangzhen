#!/usr/bin/env python3
"""
修复轨迹时间戳脚本
解决"Trajectory should be given a growing value of time"错误
"""

import os
import sys
import yaml
import json
from trajectory_to_ros import TrajectoryConverter

def fix_existing_trajectory():
    """修复现有的轨迹文件"""
    print("=== 修复轨迹时间戳 ===")
    
    # 查找现有的轨迹文件
    guiji_dir = os.path.join(os.path.dirname(__file__), 'guiji')
    if not os.path.exists(guiji_dir):
        print("未找到guiji目录")
        return
    
    # 查找JSON轨迹文件
    json_files = [f for f in os.listdir(guiji_dir) if f.startswith('trajectory_') and f.endswith('.json')]
    if not json_files:
        print("未找到JSON轨迹文件")
        return
    
    # 使用最新的JSON文件
    json_files.sort(key=lambda x: os.path.getmtime(os.path.join(guiji_dir, x)), reverse=True)
    json_file = os.path.join(guiji_dir, json_files[0])
    
    print(f"处理轨迹文件: {json_file}")
    
    # 生成输出文件名
    base_name = os.path.basename(json_file).replace('.json', '')
    ros_trajectory_file = os.path.join(guiji_dir, f"{base_name}_ros_fixed.yaml")
    launch_file = os.path.join(guiji_dir, f"{base_name}_launch_fixed.launch")
    publisher_script = os.path.join(guiji_dir, f"{base_name}_publisher_fixed.py")
    
    # 创建转换器实例
    converter = TrajectoryConverter()
    
    # 重新转换轨迹
    converter.convert_trajectory(json_file, ros_trajectory_file)
    
    # 创建修复的launch文件
    converter.create_launch_file(ros_trajectory_file, launch_file)
    
    # 创建修复的发布脚本
    converter.create_trajectory_publisher_script(ros_trajectory_file, publisher_script)
    
    print(f"\n=== 修复完成 ===")
    print(f"修复的轨迹文件: {ros_trajectory_file}")
    print(f"修复的Launch文件: {launch_file}")
    print(f"修复的发布脚本: {publisher_script}")
    print("\n使用方法:")
    print(f"1. 首先启动环境和机器人:")
    print(f"   roslaunch underwater_coverage_planning compact_terrain_rexrov_with_trajectory.launch")
    print(f"2. 然后在新终端中启动修复的轨迹控制器:")
    print(f"   roslaunch {os.path.relpath(launch_file, start=os.path.join(os.path.dirname(__file__), '../../..'))}")

if __name__ == "__main__":
    fix_existing_trajectory()