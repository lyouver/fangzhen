#!/usr/bin/env python3
"""
TF树生成器
用于生成当前ROS系统中的TF树frames.pdf文件
适用于同时启动compact_terrain_rexrov_with_trajectory和trajectory_launch的环境
"""

import os
import subprocess
import time
import sys
import shutil
from datetime import datetime

class TFFramesGenerator:
    def __init__(self):
        self.output_dir = os.path.join(os.path.dirname(__file__), 'tf_frames')
        self.ensure_output_dir()
        
    def ensure_output_dir(self):
        """确保输出目录存在"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建输出目录: {self.output_dir}")
    
    def check_ros_master(self):
        """检查ROS master是否运行"""
        try:
            result = subprocess.run(['rostopic', 'list'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=5)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    def check_tf_topics(self):
        """检查TF相关话题是否存在"""
        try:
            result = subprocess.run(['rostopic', 'list'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=5)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                tf_topics = [topic for topic in topics if 'tf' in topic.lower()]
                print(f"发现TF相关话题: {tf_topics}")
                return len(tf_topics) > 0
            return False
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    def wait_for_tf_data(self, timeout=30):
        """等待TF数据可用"""
        print("等待TF数据...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # 检查是否有TF数据
                result = subprocess.run(['rostopic', 'echo', '/tf', '-n', '1'], 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=3)
                if result.returncode == 0 and result.stdout.strip():
                    print("TF数据已可用")
                    return True
            except subprocess.TimeoutExpired:
                pass
            
            print(".", end="", flush=True)
            time.sleep(1)
        
        print("\n警告: 等待TF数据超时")
        return False
    
    def generate_frames_pdf(self):
        """生成TF树的frames.pdf文件"""
        print("生成TF树frames.pdf...")
        
        # 切换到输出目录
        original_dir = os.getcwd()
        os.chdir(self.output_dir)
        
        try:
            # 运行rosrun tf view_frames
            result = subprocess.run(['rosrun', 'tf', 'view_frames'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=30)
            
            if result.returncode == 0:
                print("TF树生成成功")
                print(f"输出: {result.stdout}")
                return True
            else:
                print(f"TF树生成失败: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("TF树生成超时")
            return False
        except FileNotFoundError:
            print("错误: 找不到rosrun命令，请确保ROS环境已正确设置")
            return False
        finally:
            os.chdir(original_dir)
    
    def rename_and_organize_files(self):
        """重命名和整理生成的文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 查找生成的文件
        frames_pdf = os.path.join(self.output_dir, 'frames.pdf')
        frames_gv = os.path.join(self.output_dir, 'frames.gv')
        
        if os.path.exists(frames_pdf):
            # 重命名PDF文件
            new_pdf_name = f"tf_frames_{timestamp}.pdf"
            new_pdf_path = os.path.join(self.output_dir, new_pdf_name)
            shutil.move(frames_pdf, new_pdf_path)
            print(f"PDF文件已保存为: {new_pdf_path}")
            
            # 如果存在.gv文件，也重命名
            if os.path.exists(frames_gv):
                new_gv_name = f"tf_frames_{timestamp}.gv"
                new_gv_path = os.path.join(self.output_dir, new_gv_name)
                shutil.move(frames_gv, new_gv_path)
                print(f"GraphViz文件已保存为: {new_gv_path}")
            
            return new_pdf_path
        else:
            print("错误: 未找到生成的frames.pdf文件")
            return None
    
    def list_tf_frames(self):
        """列出当前的TF框架"""
        print("当前TF框架列表:")
        try:
            result = subprocess.run(['rosrun', 'tf', 'tf_monitor'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=10)
            if result.returncode == 0:
                print(result.stdout)
            else:
                print("无法获取TF框架信息")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            print("tf_monitor命令执行失败")
    
    def check_specific_transforms(self):
        """检查特定的坐标系转换"""
        transforms_to_check = [
            ('world', 'world_ned'),
            ('world', 'rexrov/base_link'),
            ('rexrov/base_link', 'rexrov/base_link_ned'),
            ('world_ned', 'rexrov/base_link_ned')
        ]
        
        print("\n检查关键坐标系转换:")
        for source, target in transforms_to_check:
            try:
                result = subprocess.run(['rosrun', 'tf', 'tf_echo', source, target], 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=3)
                if result.returncode == 0:
                    print(f"✓ {source} -> {target}: 可用")
                else:
                    print(f"✗ {source} -> {target}: 不可用")
            except subprocess.TimeoutExpired:
                print(f"✗ {source} -> {target}: 超时")
    
    def open_pdf_viewer(self, pdf_path):
        """尝试打开PDF查看器"""
        if not pdf_path or not os.path.exists(pdf_path):
            return
        
        try:
            # 尝试不同的PDF查看器
            viewers = ['evince', 'okular', 'xpdf', 'acroread']
            for viewer in viewers:
                if shutil.which(viewer):
                    print(f"使用{viewer}打开PDF文件...")
                    subprocess.Popen([viewer, pdf_path])
                    return
            
            print("未找到PDF查看器，请手动打开文件:")
            print(f"文件路径: {pdf_path}")
            
        except Exception as e:
            print(f"打开PDF文件时出错: {e}")
    
    def generate_report(self, pdf_path):
        """生成TF树分析报告"""
        report_path = os.path.join(self.output_dir, f"tf_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("TF树分析报告\n")
            f.write("=" * 50 + "\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"PDF文件: {pdf_path}\n\n")
            
            # 获取话题列表
            try:
                result = subprocess.run(['rostopic', 'list'], 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=5)
                if result.returncode == 0:
                    f.write("ROS话题列表:\n")
                    f.write(result.stdout)
                    f.write("\n")
            except:
                pass
            
            # 获取节点列表
            try:
                result = subprocess.run(['rosnode', 'list'], 
                                      capture_output=True, 
                                      text=True, 
                                      timeout=5)
                if result.returncode == 0:
                    f.write("ROS节点列表:\n")
                    f.write(result.stdout)
                    f.write("\n")
            except:
                pass
            
            f.write("关键坐标系转换检查:\n")
            transforms_to_check = [
                ('world', 'world_ned'),
                ('world', 'rexrov/base_link'),
                ('rexrov/base_link', 'rexrov/base_link_ned'),
                ('world_ned', 'rexrov/base_link_ned')
            ]
            
            for source, target in transforms_to_check:
                try:
                    result = subprocess.run(['rosrun', 'tf', 'tf_echo', source, target], 
                                          capture_output=True, 
                                          text=True, 
                                          timeout=3)
                    if result.returncode == 0:
                        f.write(f"✓ {source} -> {target}: 可用\n")
                    else:
                        f.write(f"✗ {source} -> {target}: 不可用\n")
                except:
                    f.write(f"✗ {source} -> {target}: 检查失败\n")
        
        print(f"分析报告已保存: {report_path}")
        return report_path

def main():
    print("=== TF树生成器 ===")
    print("此脚本用于生成compact_terrain_rexrov_with_trajectory和trajectory_launch同时运行时的TF树")
    print()
    
    generator = TFFramesGenerator()
    
    # 检查ROS环境
    if not generator.check_ros_master():
        print("错误: ROS master未运行")
        print("请先启动以下launch文件:")
        print("1. roslaunch underwater_coverage_planning compact_terrain_rexrov.launch")
        print("2. roslaunch underwater_coverage_planning trajectory_20250720_121855_launch.launch")
        sys.exit(1)
    
    print("✓ ROS master正在运行")
    
    # 检查TF话题
    if not generator.check_tf_topics():
        print("警告: 未发现TF相关话题")
        print("请确保已启动相关的launch文件")
    
    # 等待TF数据
    if not generator.wait_for_tf_data():
        print("警告: TF数据可能不完整，但仍尝试生成TF树")
    
    # 列出当前TF框架
    generator.list_tf_frames()
    
    # 检查特定转换
    generator.check_specific_transforms()
    
    # 生成TF树
    if generator.generate_frames_pdf():
        pdf_path = generator.rename_and_organize_files()
        if pdf_path:
            print(f"\n✓ TF树PDF文件生成成功: {pdf_path}")
            
            # 生成分析报告
            report_path = generator.generate_report(pdf_path)
            
            # 尝试打开PDF文件
            generator.open_pdf_viewer(pdf_path)
            
            print("\n使用说明:")
            print("1. 查看生成的PDF文件以了解TF树结构")
            print("2. 检查是否存在断开的坐标系转换")
            print("3. 确认轨迹坐标系(world_ned)与机器人坐标系的连接")
            print(f"4. 查看详细分析报告: {report_path}")
        else:
            print("✗ TF树PDF文件生成失败")
            sys.exit(1)
    else:
        print("✗ 无法生成TF树")
        sys.exit(1)

if __name__ == "__main__":
    main()