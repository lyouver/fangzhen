#!/usr/bin/env python3
"""
虚拟机版本的交互式3D覆盖路径规划可视化器
结合地形显示和实时路径规划过程，并保存轨迹文件
适用于虚拟机环境，保留ROS路径配置
"""

import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import numpy as np
import trimesh
import time
import os
import json
import rospkg
from typing import List, Tuple, Optional
from dataclasses import dataclass
from datetime import datetime

@dataclass
class Waypoint3D:
    """三维航点"""
    x: float
    y: float
    z: float
    yaw: float = 0.0
    timestamp: float = 0.0

class VMInteractiveCoveragePlanner:
    """虚拟机版本的交互式覆盖路径规划器"""
    
    def __init__(self):
        # 配置参数
        self.x_min, self.x_max = -200.0, 200.0
        self.y_min, self.y_max = -200.0, 200.0
        self.z_min, self.z_max = -25.0, 1.0
        
        self.path_spacing = 25.0      # 路径间距
        self.safety_height = 3.0      # 安全高度
        self.waypoint_spacing = 10.0  # 航点间距
        self.max_velocity = 2.0       # 最大速度 m/s
        
        # 地形数据
        self.mesh = None
        self.terrain_vertices = None
        self.terrain_faces = None
        self.height_map = None
        
        # 路径数据
        self.waypoints = []
        self.current_path_index = 0
        
        # Plotly图形
        self.fig = None
        
        # 确保guiji文件夹存在
        self.output_dir = os.path.join(os.path.dirname(__file__), 'guiji')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建输出目录: {self.output_dir}")
        
    def load_terrain(self, dae_file_path: str):
        """加载地形数据"""
        print(f"正在加载地形文件: {dae_file_path}")
        
        if not os.path.exists(dae_file_path):
            raise FileNotFoundError(f"地形文件不存在: {dae_file_path}")
        
        # 加载DAE文件
        scene = trimesh.load(dae_file_path, process=False)
        
        if isinstance(scene, trimesh.Scene):
            if len(scene.geometry) == 0:
                raise ValueError("DAE文件中没有几何体")
            self.mesh = list(scene.geometry.values())[0]
        else:
            self.mesh = scene
        
        # 应用尺度和偏移（与visualize_terrain_3d.py保持一致）
        scale_factor = 50.0
        self.mesh.vertices *= scale_factor
        self.mesh.vertices[:, 2] -= 25.0  # Z轴偏移
        
        self.terrain_vertices = self.mesh.vertices
        self.terrain_faces = self.mesh.faces
        
        print(f"地形加载成功:")
        print(f"  顶点数: {len(self.terrain_vertices)}")
        print(f"  面数: {len(self.terrain_faces)}")
        print(f"  边界: {self.mesh.bounds}")
        
        # 生成高度图用于路径规划
        self._generate_height_map()
    
    def _generate_height_map(self):
        """生成高度图"""
        print("生成地形高度图...")
        
        # 创建网格
        grid_resolution = 5.0  # 网格分辨率
        x_range = np.arange(self.x_min, self.x_max + grid_resolution, grid_resolution)
        y_range = np.arange(self.y_min, self.y_max + grid_resolution, grid_resolution)
        
        X, Y = np.meshgrid(x_range, y_range)
        Z = np.zeros_like(X)
        
        # 对每个网格点查询地形高度
        vertices = self.terrain_vertices
        
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                x, y = X[i, j], Y[i, j]
                
                # 找到最近的地形点
                distances = np.sqrt((vertices[:, 0] - x)**2 + (vertices[:, 1] - y)**2)
                nearest_idx = np.argmin(distances)
                
                if distances[nearest_idx] < grid_resolution * 2:
                    Z[i, j] = vertices[nearest_idx, 2]
                else:
                    Z[i, j] = self.z_min
        
        self.height_map = {'X': X, 'Y': Y, 'Z': Z}
        print(f"高度图生成完成，高度范围: {Z.min():.2f} 到 {Z.max():.2f}")
    
    def get_terrain_height(self, x: float, y: float) -> float:
        """获取指定位置的地形高度"""
        if self.height_map is None:
            return self.z_min
        
        X, Y, Z = self.height_map['X'], self.height_map['Y'], self.height_map['Z']
        
        # 边界检查
        if (x < self.x_min or x > self.x_max or 
            y < self.y_min or y > self.y_max):
            return self.z_min
        
        try:
            # 找到最近的网格点
            grid_res = 5.0
            x_idx = int((x - self.x_min) / grid_res)
            y_idx = int((y - self.y_min) / grid_res)
            
            # 边界处理
            x_idx = max(0, min(x_idx, Z.shape[1] - 1))
            y_idx = max(0, min(y_idx, Z.shape[0] - 1))
            
            return Z[y_idx, x_idx]
        except:
            return self.z_min
    
    def create_initial_visualization(self):
        """创建初始的3D可视化"""
        print("创建交互式3D可视化...")
        
        # 创建地形网格
        x = self.terrain_vertices[:, 0]
        y = self.terrain_vertices[:, 1]
        z = self.terrain_vertices[:, 2]
        faces = self.terrain_faces
        
        # 创建Plotly图形
        self.fig = go.Figure()
        
        # 添加地形
        self.fig.add_trace(go.Mesh3d(
            x=x,
            y=y,
            z=z,
            i=faces[:, 0],
            j=faces[:, 1],
            k=faces[:, 2],
            intensity=z,
            colorscale='earth',
            showscale=True,
            colorbar=dict(
                title="高度 (m)",
                x=1.02
            ),
            lighting=dict(
                ambient=0.4,
                diffuse=0.8,
                specular=0.1
            ),
            lightposition=dict(x=100, y=100, z=100),
            name="地形",
            opacity=0.8
        ))
        
        # 添加覆盖区域边界
        self._add_coverage_boundary()
        
        # 设置布局
        self.fig.update_layout(
            title={
                'text': 'UUV 3D覆盖路径规划 - 虚拟机交互式可视化',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            scene=dict(
                xaxis_title='X (m)',
                yaxis_title='Y (m)',
                zaxis_title='Z (m)',
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.0)
                ),
                aspectmode='data'
            ),
            width=1400,
            height=900,
            margin=dict(l=0, r=0, t=50, b=0)
        )
        
        print("初始可视化创建完成")
        return self.fig
    
    def _add_coverage_boundary(self):
        """添加覆盖区域边界"""
        # 创建边界框
        boundary_x = [self.x_min, self.x_max, self.x_max, self.x_min, self.x_min]
        boundary_y = [self.y_min, self.y_min, self.y_max, self.y_max, self.y_min]
        boundary_z = [self.z_max] * 5
        
        self.fig.add_trace(go.Scatter3d(
            x=boundary_x,
            y=boundary_y,
            z=boundary_z,
            mode='lines',
            line=dict(color='red', width=5),
            name='覆盖区域边界'
        ))
    
    def plan_and_visualize_coverage(self):
        """规划并可视化覆盖路径"""
        print("开始交互式路径规划...")
        
        # 先规划所有路径
        self._plan_coverage_paths()
        
        print("路径规划完成！")
        print(f"总共生成 {len(self.waypoints)} 个航点")
        
        # 添加所有路径到可视化
        self._add_all_paths_to_visualization()
        
        # 添加时间戳
        self._add_timestamps()
        
        # 保存轨迹文件
        self._save_trajectory_file()
        
        # 显示完整的交互式图形
        # 配置为可从虚拟机访问
        import plotly.offline as pyo
        
        # 方法1: 保存为HTML文件
        html_file = os.path.join(self.output_dir, f"coverage_visualization_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html")
        pyo.plot(self.fig, filename=html_file, auto_open=True)
        print(f"可视化已保存为: {html_file}")
        print("你可以将此HTML文件复制到其他机器中用浏览器打开")
        
        # 方法2: 尝试在指定端口显示（如果需要网络访问）
        try:
            self.fig.show(config={'displayModeBar': True})
        except:
            print("无法启动交互式服务器，请使用HTML文件")
    
    def _plan_coverage_paths(self):
        """规划覆盖路径"""
        # 计算路径数量
        coverage_width = self.y_max - self.y_min
        num_paths = int(coverage_width / self.path_spacing) + 1
        
        print(f"规划 {num_paths} 条覆盖路径，间距: {self.path_spacing}m")
        
        self.path_segments = []  # 存储路径段
        
        for path_idx in range(num_paths):
            # 计算当前路径的Y坐标
            y = self.y_min + path_idx * self.path_spacing
            
            if y > self.y_max:
                break
            
            print(f"规划第 {path_idx + 1}/{num_paths} 条路径 (y={y:.1f})")
            
            # 交替方向（之字形）
            if path_idx % 2 == 0:
                # 从左到右
                x_start, x_end = self.x_min, self.x_max
                x_step = self.waypoint_spacing
            else:
                # 从右到左
                x_start, x_end = self.x_max, self.x_min
                x_step = -self.waypoint_spacing
            
            # 生成当前路径的航点
            path_waypoints = self._generate_path_waypoints(x_start, x_end, x_step, y)
            
            if path_waypoints:
                self.path_segments.append((path_idx, path_waypoints))
                self.waypoints.extend(path_waypoints)
    
    def _generate_path_waypoints(self, x_start: float, x_end: float, x_step: float, y: float) -> List[Waypoint3D]:
        """生成单条路径的航点"""
        waypoints = []
        
        if x_step > 0:
            x_positions = np.arange(x_start, x_end + x_step, x_step)
        else:
            x_positions = np.arange(x_start, x_end + x_step, x_step)
        
        for x in x_positions:
            # 获取地形高度
            terrain_height = self.get_terrain_height(x, y)
            
            # 计算安全飞行高度
            flight_height = terrain_height + self.safety_height
            
            # 确保在指定的Z范围内
            flight_height = max(self.z_min, min(flight_height, self.z_max))
            
            # 计算航向角
            if waypoints:
                prev_wp = waypoints[-1]
                yaw = np.arctan2(y - prev_wp.y, x - prev_wp.x)
            else:
                yaw = 0.0
            
            waypoint = Waypoint3D(x=x, y=y, z=flight_height, yaw=yaw)
            waypoints.append(waypoint)
        
        return waypoints
    
    def _add_all_paths_to_visualization(self):
        """添加所有路径到可视化"""
        if not hasattr(self, 'path_segments') or not self.path_segments:
            return
        
        # 路径颜色
        colors = ['blue', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta']
        
        for path_index, path_waypoints in self.path_segments:
            if not path_waypoints:
                continue
            
            # 提取坐标
            xs = [wp.x for wp in path_waypoints]
            ys = [wp.y for wp in path_waypoints]
            zs = [wp.z for wp in path_waypoints]
            
            color = colors[path_index % len(colors)]
            
            # 添加路径线
            self.fig.add_trace(go.Scatter3d(
                x=xs,
                y=ys,
                z=zs,
                mode='lines+markers',
                line=dict(color=color, width=4),
                marker=dict(size=3, color=color),
                name=f'路径 {path_index + 1}',
                showlegend=True
            ))
            
            print(f"  添加路径 {path_index + 1}: {len(path_waypoints)} 个航点")
        
        # 添加起点和终点标记
        if self.waypoints:
            # 起点
            start_wp = self.waypoints[0]
            self.fig.add_trace(go.Scatter3d(
                x=[start_wp.x],
                y=[start_wp.y],
                z=[start_wp.z],
                mode='markers',
                marker=dict(size=12, color='green', symbol='diamond'),
                name='起点',
                showlegend=True
            ))
            
            # 终点
            end_wp = self.waypoints[-1]
            self.fig.add_trace(go.Scatter3d(
                x=[end_wp.x],
                y=[end_wp.y],
                z=[end_wp.z],
                mode='markers',
                marker=dict(size=12, color='red', symbol='diamond'),
                name='终点',
                showlegend=True
            ))
            
            # 完整轨迹连线
            all_xs = [wp.x for wp in self.waypoints]
            all_ys = [wp.y for wp in self.waypoints]
            all_zs = [wp.z for wp in self.waypoints]
            
            self.fig.add_trace(go.Scatter3d(
                x=all_xs,
                y=all_ys,
                z=all_zs,
                mode='lines',
                line=dict(color='red', width=2, dash='dash'),
                name='完整轨迹',
                opacity=0.7,
                showlegend=True
            ))
        
        # 更新标题显示统计信息
        self._update_title_with_stats()
    
    def _update_title_with_stats(self):
        """更新标题显示统计信息"""
        if not self.waypoints:
            return
        
        # 计算统计信息
        total_distance = sum(
            np.sqrt((self.waypoints[i].x - self.waypoints[i-1].x)**2 + 
                   (self.waypoints[i].y - self.waypoints[i-1].y)**2 + 
                   (self.waypoints[i].z - self.waypoints[i-1].z)**2)
            for i in range(1, len(self.waypoints))
        )
        
        coverage_area = (self.x_max - self.x_min) * (self.y_max - self.y_min) / 1000000  # 平方公里
        
        # 更新标题显示统计信息
        self.fig.update_layout(
            title={
                'text': f'UUV 3D覆盖路径规划完成<br>'
                       f'航点数: {len(self.waypoints)} | '
                       f'路径长度: {total_distance:.1f}m | '
                       f'覆盖面积: {coverage_area:.2f}km²',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 16}
            }
        )
        
        print(f"\n=== 路径统计 ===")
        print(f"总航点数: {len(self.waypoints)}")
        print(f"总路径长度: {total_distance:.2f} 米")
        print(f"覆盖面积: {coverage_area:.2f} 平方公里")
    
    def _add_timestamps(self):
        """为航点添加时间戳"""
        if len(self.waypoints) < 2:
            return
        
        current_time = 0.0
        self.waypoints[0].timestamp = current_time
        
        for i in range(1, len(self.waypoints)):
            prev_wp = self.waypoints[i-1]
            current_wp = self.waypoints[i]
            
            # 计算距离
            distance = np.sqrt((current_wp.x - prev_wp.x)**2 + 
                             (current_wp.y - prev_wp.y)**2 + 
                             (current_wp.z - prev_wp.z)**2)
            
            # 计算时间
            travel_time = distance / self.max_velocity
            current_time += travel_time
            
            current_wp.timestamp = current_time
    
    def _save_trajectory_file(self):
        """保存轨迹文件"""
        if not self.waypoints:
            print("没有航点数据，跳过轨迹保存")
            return
        
        # 创建轨迹数据
        trajectory_data = {
            'metadata': {
                'generated_time': datetime.now().isoformat(),
                'total_waypoints': len(self.waypoints),
                'total_time': self.waypoints[-1].timestamp if self.waypoints else 0,
                'coverage_area': {
                    'x_range': [self.x_min, self.x_max],
                    'y_range': [self.y_min, self.y_max],
                    'z_range': [self.z_min, self.z_max]
                },
                'planning_parameters': {
                    'path_spacing': self.path_spacing,
                    'safety_height': self.safety_height,
                    'waypoint_spacing': self.waypoint_spacing,
                    'max_velocity': self.max_velocity
                }
            },
            'waypoints': [
                {
                    'x': wp.x,
                    'y': wp.y,
                    'z': wp.z,
                    'yaw': wp.yaw,
                    'timestamp': wp.timestamp
                }
                for wp in self.waypoints
            ]
        }
        
        # 保存JSON文件
        trajectory_file = os.path.join(self.output_dir, f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        with open(trajectory_file, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        print(f"\n轨迹已保存到: {trajectory_file}")
        
        # 计算并显示额外统计信息
        total_distance = sum(
            np.sqrt((self.waypoints[i].x - self.waypoints[i-1].x)**2 + 
                   (self.waypoints[i].y - self.waypoints[i-1].y)**2 + 
                   (self.waypoints[i].z - self.waypoints[i-1].z)**2)
            for i in range(1, len(self.waypoints))
        )
        
        print(f"预计执行时间: {self.waypoints[-1].timestamp:.2f} 秒")
        print(f"平均速度: {total_distance/self.waypoints[-1].timestamp:.2f} m/s")

def main():
    """主函数"""
    print("=== 虚拟机版本 UUV 3D覆盖路径规划器 ===")
    
    try:
        # 创建规划器
        planner = VMInteractiveCoveragePlanner()
        
        # 获取ROS包路径
        rospack = rospkg.RosPack()
        heightmap_path = os.path.join(rospack.get_path('uuv_gazebo_worlds'), 
                                      'models/sand_heightmap/meshes/heightmap.dae')
        
        # 加载地形
        planner.load_terrain(heightmap_path)
        
        # 创建初始可视化
        fig = planner.create_initial_visualization()
        
        # 询问是否开始路径规划
        input("\n按回车键开始交互式路径规划...")
        
        # 开始规划和可视化
        planner.plan_and_visualize_coverage()
        
        print("\n规划完成！你可以在浏览器中交互式查看结果：")
        print("- 鼠标左键拖拽：旋转视角")
        print("- 鼠标滚轮：缩放")
        print("- 鼠标右键拖拽：平移")
        print("- 双击：重置视角")
        
        print(f"\n生成的文件保存在 '{planner.output_dir}' 文件夹中")
        print("接下来可以运行 trajectory_to_ros.py 转换为ROS格式")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()