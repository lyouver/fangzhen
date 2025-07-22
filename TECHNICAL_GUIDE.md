# 🔧 Technical Guide - Underwater Coverage Planning System

## 📐 System Architecture Deep Dive

### Component Interaction Diagram

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Gazebo World  │    │   RexROV Robot  │    │ Trajectory Sys  │
│                 │    │                 │    │                 │
│ • Environment   │◄──►│ • Sensors       │◄──►│ • Path Planner  │
│ • Physics       │    │ • Thrusters     │    │ • Controller    │
│ • Terrain       │    │ • Dynamics      │    │ • Publisher     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │   RViz Visual   │
                    │                 │
                    │ • 3D Display    │
                    │ • Sensor Data   │
                    │ • Path Visual   │
                    └─────────────────┘
```

### Data Flow Architecture

```
Input Sources → Processing → Control → Output
     │              │          │        │
┌────▼────┐    ┌────▼────┐ ┌──▼───┐ ┌──▼────┐
│Trajectory│    │Path     │ │PID   │ │Thruster│
│YAML File │    │Planning │ │Ctrl  │ │Commands│
└─────────┘    └─────────┘ └──────┘ └───────┘
     │              │          │        │
┌────▼────┐    ┌────▼────┐ ┌──▼───┐ ┌──▼────┐
│Sensor   │    │State    │ │Error │ │Robot  │
│Feedback │    │Estimate │ │Calc  │ │Motion │
└─────────┘    └─────────┘ └──────┘ └───────┘
```

## 🏗️ Launch File Architecture

### System Launch Hierarchy

```
compact_terrain_rexrov.launch (Base System)
├── gazebo_ros/empty_world.launch
│   └── compact_underwater_terrain.world
├── uuv_assistants/publish_world_ned_frame.launch
├── uuv_descriptions/upload_rexrov_default.launch
└── rviz (with rexrov_default.rviz)

trajectory_20250720_121855_launch.launch (Control System)
├── uuv_thruster_manager/thruster_manager.launch
├── trajectory_20250720_121855_publisher.py
└── uuv_trajectory_control/rov_pid_controller.py
```

### Launch Parameters Matrix

| Parameter | Base Launch | Trajectory Launch | Description |
|-----------|-------------|-------------------|-------------|
| `uuv_name` | ✅ rexrov | ✅ rexrov | Robot namespace |
| `model_name` | ✅ rexrov | ✅ rexrov | Robot model type |
| `use_ned_frame` | ✅ true | ✅ true | Coordinate system |
| `gui` | ✅ true | ❌ N/A | Gazebo GUI |
| `paused` | ✅ false | ❌ N/A | Simulation state |

## 🤖 Robot Model Configuration

### RexROV Specifications

```xml
<!-- Physical Properties -->
<mass>1862.87</mass>
<inertia>
  <ixx>525.39</ixx>
  <iyy>794.20</iyy>
  <izz>691.23</izz>
</inertia>

<!-- Thruster Configuration -->
<thrusters count="8">
  <thruster id="0" position="[2.0, -1.0, 0.0]"/>
  <thruster id="1" position="[2.0, 1.0, 0.0]"/>
  <!-- ... additional thrusters -->
</thrusters>

<!-- Sensor Suite -->
<sensors>
  <imu name="imu_sensor"/>
  <pressure name="pressure_sensor"/>
  <camera name="forward_camera"/>
</sensors>
```

### Coordinate System Mapping

```
World NED Frame:
  X: North (Forward)
  Y: East (Right)  
  Z: Down (Depth)

Robot Body Frame:
  X: Forward
  Y: Starboard
  Z: Down

Gazebo World Frame:
  X: East
  Y: North
  Z: Up (Inverted)
```

## 🎯 Trajectory System Design

### Trajectory Data Structure

```python
@dataclass
class TrajectoryPoint:
    positions: List[float]      # [x, y, z, roll, pitch, yaw]
    velocities: List[float]     # [vx, vy, vz, wx, wy, wz]
    accelerations: List[float]  # [ax, ay, az, αx, αy, αz]
    effort: List[float]         # [fx, fy, fz, τx, τy, τz]
    time_from_start: Duration   # Timestamp
```

### Path Planning Algorithm

```python
def generate_coverage_path(terrain, bounds, spacing):
    """
    Generate coverage path using boustrophedon pattern
    
    Args:
        terrain: Height map data
        bounds: [x_min, x_max, y_min, y_max, z_min, z_max]
        spacing: Distance between parallel tracks
    
    Returns:
        List of waypoints with terrain-following heights
    """
    waypoints = []
    
    # Generate parallel tracks
    for y in range(bounds[2], bounds[3], spacing):
        if len(waypoints) % 2 == 0:
            # Left to right
            x_range = range(bounds[0], bounds[1], spacing)
        else:
            # Right to left (boustrophedon)
            x_range = range(bounds[1], bounds[0], -spacing)
        
        for x in x_range:
            z = get_terrain_height(terrain, x, y) + safety_offset
            waypoints.append(Waypoint(x, y, z))
    
    return waypoints
```

### Control System Implementation

```python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.previous_error = 0
    
    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.previous_error = error
        return output
```

## 🌊 Environment Configuration

### World File Structure

```xml
<world name="compact_underwater_terrain">
  <!-- Lighting -->
  <include><uri>model://sun</uri></include>
  
  <!-- Ocean Environment -->
  <model name="compact_ocean_box">
    <!-- Seafloor at Z=-50 -->
    <visual name="ground">
      <pose>0 0 -50 0 0 0</pose>
      <geometry><box><size>444 444 0.1</size></box></geometry>
    </visual>
    
    <!-- Water surface at Z=30 -->
    <visual name="surface">
      <pose>0 0 30 0 0 0</pose>
      <geometry><box><size>444 444 0.1</size></box></geometry>
    </visual>
    
    <!-- Water volume -->
    <visual name="main_water_body">
      <pose>0 0 -10 0 0 0</pose>
      <geometry><box><size>444 444 80</size></box></geometry>
    </visual>
  </model>
  
  <!-- Terrain -->
  <include>
    <uri>model://sand_heightmap</uri>
    <pose>0 0 -25 0 0 0</pose>
  </include>
  
  <!-- Current simulation -->
  <plugin name="underwater_current_plugin" 
          filename="libuuv_underwater_current_ros_plugin.so">
    <constant_current>
      <velocity><mean>0</mean><max>5</max></velocity>
    </constant_current>
  </plugin>
</world>
```

### Physics Configuration

```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>100</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
```

## 📡 Communication Protocols

### ROS Topic Architecture

```
/rexrov/
├── pose_gt                    # geometry_msgs/Pose
├── odom                       # nav_msgs/Odometry  
├── trajectory                 # trajectory_msgs/JointTrajectory
├── path_visualization         # nav_msgs/Path
├── cmd_vel                    # geometry_msgs/Twist
├── thruster_manager/
│   ├── input                  # uuv_gazebo_ros_plugins_msgs/FloatStamped[]
│   └── output                 # uuv_gazebo_ros_plugins_msgs/FloatStamped[]
└── sensors/
    ├── imu                    # sensor_msgs/Imu
    ├── pressure               # sensor_msgs/FluidPressure
    └── camera/image_raw       # sensor_msgs/Image
```

### Message Flow Timing

```
Time (ms)    Publisher              Topic                    Subscriber
0            trajectory_publisher   /rexrov/trajectory       trajectory_controller
10           trajectory_controller  /rexrov/cmd_vel          thruster_manager
20           thruster_manager       /rexrov/thrusters/*/input gazebo_plugin
30           gazebo_plugin          /rexrov/pose_gt          trajectory_controller
40           gazebo_plugin          /rexrov/sensors/*        rviz
```

## 🔧 Configuration Management

### Parameter Hierarchy

```yaml
# Global Parameters
/uuv_name: "rexrov"
/use_sim_time: true

# Trajectory Planner
/trajectory_planner/
  max_velocity: 2.0
  path_spacing: 25.0
  safety_height: 3.0
  
# PID Controller
/trajectory_controller/
  position/
    kp: [10.0, 10.0, 10.0]
    ki: [0.1, 0.1, 0.1]
    kd: [1.0, 1.0, 1.0]
  orientation/
    kp: [5.0, 5.0, 5.0]
    ki: [0.05, 0.05, 0.05]
    kd: [0.5, 0.5, 0.5]

# Thruster Manager
/rexrov/thruster_manager/
  timeout: 5.0
  max_thrust: 1000.0
```

### Environment Variables

```bash
# ROS Configuration
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find underwater_coverage_planning)/models

# Gazebo Configuration  
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(rospack find underwater_coverage_planning)
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/$ROS_DISTRO/lib
```

## 🧪 Testing Framework

### Unit Tests

```python
import unittest
from underwater_coverage_planning import TrajectoryPublisher

class TestTrajectoryPublisher(unittest.TestCase):
    def setUp(self):
        self.publisher = TrajectoryPublisher()
    
    def test_load_trajectory(self):
        """Test trajectory file loading"""
        trajectory = self.publisher.load_trajectory('test_trajectory.yaml')
        self.assertIsNotNone(trajectory)
        self.assertIn('points', trajectory)
    
    def test_coordinate_transform(self):
        """Test coordinate system transformation"""
        ned_point = [10, 20, -5]
        gazebo_point = self.publisher.ned_to_gazebo(ned_point)
        self.assertEqual(gazebo_point, [20, 10, 5])
```

### Integration Tests

```bash
#!/bin/bash
# integration_test.sh

# Start roscore
roscore &
ROSCORE_PID=$!

# Launch system
roslaunch underwater_coverage_planning compact_terrain_rexrov.launch &
GAZEBO_PID=$!

# Wait for initialization
sleep 30

# Run tests
python -m pytest tests/integration/

# Cleanup
kill $GAZEBO_PID $ROSCORE_PID
```

### Performance Benchmarks

```python
def benchmark_trajectory_following():
    """Benchmark trajectory following accuracy"""
    start_time = time.time()
    
    # Execute trajectory
    execute_trajectory('test_trajectory.yaml')
    
    # Measure performance
    execution_time = time.time() - start_time
    position_error = calculate_position_error()
    
    assert execution_time < 300  # 5 minutes max
    assert position_error < 0.5  # 0.5m accuracy
```

## 🔍 Debugging Tools

### Log Analysis

```bash
# ROS Logs
tail -f ~/.ros/log/latest/rosout.log

# Gazebo Logs  
tail -f ~/.gazebo/server-11345/default.log

# Custom Logging
rosrun underwater_coverage_planning monitor_movement.py --verbose
```

### Performance Monitoring

```python
# CPU/Memory monitoring
import psutil

def monitor_system_resources():
    cpu_percent = psutil.cpu_percent(interval=1)
    memory_info = psutil.virtual_memory()
    
    rospy.loginfo(f"CPU: {cpu_percent}%, Memory: {memory_info.percent}%")
```

### Network Analysis

```bash
# Topic bandwidth
rostopic bw /rexrov/trajectory

# Message frequency
rostopic hz /rexrov/pose_gt

# Network latency
rostopic delay /rexrov/cmd_vel
```

## 🚀 Performance Optimization

### Gazebo Optimization

```xml
<!-- Reduce physics update rate -->
<max_step_size>0.02</max_step_size>
<real_time_factor>0.8</real_time_factor>

<!-- Disable unnecessary features -->
<gui>
  <camera name="user_camera">
    <pose>0 0 20 0 1.57 0</pose>
  </camera>
</gui>
```

### ROS Optimization

```python
# Reduce message publishing rate
rospy.Timer(rospy.Duration(0.1), self.publish_trajectory)  # 10Hz instead of 100Hz

# Use efficient data structures
import numpy as np
positions = np.array(trajectory_points, dtype=np.float32)
```

### Memory Management

```cpp
// C++ node optimization
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "optimized_node");
    ros::NodeHandle nh;
    
    // Pre-allocate message buffers
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
    
    // Use efficient publishers
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    return 0;
}
```

## 📊 Metrics and Analytics

### Key Performance Indicators

```python
class SystemMetrics:
    def __init__(self):
        self.trajectory_accuracy = []
        self.execution_time = []
        self.energy_consumption = []
    
    def calculate_trajectory_accuracy(self, planned, actual):
        """Calculate RMS error between planned and actual trajectory"""
        errors = np.array(planned) - np.array(actual)
        rms_error = np.sqrt(np.mean(errors**2))
        return rms_error
    
    def log_performance(self):
        """Log system performance metrics"""
        avg_accuracy = np.mean(self.trajectory_accuracy)
        avg_time = np.mean(self.execution_time)
        
        rospy.loginfo(f"Average Accuracy: {avg_accuracy:.3f}m")
        rospy.loginfo(f"Average Execution Time: {avg_time:.1f}s")
```

### Data Collection

```bash
# Record system data
rosbag record -a -O system_performance.bag

# Analyze recorded data
rostopic echo -b system_performance.bag -p /rexrov/pose_gt > trajectory_data.csv
```

## 🔐 Security Considerations

### Network Security

```python
# Secure ROS communication
import rospy
from rospy import ROSException

def secure_publisher():
    try:
        pub = rospy.Publisher('/secure_topic', String, queue_size=10)
        # Add authentication/encryption here
    except ROSException as e:
        rospy.logerr(f"Security error: {e}")
```

### Input Validation

```python
def validate_trajectory_input(trajectory_data):
    """Validate trajectory input for safety"""
    for point in trajectory_data['points']:
        # Check position bounds
        if not (-250 <= point['positions'][0] <= 250):
            raise ValueError("X position out of bounds")
        
        # Check depth limits
        if point['positions'][2] > 0:
            raise ValueError("Invalid depth (above water)")
        
        # Check velocity limits
        if max(abs(v) for v in point['velocities']) > 5.0:
            raise ValueError("Velocity exceeds safety limits")
```

---

This technical guide provides comprehensive implementation details for developers working with the underwater coverage planning system. For user-focused documentation, refer to the main README.md file.