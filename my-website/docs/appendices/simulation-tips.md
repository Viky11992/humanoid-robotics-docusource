---
sidebar_position: 2
title: Simulation Tips and Best Practices
---

# Simulation Tips and Best Practices

## Overview

Simulation is a critical component of humanoid robotics development, allowing for safe testing, algorithm validation, and rapid prototyping. This guide provides tips and best practices for effective simulation in ROS 2 environments.

## Gazebo Simulation

### Performance Optimization

#### Physics Engine Configuration
```xml
<!-- In your world file or SDF model -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller for stability -->
  <real_time_factor>1.0</real_time_factor>  <!-- Target real-time performance -->
  <real_time_update_rate>1000.0</real_time_update_rate>  <!-- Update rate in Hz -->
</physics>
```

#### Model Optimization
- **Reduce visual complexity**: Use simplified collision models
- **Limit joints**: Only include necessary degrees of freedom
- **Optimize meshes**: Use low-poly models for real-time performance
- **Use instancing**: For multiple identical objects

### Model Creation Best Practices

#### URDF/XACRO Guidelines
```xml
<!-- Good: Use macros for repeated elements -->
<xacro:macro name="wheel" params="prefix *origin">
  <joint name="${prefix}_wheel_joint" type="continuous">
    <xacro:insert_block name="origin"/>
    <axis xyz="0 1 0"/>
    <parent link="${prefix}_link"/>
    <child link="${prefix}_wheel_link"/>
  </joint>

  <link name="${prefix}_wheel_link">
    <visual>
      <geometry>
        <mesh filename="package://my_robot/meshes/wheel.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</xacro:macro>
```

#### Collision vs Visual Models
- **Visual models**: High-resolution for rendering
- **Collision models**: Simplified for physics (convex hulls, primitive shapes)
- **Use multiple collision elements** for complex shapes

### Sensor Configuration

#### Camera Sensors
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>100</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU Sensors
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## NVIDIA Isaac Sim

### USD Scene Optimization
- **Use instancing** for repeated objects
- **Level of Detail (LOD)** for distant objects
- **Occlusion culling** to hide non-visible objects
- **Texture streaming** for large environments

### Robot Configuration for Isaac Sim
```python
# In Isaac Sim Python script
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World(stage_units_in_meters=1.0)

# Add robot to stage
assets_root_path = get_assets_root_path()
robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

# Configure physics
world.scene.add_default_ground_plane()
```

## Simulation Testing Strategies

### Unit Testing in Simulation
```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

class TestNavigation(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_navigation_node')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Publisher for sending commands
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for robot pose
        self.pose_sub = self.node.create_subscription(
            Pose, '/robot_pose', self.pose_callback, 10
        )

        self.initial_pose = None
        self.current_pose = None

    def pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg
        self.current_pose = msg

    def test_move_forward(self):
        """Test that robot moves forward when given forward command."""
        # Send forward command
        cmd = Twist()
        cmd.linear.x = 1.0
        self.cmd_vel_pub.publish(cmd)

        # Wait for movement
        start_time = self.node.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=5.0)

        while (self.current_pose is None or
               self.initial_pose is None) and \
              (self.node.get_clock().now() - start_time) < timeout:
            self.executor.spin_once(timeout_sec=0.1)

        # Check that robot moved
        if self.initial_pose and self.current_pose:
            self.assertGreater(
                self.current_pose.position.x,
                self.initial_pose.position.x
            )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

### Integration Testing
```python
import subprocess
import time
import signal
import rospy
from std_msgs.msg import String

class SimulationIntegrationTest:
    def __init__(self):
        rospy.init_node('sim_integration_test')
        self.test_results = rospy.Publisher('/test_results', String, queue_size=10)

    def run_simulation_test(self):
        """Run complete simulation test scenario."""
        # Launch Gazebo with specific world
        gazebo_process = subprocess.Popen([
            'ros2', 'launch', 'gazebo_ros', 'empty_world.launch.py',
            'world:=/path/to/test.world'
        ])

        time.sleep(5)  # Wait for Gazebo to start

        # Launch robot bringup
        bringup_process = subprocess.Popen([
            'ros2', 'launch', 'my_robot_bringup', 'robot.launch.py'
        ])

        time.sleep(10)  # Wait for robot to spawn

        # Run test scenarios
        self.execute_test_scenarios()

        # Cleanup
        bringup_process.terminate()
        gazebo_process.terminate()

    def execute_test_scenarios(self):
        """Execute specific test scenarios."""
        # Test 1: Navigation to goal
        self.test_navigation()

        # Test 2: Object manipulation
        self.test_manipulation()

        # Test 3: Sensor accuracy
        self.test_sensors()

    def test_navigation(self):
        """Test navigation stack."""
        # Send navigation goal
        # Monitor progress
        # Verify success/failure
        pass

if __name__ == '__main__':
    tester = SimulationIntegrationTest()
    tester.run_simulation_test()
```

## Performance Monitoring in Simulation

### CPU/GPU Monitoring
```bash
# Monitor Gazebo server process
htop -p $(pgrep gzserver)

# Monitor GPU usage (NVIDIA)
nvidia-smi -l 1

# Monitor ROS 2 processes
ros2 topic hz /clock  # Check simulation time rate
```

### Simulation Quality Metrics
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimulationMetrics(Node):
    def __init__(self):
        super().__init__('simulation_metrics')

        self.metrics_pub = self.create_publisher(Float32, '/sim_metrics/real_time_factor', 10)
        self.timer = self.create_timer(1.0, self.publish_metrics)

        # Store simulation time for RTF calculation
        self.last_sim_time = None
        self.last_real_time = None

    def publish_metrics(self):
        # Calculate Real Time Factor (RTF)
        current_sim_time = self.get_clock().now().nanoseconds / 1e9
        current_real_time = time.time()

        if self.last_sim_time and self.last_real_time:
            sim_delta = current_sim_time - self.last_sim_time
            real_delta = current_real_time - self.last_real_time

            if real_delta > 0:
                rtf = sim_delta / real_delta
                rtf_msg = Float32()
                rtf_msg.data = rtf
                self.metrics_pub.publish(rtf_msg)

        self.last_sim_time = current_sim_time
        self.last_real_time = current_real_time
```

## Common Simulation Issues and Solutions

### Physics Instability
**Problem**: Robot shaking, parts flying apart
**Solutions**:
- Reduce `max_step_size` in physics configuration
- Increase solver iterations
- Adjust joint damping and friction
- Verify inertial properties

### Performance Issues
**Problem**: Low simulation speed, dropped frames
**Solutions**:
- Simplify collision models
- Reduce physics update rate
- Use faster physics engine (DART vs ODE)
- Optimize visual models

### Sensor Noise
**Problem**: Unrealistic sensor data
**Solutions**:
- Add realistic noise models
- Calibrate noise parameters to real hardware
- Use sensor fusion to reduce noise effects

## Simulation-to-Real Transfer (Sim-to-Real)

### Domain Randomization
```python
import random

class DomainRandomizer:
    def __init__(self):
        self.randomization_params = {
            'friction': (0.8, 1.2),  # Range for friction coefficients
            'mass': (0.9, 1.1),      # Range for mass multipliers
            'gravity': (0.95, 1.05), # Range for gravity variations
            'lighting': (0.5, 2.0)   # Range for lighting conditions
        }

    def randomize_environment(self):
        """Apply randomization to simulation environment."""
        for param, (min_val, max_val) in self.randomization_params.items():
            random_val = random.uniform(min_val, max_val)
            self.apply_parameter(param, random_val)

    def apply_parameter(self, param, value):
        """Apply parameter to simulation."""
        if param == 'friction':
            # Apply to all surfaces
            pass
        elif param == 'mass':
            # Apply to robot links
            pass
        # ... other parameters
```

### System Identification
```python
import numpy as np
from scipy.optimize import minimize

class SystemId:
    def __init__(self):
        self.sim_params = {}
        self.real_params = {}

    def identify_parameters(self, sim_data, real_data):
        """Identify parameters that minimize sim-to-real gap."""
        def objective_function(params):
            # Simulate with given parameters
            sim_output = self.simulate_with_params(params)

            # Compare with real data
            error = np.mean((sim_output - real_data) ** 2)
            return error

        # Optimize parameters
        result = minimize(objective_function,
                         x0=self.initial_params,
                         method='BFGS')
        return result.x
```

## Advanced Simulation Techniques

### Multi-Robot Simulation
```xml
<!-- Launch file for multiple robots -->
<launch>
  <group ns="robot1">
    <node pkg="gazebo_ros" exec="spawn_entity.py"
          args="-entity robot1 -file $(find-pkg-share my_robot_description)/urdf/robot.urdf -x 0 -y 0 -z 0"/>
  </group>

  <group ns="robot2">
    <node pkg="gazebo_ros" exec="spawn_entity.py"
          args="-entity robot2 -file $(find-pkg-share my_robot_description)/urdf/robot.urdf -x 2 -y 0 -z 0"/>
  </group>
</launch>
```

### Dynamic Environment Simulation
```python
class DynamicEnvironment:
    def __init__(self):
        # Create moving obstacles
        self.moving_obstacles = []
        self.create_moving_obstacles()

    def create_moving_obstacles(self):
        """Create obstacles with scripted movements."""
        # Use Gazebo plugins for dynamic movement
        pass

    def update_environment(self):
        """Update dynamic elements."""
        # Move obstacles according to schedule
        # Change lighting conditions
        # Add/remove objects
        pass
```

## Simulation Validation

### Validation Checklist
- [ ] Robot kinematics match real robot
- [ ] Dynamics parameters are realistic
- [ ] Sensor models include appropriate noise
- [ ] Environment physics are realistic
- [ ] Timing matches real-world expectations
- [ ] Control loops behave similarly

### Performance Benchmarks
- **Real-time factor**: Should be close to 1.0 for real-time simulation
- **Frame rate**: Should maintain 30+ FPS for smooth visualization
- **Control loop timing**: Should match real robot timing
- **Sensor update rates**: Should match real sensor rates

## Best Practices Summary

### Before Simulation
1. **Validate URDF/XACRO**: Check for errors and inconsistencies
2. **Set realistic parameters**: Inertial properties, friction, etc.
3. **Plan the test scenario**: Define clear objectives

### During Simulation
1. **Monitor performance**: Keep track of RTF and frame rates
2. **Log important data**: Save for analysis and debugging
3. **Validate behavior**: Ensure robot acts as expected

### After Simulation
1. **Analyze results**: Compare with expected outcomes
2. **Document issues**: Note any unexpected behaviors
3. **Iterate**: Improve models based on findings

## Troubleshooting Quick Reference

### Gazebo Won't Start
```bash
# Check for running instances
ps aux | grep gazebo

# Kill if needed
killall gzserver gzclient

# Check graphics drivers
nvidia-smi  # For NVIDIA
```

### Robot Falls Through Floor
- Check collision models
- Verify inertial properties
- Check physics parameters

### Sensors Not Publishing
- Check topic names and connections
- Verify sensor plugins are loaded
- Check update rates and noise parameters

## Next Steps

Continue to the next appendix to learn about troubleshooting common robotics issues.