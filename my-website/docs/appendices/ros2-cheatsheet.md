---
sidebar_position: 1
title: ROS 2 Cheatsheet
---

# ROS 2 Cheatsheet

## Overview

This cheatsheet provides quick reference commands and concepts for ROS 2 development in humanoid robotics applications.

## Common Commands

### System Commands
```bash
# Check ROS 2 installation
ros2 --version

# List available commands
ros2 --help

# Source ROS 2 environment (if not in ~/.bashrc)
source /opt/ros/humble/setup.bash
```

### Package Management
```bash
# Create a new package
ros2 pkg create --build-type ament_cmake <package_name>

# Create package with dependencies
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs <package_name>

# List all packages
ros2 pkg list

# Find a specific package
ros2 pkg executables <package_name>
```

### Node Management
```bash
# List active nodes
ros2 node list

# Get info about a specific node
ros2 node info <node_name>

# Run a node
ros2 run <package_name> <executable_name>

# Launch multiple nodes
ros2 launch <package_name> <launch_file>.py
```

### Topic Communication
```bash
# List active topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo <topic_name> <msg_type>

# Publish a message to a topic
ros2 topic pub <topic_name> <msg_type> <args>

# Get info about a topic
ros2 topic info <topic_name>

# Show topic statistics
ros2 topic hz <topic_name>
```

### Service Communication
```bash
# List active services
ros2 service list

# Call a service
ros2 service call <service_name> <srv_type> <args>

# Get info about a service
ros2 service info <service_name>
```

### Action Communication
```bash
# List active actions
ros2 action list

# Send a goal to an action
ros2 action send_goal <action_name> <action_type> <goal_args>

# Get info about an action
ros2 action info <action_name>
```

### Parameter Management
```bash
# List parameters of a node
ros2 param list <node_name>

# Get parameter value
ros2 param get <node_name> <param_name>

# Set parameter value
ros2 param set <node_name> <param_name> <value>

# Load parameters from file
ros2 param load <node_name> <param_file.yaml>
```

## Workspace Management

### Create and Build Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Build with specific packages
colcon build --packages-select <package_name>

# Build and install
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Development Workflow
```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Build with verbose output
colcon build --event-handlers console_direct+

# Run tests
colcon test
colcon test-result --all
```

## Message Types

### Common Message Types
```bash
# Standard messages
std_msgs/msg/String
std_msgs/msg/Int32
std_msgs/msg/Float64

# Geometry messages
geometry_msgs/msg/Twist
geometry_msgs/msg/Pose
geometry_msgs/msg/Point
geometry_msgs/msg/Quaternion

# Sensor messages
sensor_msgs/msg/Image
sensor_msgs/msg/JointState
sensor_msgs/msg/LaserScan
sensor_msgs/msg/PointCloud2

# Navigation messages
nav_msgs/msg/OccupancyGrid
nav_msgs/msg/Path
geometry_msgs/msg/PoseStamped
```

### Example Usage
```bash
# Publish a velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'

# Publish a joint state
ros2 topic pub /joint_states sensor_msgs/msg/JointState '{name: ["joint1", "joint2"], position: [1.0, 2.0]}'
```

## Launch Files

### Python Launch Files
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_node',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

### Launch Commands
```bash
# Launch a file
ros2 launch <package_name> <launch_file>.py

# Launch with arguments
ros2 launch <package_name> <launch_file>.py arg_name:=value

# Launch with simulation time
ros2 launch <package_name> <launch_file>.py use_sim_time:=true
```

## Common ROS 2 Concepts

### Quality of Service (QoS)
```cpp
// In C++
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Publisher with specific QoS
rclcpp::QoS qos(10);  // 10 message queue
qos.reliable();       // Reliable delivery
qos.durability_volatile();  // Volatile durability

auto publisher = node->create_publisher<Image>("topic", qos);
```

### Timers
```python
# In Python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.timer = self.create_timer(0.5, self.timer_callback)  # Every 0.5 seconds
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback {self.i}')
        self.i += 1
```

## Debugging Tools

### Common Debugging Commands
```bash
# Monitor all topics
rqt_plot

# Graph of nodes and topics
rqt_graph

# Message inspector
rqt_topic

# Service caller
rqt_service_caller

# Parameter editor
rqt_reconfigure
```

### Logging
```python
# In Python
self.get_logger().debug("Debug message")
self.get_logger().info("Info message")
self.get_logger().warn("Warning message")
self.get_logger().error("Error message")
self.get_logger().fatal("Fatal message")
```

```cpp
// In C++
RCLCPP_DEBUG(this->get_logger(), "Debug message");
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error message");
RCLCPP_FATAL(this->get_logger(), "Fatal message");
```

## Navigation Stack Commands

### Navigation 2
```bash
# Launch navigation
ros2 launch nav2_bringup navigation_launch.py

# Send goal pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: map}}}'
```

## MoveIt! Commands

### MoveIt! Setup
```bash
# Launch MoveIt!
ros2 launch <robot_moveit_config> move_group.launch.py

# Launch RViz with MoveIt! plugin
ros2 launch <robot_moveit_config> moveit_rviz.launch.py
```

## Simulation Commands

### Gazebo
```bash
# Launch Gazebo
ros2 launch gazebo_ros empty_world.launch.py

# Launch with world file
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.world

# Spawn robot model
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/model.urdf
```

## Lifecycle Nodes
```bash
# List lifecycle nodes
ros2 lifecycle list <node_name>

# Change node state
ros2 lifecycle set <node_name> configure
ros2 lifecycle set <node_name> activate
ros2 lifecycle set <node_name> deactivate
ros2 lifecycle set <node_name> cleanup
```

## Performance Monitoring
```bash
# Monitor node performance
ros2 run quality_of_service_demo_py publisher_latency

# Check network usage
ros2 topic bw /topic_name

# Monitor CPU/memory of ROS processes
htop
```

## Common Error Solutions

### "No executable found"
```bash
# Solution: Make sure workspace is sourced
source ~/ros2_ws/install/setup.bash

# Solution: Rebuild workspace
cd ~/ros2_ws && colcon build --symlink-install
```

### "Topic not found"
```bash
# Check if node is running
ros2 node list

# Check if topic exists
ros2 topic list

# Verify topic names match exactly (case-sensitive)
```

### "Permission denied" for serial devices
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
```

## Environment Variables
```bash
# Set ROS domain ID (for network isolation)
export ROS_DOMAIN_ID=1

# Use simulation time
export USE_SIM_TIME=true

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

## ROS 2 vs ROS 1 Differences

| ROS 1 | ROS 2 |
|-------|-------|
| roscore | Automatic discovery |
| rosmaster | DDS-based discovery |
| roscpp | rclcpp |
| rospy | rclpy |
| catkin_make | colcon build |
| roslaunch | ros2 launch |

## Next Steps

Continue to the next appendix to learn about simulation tips and best practices.