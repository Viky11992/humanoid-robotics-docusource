---
sidebar_position: 3
title: Troubleshooting Guide
---

# Troubleshooting Guide

## Overview

This comprehensive troubleshooting guide addresses common issues encountered in humanoid robotics development with ROS 2, simulation, and hardware integration. Use this guide systematically to diagnose and resolve problems.

## ROS 2 Troubleshooting

### Common Installation Issues

#### "Command not found" Errors
**Problem**: `ros2` command not recognized
**Solutions**:
1. Check if ROS 2 is installed:
   ```bash
   ls /opt/ros/
   ```
2. Source the environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Add to bashrc permanently:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

#### Package Build Failures
**Problem**: `colcon build` fails with compilation errors
**Solutions**:
1. Clean previous build:
   ```bash
   rm -rf build/ install/ log/
   ```
2. Update dependencies:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build with verbose output:
   ```bash
   colcon build --event-handlers console_direct+ --packages-select <package_name>
   ```

#### Node Communication Issues
**Problem**: Nodes can't communicate or topics aren't connecting
**Solutions**:
1. Check network configuration:
   ```bash
   echo $ROS_DOMAIN_ID
   echo $ROS_LOCALHOST_ONLY
   ```
2. Verify nodes are running:
   ```bash
   ros2 node list
   ros2 topic list
   ```
3. Check for firewall issues if using multiple machines

### Permission Issues

#### Serial Port Access
**Problem**: Cannot access serial devices for robot communication
**Solutions**:
1. Add user to dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
2. Log out and back in for changes to take effect
3. Verify permissions:
   ```bash
   ls -l /dev/ttyUSB*
   ls -l /dev/ttyACM*
   ```

#### Gazebo Permission Issues
**Problem**: Gazebo fails to start or access resources
**Solutions**:
1. Check user permissions for graphics:
   ```bash
   groups $USER
   ```
2. Ensure user is in video group:
   ```bash
   sudo usermod -a -G video $USER
   ```

## Simulation Troubleshooting

### Gazebo Issues

#### Gazebo Won't Start
**Problem**: Gazebo fails to launch or crashes immediately
**Solutions**:
1. Check for existing instances:
   ```bash
   killall gzserver gzclient
   ```
2. Check graphics drivers:
   ```bash
   nvidia-smi  # For NVIDIA
   glxinfo | grep "OpenGL renderer"  # For OpenGL support
   ```
3. Launch with software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gazebo
   ```

#### Robot Falls Through Floor
**Problem**: Robot model falls through the ground plane
**Solutions**:
1. Check collision models in URDF:
   ```xml
   <collision>
     <geometry>
       <box size="1 1 1"/>  <!-- Ensure collision geometry exists -->
     </geometry>
   </collision>
   ```
2. Verify inertial properties:
   ```xml
   <inertial>
     <mass value="1.0"/>
     <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
   </inertial>
   ```
3. Check physics parameters in world file

#### Slow Simulation Performance
**Problem**: Low real-time factor (< 0.5)
**Solutions**:
1. Optimize physics settings:
   ```xml
   <physics type="ode">
     <max_step_size>0.01</max_step_size>  <!-- Increase from 0.001 -->
     <real_time_update_rate>100</real_time_update_rate>  <!-- Decrease if too high -->
   </physics>
   ```
2. Simplify collision models
3. Reduce visual complexity of models
4. Close unnecessary applications to free resources

### Isaac Sim Issues

#### USD Loading Problems
**Problem**: Models or scenes fail to load in Isaac Sim
**Solutions**:
1. Check file paths are correct
2. Verify USD files are valid:
   ```bash
   usdview /path/to/file.usd
   ```
3. Check for missing assets in Nucleus server

#### Rendering Issues
**Problem**: Black screen or poor rendering quality
**Solutions**:
1. Verify NVIDIA GPU drivers are up to date
2. Check VRAM usage:
   ```bash
   nvidia-smi
   ```
3. Adjust rendering settings in Isaac Sim preferences

## Hardware Troubleshooting

### Robot Communication

#### Connection Failures
**Problem**: Cannot connect to robot hardware
**Solutions**:
1. Check physical connections:
   ```bash
   lsusb  # For USB connections
   ifconfig  # For network connections
   ```
2. Verify correct IP address or port
3. Check robot power status
4. Test basic connectivity:
   ```bash
   ping <robot_ip>
   ```

#### Motor Control Issues
**Problem**: Motors don't respond or move erratically
**Solutions**:
1. Check motor controller status
2. Verify power supply voltage
3. Check joint limits and safety limits
4. Review control loop frequency

### Sensor Troubleshooting

#### Camera Issues
**Problem**: Camera not publishing images or poor image quality
**Solutions**:
1. Check camera connection:
   ```bash
   ls /dev/video*
   v4l2-ctl --list-devices
   ```
2. Test camera directly:
   ```bash
   cheese  # Simple camera test
   ```
3. Check camera calibration
4. Verify camera driver and configuration

#### IMU Problems
**Problem**: IMU data is noisy or incorrect
**Solutions**:
1. Check for magnetic interference
2. Verify IMU calibration
3. Check update rate settings
4. Implement sensor fusion for better accuracy

## Navigation and Control Issues

### MoveIt! Problems

#### Planning Failures
**Problem**: MoveIt! cannot find valid paths
**Solutions**:
1. Check collision objects in environment
2. Verify robot state publisher:
   ```bash
   ros2 topic echo /joint_states
   ```
3. Check planning scene:
   ```bash
   ros2 run rviz2 rviz2  # Visualize planning scene
   ```
4. Adjust planning parameters

#### Execution Issues
**Problem**: Planned trajectories don't execute properly
**Solutions**:
1. Check controller status:
   ```bash
   ros2 control list_controllers
   ```
2. Verify joint limits
3. Check control loop timing
4. Adjust trajectory tolerances

### Navigation Stack Issues

#### Localization Problems
**Problem**: Robot cannot determine its position
**Solutions**:
1. Check sensor data:
   ```bash
   ros2 topic echo /scan
   ros2 topic echo /camera/depth/image_rect_raw
   ```
2. Verify map quality and features
3. Check initial pose estimation
4. Adjust localization parameters

#### Path Planning Failures
**Problem**: Cannot find path to goal
**Solutions**:
1. Check costmap configuration
2. Verify obstacle detection
3. Check map resolution
4. Adjust inflation parameters

## AI and Perception Issues

### Computer Vision Problems

#### Object Detection Failures
**Problem**: Objects not detected or false detections
**Solutions**:
1. Check camera calibration
2. Verify lighting conditions
3. Adjust detection thresholds
4. Retrain models with domain-specific data

#### Point Cloud Issues
**Problem**: Poor point cloud quality or missing data
**Solutions**:
1. Check sensor configuration
2. Verify sensor calibration
3. Check for sensor occlusions
4. Adjust filtering parameters

### Language Processing Issues

#### Speech Recognition Problems
**Problem**: Poor speech recognition accuracy
**Solutions**:
1. Check microphone quality and placement
2. Reduce background noise
3. Use noise cancellation
4. Adjust recognition thresholds

#### Natural Language Understanding
**Problem**: Misunderstood commands or responses
**Solutions**:
1. Improve command parsing
2. Add context awareness
3. Implement clarification dialogs
4. Train models on domain-specific language

## Performance Optimization

### CPU Usage Issues

#### High CPU Usage
**Problem**: ROS 2 nodes consuming excessive CPU
**Solutions**:
1. Profile node performance:
   ```bash
   ros2 run top top
   ```
2. Optimize loops and callbacks
3. Use appropriate update rates
4. Implement efficient data structures

#### Memory Leaks
**Problem**: Memory usage grows over time
**Solutions**:
1. Use memory profiling tools:
   ```bash
   valgrind --tool=memcheck --leak-check=full your_program
   ```
2. Implement proper cleanup in destructors
3. Use smart pointers in C++
4. Monitor memory usage:
   ```bash
   ros2 run top top
   ```

### Network Performance

#### Communication Delays
**Problem**: High latency in robot communication
**Solutions**:
1. Check network bandwidth:
   ```bash
   iperf3 -s  # Server
   iperf3 -c <server_ip>  # Client
   ```
2. Use appropriate QoS settings
3. Optimize message sizes
4. Check for network congestion

## Debugging Techniques

### ROS 2 Debugging Tools

#### Message Inspection
```bash
# Monitor topic messages
ros2 topic echo /topic_name --field field_name

# Check message frequency
ros2 topic hz /topic_name

# Monitor bandwidth
ros2 topic bw /topic_name
```

#### Node Debugging
```bash
# Get detailed node information
ros2 node info /node_name

# Monitor node parameters
ros2 param list /node_name
```

### Logging and Monitoring
```python
# In Python nodes
import rclpy
from rclpy.qos import QoSProfile

def debug_callback(self, msg):
    self.get_logger().debug(f"Received message: {msg}")
    # Additional debugging code here
```

### Visualization Tools
```bash
# Launch RViz for visualization
ros2 run rviz2 rviz2

# Launch rqt for various debugging tools
rqt
rqt_graph  # Node connections
rqt_plot   # Plot numerical data
```

## Common Error Messages and Solutions

### "No data" or "Timeout" Errors
- **Cause**: Nodes not publishing or subscribing correctly
- **Solution**: Check topic names, message types, and network configuration

### "Transform failed" Errors
- **Cause**: TF tree issues or missing transforms
- **Solution**: Check TF publishers, verify frame names, check transform timing

### "Failed to load controller" Errors
- **Cause**: Controller configuration issues
- **Solution**: Check controller manager status, verify configuration files

### "Could not find a connection" Errors
- **Cause**: Network or discovery issues
- **Solution**: Check ROS_DOMAIN_ID, network configuration, firewall settings

## Systematic Troubleshooting Approach

### Step 1: Identify the Problem
1. Reproduce the issue consistently
2. Note error messages and symptoms
3. Determine if it's a new or intermittent issue

### Step 2: Gather Information
1. Check ROS 2 status:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 service list
   ```
2. Review logs:
   ```bash
   journalctl -u ros-*
   tail -f ~/ros2_ws/log/latest/*.log
   ```
3. Check system resources:
   ```bash
   htop
   df -h
   free -h
   ```

### Step 3: Isolate the Issue
1. Test individual components separately
2. Use minimal test cases
3. Check dependencies and prerequisites

### Step 4: Apply Solutions
1. Start with simplest solutions first
2. Test after each change
3. Document what works and what doesn't

### Step 5: Verify the Fix
1. Test the original problem scenario
2. Run regression tests
3. Monitor for side effects

## Preventive Measures

### Regular Maintenance
1. Update ROS 2 packages regularly
2. Monitor system resources
3. Backup configurations regularly
4. Test in simulation before real hardware

### Best Practices
1. Use version control for all code
2. Document configuration changes
3. Implement comprehensive logging
4. Create automated tests for critical functionality

## When to Seek Help

### Community Resources
- **ROS Answers**: https://answers.ros.org/
- **ROS Discourse**: https://discourse.ros.org/
- **GitHub Issues**: For specific packages
- **Local ROS community**: Meetups, forums

### Professional Support
- When safety is a concern
- When dealing with expensive hardware
- When facing tight deadlines
- When issues affect multiple systems

## Quick Reference Commands

### System Status
```bash
# Check ROS 2 installation
ros2 --version

# Check running nodes
ros2 node list

# Check all topics
ros2 topic list -t -p

# Check system resources
htop
nvidia-smi  # For GPU status
```

### Common Fixes
```bash
# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Clean and rebuild workspace
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# Check ROS network
ros2 topic list  # Should work across all machines in domain
```

## Next Steps

Continue to the next appendix to learn about additional resources and references for humanoid robotics development.