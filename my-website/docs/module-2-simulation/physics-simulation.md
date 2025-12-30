 ---
sidebar_position: 2
title: Physics Simulation in Gazebo
---

# Physics Simulation in Gazebo

## Understanding Gazebo Physics

Gazebo provides a realistic physics simulation environment that is crucial for humanoid robotics development. The physics engine accurately models:

- **Gravity and weight**: Essential for humanoid balance and locomotion
- **Collision detection**: Accurate contact between robot and environment
- **Friction**: Realistic surface interactions for walking and manipulation
- **Inertial properties**: Proper mass distribution and dynamics

## Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics:

### 1. ODE (Open Dynamics Engine)
- **Pros**: Fast, stable, good for real-time simulation
- **Cons**: Less accurate for complex contacts
- **Best for**: Basic humanoid locomotion, real-time control

### 2. Bullet
- **Pros**: Good balance of speed and accuracy
- **Cons**: Can be less stable with complex constraints
- **Best for**: General-purpose robotics simulation

### 3. DART (Dynamic Animation and Robotics Toolkit)
- **Pros**: Highly accurate, good for complex contacts
- **Cons**: Slower than ODE
- **Best for**: High-fidelity simulation and validation

## Configuring Physics Properties

### World Physics Configuration

```xml
<sdf version='1.7'>
  <world name='humanoid_world'>
    <!-- Physics engine configuration -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>

      <!-- Gravity -->
      <gravity>0 0 -9.8</gravity>

      <!-- ODE-specific parameters -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Rest of the world definition -->
  </world>
</sdf>
```

## Humanoid-Specific Physics Considerations

### 1. Center of Mass Management

For humanoid robots, the center of mass is critical for stable locomotion:

```xml
<!-- Example: Torso link with proper inertial properties -->
<link name="torso">
  <inertial>
    <mass value="5.0"/>  <!-- Mass in kg -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Offset from link origin -->
    <inertia
      ixx="0.1" ixy="0.0" ixz="0.0"
      iyy="0.1" iyz="0.0"
      izz="0.05"/>  <!-- Lower moment of inertia around Z (vertical axis) -->
  </inertial>

  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.1"/>
    </geometry>
  </visual>

  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.1"/>
    </geometry>
  </collision>
</link>
```

### 2. Joint Damping and Friction

Proper joint parameters are essential for realistic humanoid movement:

```xml
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.1" upper="2.3" effort="200" velocity="5.0"/>
  <dynamics damping="1.0" friction="0.5"/>
</joint>
```

### 3. Contact Properties for Feet

For stable walking, foot contact properties are crucial:

```xml
<link name="left_foot">
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.002" ixy="0.0" ixz="0.0"
             iyy="0.003" iyz="0.0"
             izz="0.001"/>
  </inertial>

  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
</link>

<!-- Gazebo-specific contact properties -->
<gazebo reference="left_foot">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Coefficient of friction -->
          <mu2>0.8</mu2>
          <fdir1>0 0 1</fdir1>  <!-- Friction direction -->
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>  <!-- Minimal bounce -->
        <threshold>100000</threshold>
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0.001</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>10000000</kp>  <!-- Contact stiffness -->
          <kd>100</kd>      <!-- Contact damping -->
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Creating Stable Humanoid Models

### 1. Mass Distribution

For stable humanoid simulation:

- Keep the center of mass low (in the torso/hip area)
- Distribute mass realistically across limbs
- Ensure sufficient mass in the torso for stability

### 2. Joint Limits and Safety

```xml
<!-- Hip joints with realistic human-like limits -->
<joint name="left_hip_yaw" type="revolute">
  <limit lower="-0.5" upper="0.5" effort="300" velocity="3.0"/>
  <dynamics damping="2.0" friction="1.0"/>
</joint>

<joint name="left_hip_roll" type="revolute">
  <limit lower="-0.4" upper="0.8" effort="300" velocity="3.0"/>
  <dynamics damping="2.0" friction="1.0"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <limit lower="-1.0" upper="2.0" effort="300" velocity="3.0"/>
  <dynamics damping="2.0" friction="1.0"/>
</joint>
```

## Physics Debugging Tips

### 1. Checking for Physics Issues

```bash
# Run Gazebo with verbose physics output
gzserver --verbose your_world.world

# Check for joint limit violations
# Look for messages about "joint limit exceeded"
```

### 2. Tuning Physics Parameters

Start with conservative values and gradually adjust:

- **Increase damping** if the robot oscillates or vibrates
- **Increase contact stiffness (kp)** if parts pass through each other
- **Decrease step size** for more accurate simulation (slower)
- **Adjust ERP (Error Reduction Parameter)** for constraint stability

## Performance Optimization

### 1. Physics Update Rate

Balance accuracy with performance:
- Higher rates: More accurate but slower
- Lower rates: Faster but less stable

```xml
<physics type='ode'>
  <max_step_size>0.001</max_step_size>  <!-- 1ms steps = 1000 Hz -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
  <real_time_update_rate>1000.0</real_time_update_rate>
</physics>
```

### 2. Simplified Collision Models

For performance, use simplified collision geometries:

```xml
<!-- Use simpler shapes for collision detection -->
<collision>
  <geometry>
    <cylinder length="0.3" radius="0.05"/>  <!-- Instead of complex mesh -->
  </geometry>
</collision>
```

## Gazebo Plugins for Physics

### 1. Joint State Publisher Plugin

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/humanoid</namespace>
    </ros>
    <update_rate>50</update_rate>
    <joint_name>left_hip_joint</joint_name>
    <joint_name>left_knee_joint</joint_name>
    <joint_name>left_ankle_joint</joint_name>
    <!-- Add all joint names -->
  </plugin>
</gazebo>
```

### 2. Joint Trajectory Controller Plugin

```xml
<gazebo>
  <plugin name="tricycle_drive_controller" filename="libgazebo_ros_tricycle_drive.so">
    <ros>
      <namespace>/humanoid</namespace>
    </ros>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <joint_state_topic>joint_states</joint_state_topic>
  </plugin>
</gazebo>
```

## Validation Techniques

### 1. Static Balance Test

Create a simple test to verify the robot can stand:

```bash
# Launch your robot model
ros2 launch your_package your_robot.launch.py

# Check if the robot maintains position without control inputs
# Use RViz to visualize joint states
```

### 2. Simple Movement Test

Test basic joint movement:

```bash
# Send simple joint commands
ros2 topic pub /joint_commands trajectory_msgs/msg/JointTrajectory "..."
```

## Common Physics Issues and Solutions

### 1. Robot Falls Through Ground
- **Cause**: Insufficient contact parameters or wrong collision geometry
- **Solution**: Increase contact stiffness, verify collision models

### 2. Joint Oscillation
- **Cause**: Insufficient damping or high controller gains
- **Solution**: Increase joint damping, tune controller parameters

### 3. Unstable Walking
- **Cause**: Poor mass distribution or contact parameters
- **Solution**: Adjust CoM, tune foot contact properties

## Next Steps

Continue to the next section to learn about Unity for human-robot interaction.