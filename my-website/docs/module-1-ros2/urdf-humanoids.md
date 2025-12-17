---
sidebar_position: 4
title: URDF for Humanoid Robots
---

# URDF for Humanoid Robots

## Understanding URDF

**URDF (Unified Robot Description Format)** is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines:

- Physical structure (links and joints)
- Kinematic properties
- Visual and collision geometry
- Inertial properties
- Sensor placements

## URDF Structure for Humanoids

### Basic Humanoid Skeleton

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base/Fixed link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0.2 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Additional links for complete humanoid would continue... -->
</robot>
```

## Key Components for Humanoid Robots

### 1. Links

Links represent rigid bodies in the robot. For humanoids, typical links include:
- `base_link` (or `pelvis` for bipedal robots)
- `torso`/`chest`
- `head`
- `left_upper_arm`, `left_lower_arm`, `left_hand`
- `right_upper_arm`, `right_lower_arm`, `right_hand`
- `left_upper_leg`, `left_lower_leg`, `left_foot`
- `right_upper_leg`, `right_lower_leg`, `right_foot`

### 2. Joints

Joints define the connections between links. For humanoids, joint types include:

**Revolute Joints** (rotational):
```xml
<joint name="hip_joint" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="200" velocity="3.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

**Fixed Joints** (no movement):
```xml
<joint name="sensor_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

### 3. Visual and Collision Elements

```xml
<link name="upper_arm">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.5"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

## Special Considerations for Humanoids

### 1. Bipedal Locomotion Requirements

Humanoid robots need special attention to:
- **Center of Mass**: Critical for balance
- **Foot Design**: For stable walking
- **Joint Limits**: Realistic human-like ranges

```xml
<link name="left_foot">
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
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
  </inertial>
</link>
```

### 2. Sensor Integration

Integrate sensors directly into the URDF:

```xml
<!-- Camera on head -->
<link name="camera_rgb_frame"/>
<link name="camera_rgb_optical_frame"/>
<joint name="camera_rgb_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_rgb_frame"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>
<joint name="camera_rgb_optical_joint" type="fixed">
  <parent link="camera_rgb_frame"/>
  <child link="camera_rgb_optical_frame"/>
  <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
</joint>
```

### 3. Transmission Elements

Define how joints are controlled:

```xml
<transmission name="left_hip_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Advanced URDF Features

### 1. Gazebo-Specific Elements

For simulation in Gazebo:

```xml
<gazebo reference="left_foot">
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
</gazebo>
```

### 2. Materials and Colors

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

## URDF Best Practices for Humanoids

### 1. Maintain Consistent Units
- Use meters for length
- Use kilograms for mass
- Use radians for angles

### 2. Realistic Inertial Properties
```xml
<!-- Example: Approximate inertial calculation for a cylinder -->
<!-- mass = density * volume -->
<!-- For human limbs, use approximate human anatomy data -->
```

### 3. Proper Joint Naming Conventions
- Use descriptive names: `left_hip_yaw_joint`
- Follow consistent patterns: `side_component_jointtype`

### 4. Kinematic Chain Integrity
- Ensure all links are connected through joints
- Avoid floating links (links not connected to the base)

## Validation and Testing

### 1. URDF Validation
```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Parse and display information
urdf_to_graphiz /path/to/robot.urdf
```

### 2. Visualization in RViz
```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
```

## Xacro for Complex Humanoids

For complex humanoid models, use Xacro (XML Macros):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="0.5" />
  <xacro:property name="arm_length" value="0.4" />

  <!-- Macro for symmetric limbs -->
  <xacro:macro name="arm" params="side parent *origin">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_upper_arm"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" parent="torso">
    <origin xyz="0.1 0.2 0.2" rpy="0 0 0"/>
  </xacro:arm>

  <xacro:arm side="right" parent="torso">
    <origin xyz="0.1 -0.2 0.2" rpy="0 0 0"/>
  </xacro:arm>

</robot>
```

## Next Steps

Continue to the next section to learn about creating ROS 2 examples and exercises.