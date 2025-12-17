---
sidebar_position: 5
title: Gazebo Setup and Configuration Guide
---

# Gazebo Setup and Configuration Guide

## Installing Gazebo for Humanoid Robotics

### System Requirements

Before installing Gazebo, ensure your system meets the requirements:

- **OS**: Ubuntu 22.04 LTS (recommended) or similar Linux distribution
- **GPU**: Dedicated GPU with OpenGL 3.3+ support (NVIDIA recommended)
- **RAM**: 8GB+ (16GB+ recommended for complex humanoid models)
- **CPU**: Multi-core processor (4+ cores recommended)

### Installation Options

#### 1. Install Gazebo Harmonic (Latest)

```bash
# Add the osrfoundation repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL http://get.gazebosim.org | sh

# Install Gazebo Harmonic
sudo apt install gz-harmonic
```

#### 2. Install Gazebo with ROS 2 Iron Integration

```bash
# Install ROS 2 Iron first if not already installed
sudo apt update
sudo apt install ros-iron-desktop

# Install Gazebo ROS packages
sudo apt install ros-iron-gazebo-*
sudo apt install ros-iron-gazebo-ros-pkgs
sudo apt install ros-iron-gazebo-dev
```

### Verification Installation

```bash
# Test basic Gazebo functionality
gz sim

# Test with a simple world
gz sim -r -v 1 empty.sdf

# Check available worlds
ls /usr/share/gazebo/worlds/
```

## Basic Gazebo Configuration

### 1. Environment Variables

Add these to your `~/.bashrc` or `~/.zshrc`:

```bash
# Gazebo configuration
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/physical_ai_ws/src/humanoid_description/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/physical_ai_ws/src/humanoid_description/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/physical_ai_ws/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/physical_ai_ws/install/lib

# Increase Gazebo timeout for large models
export GAZEBO_IPA_TOLERANCE=0.001
```

### 2. Basic World File Structure

Create a basic world file (`humanoid_world.world`):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include a basic environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Your humanoid robot would be included here -->
    <!-- <include>
      <uri>model://your_humanoid_robot</uri>
    </include> -->

    <!-- Optional: Add objects for interaction -->
    <include>
      <uri>model://table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://cylinder</uri>
      <pose>-1 1 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Humanoid Robot Model Integration

### 1. Creating a Robot Model Directory

```bash
# Create the model directory structure
mkdir -p ~/physical_ai_ws/src/humanoid_description/models/humanoid_robot
mkdir -p ~/physical_ai_ws/src/humanoid_description/models/humanoid_robot/meshes
mkdir -p ~/physical_ai_ws/src/humanoid_description/models/humanoid_robot/materials/textures
mkdir -p ~/physical_ai_ws/src/humanoid_description/models/humanoid_robot/materials/scripts
```

### 2. Robot Model Configuration File (`model.config`)

```xml
<?xml version="1.0"?>
<model>
  <name>humanoid_robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>

  <description>
    A basic humanoid robot model for simulation and control testing.
  </description>
</model>
```

### 3. Complete Robot Model File (`model.sdf`)

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="humanoid_robot">
    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 0.8 0 0 0</pose>
      <inertial>
        <mass>0.0001</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Torso -->
    <link name="torso">
      <inertial>
        <mass>5.0</mass>
        <origin xyz="0 0 0.15"/>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.05</izz>
        </inertia>
      </inertial>

      <visual name="torso_visual">
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <collision name="torso_collision">
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="base_to_torso" type="fixed">
      <parent>base_link</parent>
      <child>torso</child>
    </joint>

    <!-- Head -->
    <link name="head">
      <inertial>
        <mass>2.0</mass>
        <origin xyz="0 0 0"/>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <visual name="head_visual">
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0.8 0.6 1</ambient>
          <diffuse>1 0.8 0.6 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <collision name="head_collision">
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Left Arm (example) -->
    <link name="left_upper_arm">
      <inertial>
        <mass>1.0</mass>
        <origin xyz="0 0 0.15"/>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <visual name="left_upper_arm_visual">
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <collision name="left_upper_arm_collision">
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="left_shoulder_joint" type="revolute">
      <parent>torso</parent>
      <child>left_upper_arm</child>
      <origin xyz="0.05 0.15 0.2" rpy="0 0 0"/>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>50</effort>
          <velocity>2.0</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Add more links and joints for complete humanoid model -->
  </model>
</sdf>
```

## ROS 2 Integration Configuration

### 1. Robot State Publisher Launch

Create `launch/humanoid_gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get Gazebo launch file
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch'
    )

    # Get robot description
    robot_description_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )

    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
            )
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            remappings=[
                ('/joint_states', 'joint_states')
            ]
        ),

        # Joint State Publisher (for testing)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'humanoid_robot',
                '-x', '0', '-y', '0', '-z', '0.8'
            ],
            output='screen'
        )
    ])
```

### 2. Controller Configuration

Create `config/humanoid_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    left_arm_controller:
      type: position_controllers/JointGroupPositionController

    right_arm_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_yaw
      - left_hip_roll
      - left_hip_pitch
      - left_knee
      - left_ankle_pitch
      - left_ankle_roll

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_yaw
      - right_hip_roll
      - right_hip_pitch
      - right_knee
      - right_ankle_pitch
      - right_ankle_roll

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_pitch
      - left_shoulder_roll
      - left_shoulder_yaw
      - left_elbow
      - left_wrist_yaw
      - left_wrist_pitch

right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_pitch
      - right_shoulder_roll
      - right_shoulder_yaw
      - right_elbow
      - right_wrist_yaw
      - right_wrist_pitch
```

## Advanced Configuration Options

### 1. Performance Optimization

Create a performance-optimized world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world_fast">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Optimized physics for faster simulation -->
    <physics type="ode">
      <max_step_size>0.002</max_step_size>  <!-- Larger step = faster but less accurate -->
      <real_time_factor>0.5</real_time_factor>  <!-- Allow simulation to run slower than real-time -->
      <real_time_update_rate>500.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>

      <!-- ODE-specific optimizations -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>20</iters>  <!-- Reduce iterations for speed -->
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

    <!-- Your robot model -->
  </world>
</sdf>
```

### 2. High-Fidelity Configuration

For accurate simulation, use this configuration:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world_accurate">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- High-fidelity physics -->
    <physics type="ode">
      <max_step_size>0.0005</max_step_size>  <!-- Smaller step = more accurate -->
      <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
      <real_time_update_rate>2000.0</real_time_update_rate>
      <gravity>0 0 -9.80665</gravity>  <!-- More precise gravity -->

      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>  <!-- More iterations = more accurate -->
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>1e-5</cfm>  <!-- Constraint Force Mixing -->
          <erp>0.1</erp>   <!-- Error Reduction Parameter -->
          <contact_max_correcting_vel>10.0</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Your robot model -->
  </world>
</sdf>
```

## Running Simulations

### 1. Basic Simulation

```bash
# Launch Gazebo with your world
gz sim -r -v 1 /path/to/your/world.sdf

# Or using ROS 2 launch
ros2 launch your_package humanoid_gazebo.launch.py
```

### 2. Simulation with GUI

```bash
# Launch with GUI
gz sim -g /path/to/your/world.sdf

# Or using ROS 2
ros2 launch your_package humanoid_gazebo.launch.py
```

### 3. Command Line Options

```bash
# Run simulation headless (no GUI)
gz sim -s -r -v 1 your_world.sdf

# Run with specific physics engine
gz sim --physics-engine gz-physics-dart-plugin -r your_world.sdf

# Run with verbose output
gz sim -v 4 -r your_world.sdf
```

## Troubleshooting Common Issues

### 1. Robot Falls Through Ground

**Problem**: Robot falls through the ground plane.

**Solutions**:
- Check that the ground plane model is included in your world file
- Verify that collision geometries are properly defined
- Increase contact stiffness in physics parameters

```xml
<gazebo reference="your_link">
  <collision>
    <surface>
      <contact>
        <ode>
          <kp>10000000</kp>  <!-- Increase stiffness -->
          <kd>100</kd>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

### 2. Joint Oscillation

**Problem**: Joints oscillate or vibrate.

**Solutions**:
- Increase joint damping
- Use more stable physics parameters
- Check mass distribution in inertial properties

### 3. Performance Issues

**Problem**: Simulation runs slowly.

**Solutions**:
- Reduce physics update rate
- Simplify collision geometries
- Use fewer sensors or lower sensor update rates
- Consider using simpler physics engine

### 4. Sensor Data Issues

**Problem**: Sensor data is noisy or unrealistic.

**Solutions**:
- Add appropriate noise models to sensor definitions
- Verify sensor mounting positions
- Check coordinate frame transformations

## Advanced Topics

### 1. Multi-Robot Simulation

To simulate multiple humanoid robots:

```xml
<!-- First robot -->
<include>
  <uri>model://humanoid_robot</uri>
  <name>robot1</name>
  <pose>0 0 0.8 0 0 0</pose>
</include>

<!-- Second robot -->
<include>
  <uri>model://humanoid_robot</uri>
  <name>robot2</name>
  <pose>2 0 0.8 0 0 0</pose>
</include>
```

### 2. Custom Plugins

Create a custom plugin for specialized robot control:

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class HumanoidController : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HumanoidController::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Custom control logic here
      // Apply forces, read sensors, etc.
    }

    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HumanoidController)
}
```

## Best Practices

1. **Start Simple**: Begin with basic models and gradually add complexity
2. **Validate Physics**: Test individual joints and links before full robot
3. **Monitor Performance**: Keep an eye on real-time factor during simulation
4. **Use Appropriate Fidelity**: Match simulation accuracy to your needs
5. **Document Configurations**: Keep track of working configurations for different scenarios

## Next Steps

Continue to the next section to learn about simulation examples and exercises.