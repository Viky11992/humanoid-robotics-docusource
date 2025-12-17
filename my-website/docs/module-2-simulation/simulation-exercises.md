---
sidebar_position: 6
title: Simulation Exercises for Humanoid Robotics
---

# Simulation Exercises for Humanoid Robotics

## Exercise 1: Basic Gazebo Environment Setup

### Objective
Set up a basic Gazebo environment with a simple humanoid robot model and verify basic functionality.

### Tasks
1. Install Gazebo and ROS 2 integration packages
2. Create a simple humanoid model with at least 6 DOF
3. Create a world file with basic environment
4. Launch the simulation and verify the robot appears correctly

### Solution Template

**Simple Humanoid URDF (`simple_humanoid.urdf`):**
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.05 -0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>
</robot>
```

**Basic World File (`simple_humanoid.world`):**
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_humanoid_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### Commands to Run the Exercise
```bash
# 1. Create model directory and files
mkdir -p ~/.gazebo/models/simple_humanoid
cp simple_humanoid.urdf ~/.gazebo/models/simple_humanoid/model.sdf
cp model.config ~/.gazebo/models/simple_humanoid/

# 2. Launch Gazebo with the world
gz sim -r -v 1 simple_humanoid.world

# 3. Or use ROS 2 to spawn the robot
# First, convert URDF to SDF and spawn
```

## Exercise 2: Joint Control in Simulation

### Objective
Create a ROS 2 node that controls the joints of your simulated humanoid robot.

### Tasks
1. Create a joint trajectory controller
2. Implement a publisher that sends joint commands
3. Verify the robot moves as expected in simulation
4. Add safety limits to prevent dangerous movements

### Solution Template

**Joint Controller Node (`joint_controller.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint trajectories
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer for sending commands
        self.timer = self.create_timer(2.0, self.send_joint_trajectory)

        # Define joint names (these should match your robot model)
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'neck_joint'
        ]

        self.command_counter = 0
        self.get_logger().info('Joint Controller initialized')

    def send_joint_trajectory(self):
        """Send a joint trajectory command."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Create different patterns for each cycle
        if self.command_counter % 4 == 0:
            # Wave pattern
            point.positions = [0.5, 0.2, -0.5, 0.2, 0.0]
        elif self.command_counter % 4 == 1:
            # Return to center
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        elif self.command_counter % 4 == 2:
            # Opposite wave
            point.positions = [-0.5, -0.2, 0.5, -0.2, 0.1]
        else:
            # Neutral position
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]

        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]

        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'Published joint trajectory: {point.positions}')
        self.command_counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Safety Checker Node (`safety_checker.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class SafetyChecker(Node):
    def __init__(self):
        super().__init__('safety_checker')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscribe to trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Define safe limits (in radians)
        self.joint_limits = {
            'left_shoulder_joint': (-1.57, 1.57),
            'left_elbow_joint': (-1.57, 1.57),
            'right_shoulder_joint': (-1.57, 1.57),
            'right_elbow_joint': (-1.57, 1.57),
            'neck_joint': (-0.5, 0.5)
        }

        self.joint_names = list(self.joint_limits.keys())
        self.current_positions = {}

    def joint_state_callback(self, msg):
        """Monitor current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.joint_limits and i < len(msg.position):
                self.current_positions[name] = msg.position[i]

                # Check if current position is within limits
                min_limit, max_limit = self.joint_limits[name]
                pos = msg.position[i]

                if pos < min_limit or pos > max_limit:
                    self.get_logger().warn(
                        f'Joint {name} position {pos} exceeds limits '
                        f'[{min_limit}, {max_limit}]'
                    )

    def trajectory_callback(self, msg):
        """Check trajectory commands for safety."""
        for point in msg.points:
            for i, joint_name in enumerate(msg.joint_names):
                if (joint_name in self.joint_limits and
                    i < len(point.positions)):

                    pos = point.positions[i]
                    min_limit, max_limit = self.joint_limits[joint_name]

                    if pos < min_limit or pos > max_limit:
                        self.get_logger().warn(
                            f'Trajectory command for {joint_name} '
                            f'{pos} exceeds limits [{min_limit}, {max_limit}]'
                        )
                        # In a real system, you might want to modify or block the command

def main(args=None):
    rclpy.init(args=args)
    safety_checker = SafetyChecker()

    try:
        rclpy.spin(safety_checker)
    except KeyboardInterrupt:
        pass
    finally:
        safety_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: Sensor Integration and Perception

### Objective
Add sensors to your humanoid robot model and create nodes to process sensor data.

### Tasks
1. Add a camera sensor to the robot head
2. Create a node that processes camera images
3. Add an IMU sensor for balance
4. Implement a simple perception algorithm

### Solution Template

**Robot Model with Sensors (`humanoid_with_sensors.urdf`):**
```xml
<?xml version="1.0"?>
<robot name="humanoid_with_sensors">
  <!-- Include the basic humanoid model -->
  <!-- ... (previous links and joints) ... -->

  <!-- Camera in the head -->
  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo-specific sensor definitions -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/image_raw:=/camera/image_raw</remapping>
          <remapping>~/camera_info:=/camera/camera_info</remapping>
        </ros>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU in the torso -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><stddev>0.001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.001</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><stddev>0.017</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.017</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.017</stddev></noise></z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/out:=/imu/data</remapping>
        </ros>
        <frame_name>torso</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

**Perception Node (`perception_node.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to camera and IMU data
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        # Store latest data
        self.latest_image = None
        self.latest_imu = None

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_sensors)  # 10 Hz

        self.get_logger().info('Perception Node initialized')

    def image_callback(self, msg):
        """Process incoming image."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image

            # Perform simple processing (edge detection example)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Count edges as a simple "object detection"
            edge_count = np.sum(edges > 0)
            if edge_count > 1000:  # Arbitrary threshold
                self.get_logger().info(f'Potential object detected: {edge_count} edges')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process IMU data for balance."""
        self.latest_imu = msg

        # Extract orientation (simplified)
        orientation = msg.orientation
        # Convert quaternion to roll/pitch (simplified approach)
        # In practice, use proper quaternion-to-Euler conversion

        # Check if robot is tilting too much
        if abs(orientation.z) > 0.1:  # Simplified check
            self.get_logger().warn('Robot may be tilting!')

    def process_sensors(self):
        """Main processing loop."""
        if self.latest_image is not None and self.latest_imu is not None:
            # Combine sensor data for perception
            self.get_logger().info('Processing combined sensor data')

            # Example: If we detect something AND the robot is stable, move forward
            # This is a very simplified example
            self.perform_action()

    def perform_action(self):
        """Perform an action based on sensor data."""
        # In a real implementation, this would send commands to the robot
        self.get_logger().info('Performing action based on sensor data')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: Walking Controller Simulation

### Objective
Implement a simple walking controller for your humanoid robot in simulation.

### Tasks
1. Add feet to your robot model with appropriate collision properties
2. Create a basic walking pattern generator
3. Implement ground contact detection
4. Create a simple balance controller

### Solution Template

**Walking Controller Node (`walking_controller.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState, Imu
import math

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers and subscribers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Walking parameters
        self.step_height = 0.05  # meters
        self.step_length = 0.1   # meters
        self.step_duration = 1.0 # seconds
        self.phase = 0.0
        self.walking = False

        # Joint names for legs
        self.left_leg_joints = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll'
        ]

        self.right_leg_joints = [
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
        ]

        # Current state
        self.current_joint_positions = {}
        self.current_orientation = None

        # Timer for walking control
        self.walk_timer = self.create_timer(0.02, self.walk_callback)  # 50 Hz

        self.get_logger().info('Walking Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Update orientation from IMU."""
        self.current_orientation = msg.orientation

    def start_walking(self):
        """Start the walking motion."""
        self.walking = True
        self.get_logger().info('Walking started')

    def stop_walking(self):
        """Stop the walking motion."""
        self.walking = False
        self.get_logger().info('Walking stopped')

    def walk_callback(self):
        """Main walking control loop."""
        if not self.walking:
            return

        # Update walking phase
        self.phase += 0.02 * 2 * math.pi / self.step_duration  # 2π per step duration

        # Generate walking pattern
        trajectory_msg = self.generate_walking_trajectory()

        if trajectory_msg:
            self.trajectory_pub.publish(trajectory_msg)

    def generate_walking_trajectory(self):
        """Generate walking trajectory based on current phase."""
        msg = JointTrajectory()

        # Set joint names
        msg.joint_names = (self.left_leg_joints + self.right_leg_joints +
                          ['left_shoulder_joint', 'right_shoulder_joint'])

        point = JointTrajectoryPoint()

        # Calculate walking pattern for left leg
        left_positions = self.calculate_leg_positions('left', self.phase)

        # Calculate walking pattern for right leg (opposite phase)
        right_positions = self.calculate_leg_positions('right', self.phase + math.pi)

        # Arm movements for balance
        arm_positions = self.calculate_arm_positions(self.phase)

        # Combine all positions
        all_positions = left_positions + right_positions + arm_positions

        point.positions = all_positions
        point.velocities = [0.0] * len(all_positions)
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        msg.points = [point]
        return msg

    def calculate_leg_positions(self, side, phase):
        """Calculate joint positions for a leg based on walking phase."""
        # Simplified walking pattern - in reality, this would use inverse kinematics
        positions = []

        # Hip joints
        hip_pitch = 0.1 * math.sin(phase)  # Forward/back motion
        hip_roll = 0.05 * math.sin(phase * 2)  # Slight lateral motion for stability
        hip_yaw = 0.0  # No yaw for this simple model

        # Knee joint
        knee = -0.2 * abs(math.sin(phase))  # Bending on stance phase

        # Ankle joints
        ankle_pitch = -0.1 * math.sin(phase)  # Compensate for forward motion
        ankle_roll = 0.0

        positions = [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
        return positions

    def calculate_arm_positions(self, phase):
        """Calculate arm positions for balance."""
        # Arms move opposite to legs for balance
        left_arm = 0.2 * math.sin(phase + math.pi)  # Opposite to left leg
        right_arm = 0.2 * math.sin(phase + math.pi)  # Opposite to right leg

        return [left_arm, right_arm]

def main(args=None):
    rclpy.init(args=args)
    walking_controller = WalkingController()

    # Give some time for connections, then start walking
    def start_walking_after_delay():
        walking_controller.start_walking()

    # Start walking after 2 seconds
    walking_controller.create_timer(2.0, start_walking_after_delay)

    try:
        rclpy.spin(walking_controller)
    except KeyboardInterrupt:
        walking_controller.stop_walking()
    finally:
        walking_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 5: Multi-Sensor Fusion

### Objective
Combine data from multiple sensors (camera, IMU, joint encoders) to improve robot state estimation.

### Tasks
1. Create a sensor fusion node
2. Implement a simple Kalman filter or complementary filter
3. Validate the fused state estimates
4. Use fused data for robot control

### Solution Template

**Sensor Fusion Node (`sensor_fusion.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.estimated_pose_pub = self.create_publisher(
            Vector3,  # Simplified pose as x, y, theta
            '/estimated_pose',
            10
        )

        self.balance_state_pub = self.create_publisher(
            Float64,
            '/balance_state',
            10
        )

        # State estimation variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0  # Heading angle

        # Velocity estimates
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        # Complementary filter parameters
        self.complementary_gain = 0.98  # Trust IMU more for orientation

        # Previous time for velocity calculation
        self.prev_time = None

        # Timer for fusion update
        self.fusion_timer = self.create_timer(0.01, self.fusion_callback)  # 100 Hz

        self.get_logger().info('Sensor Fusion Node initialized')

    def joint_callback(self, msg):
        """Process joint state data for odometry."""
        # For this example, we'll use a simplified approach
        # In reality, you'd use forward kinematics and odometry
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is not None:
            dt = current_time - self.prev_time

            # Simplified: assume forward motion based on joint commands
            # In practice, use inverse kinematics and forward kinematics
            if 'left_knee' in msg.name and 'right_knee' in msg.name:
                left_idx = msg.name.index('left_knee')
                right_idx = msg.name.index('right_knee')

                # Estimate forward velocity based on leg movement
                # This is a very simplified example
                avg_leg_movement = (msg.position[left_idx] + msg.position[right_idx]) / 2
                estimated_velocity = avg_leg_movement * 0.5  # Scale factor

                # Update position based on estimated velocity
                self.position_x += estimated_velocity * dt * math.cos(self.orientation)
                self.position_y += estimated_velocity * dt * math.sin(self.orientation)

        self.prev_time = current_time

    def imu_callback(self, msg):
        """Process IMU data for orientation."""
        # Extract orientation from IMU quaternion
        # Simplified: using z-component as heading
        # In practice, use proper quaternion to Euler conversion
        self.orientation = math.atan2(
            2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
            1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        )

    def fusion_callback(self):
        """Main fusion and estimation loop."""
        # Apply complementary filter for orientation
        # (This is simplified - real implementation would be more complex)

        # Publish estimated pose
        pose_msg = Vector3()
        pose_msg.x = float(self.position_x)
        pose_msg.y = float(self.position_y)
        pose_msg.z = float(self.orientation)

        self.estimated_pose_pub.publish(pose_msg)

        # Calculate balance state (simplified)
        balance_msg = Float64()
        # Simple balance metric: deviation from upright position
        balance_msg.data = abs(self.orientation)

        self.balance_state_pub.publish(balance_msg)

        self.get_logger().info(
            f'Estimated pose: x={self.position_x:.2f}, y={self.position_y:.2f}, '
            f'theta={self.orientation:.2f}, balance={balance_msg.data:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 6: Simulation Validation

### Objective
Validate your simulation by comparing it to expected behaviors and identifying discrepancies.

### Tasks
1. Create test scenarios for different robot behaviors
2. Log and analyze simulation data
3. Compare simulation results to expected outcomes
4. Identify and document any simulation inaccuracies

### Solution Template

**Validation Node (`simulation_validator.py`):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np
import csv
from datetime import datetime

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Data storage
        self.joint_data_log = []
        self.imu_data_log = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Validation parameters
        self.validation_window = 10.0  # seconds to collect data
        self.data_collection_active = True

        # Timer for validation
        self.validation_timer = self.create_timer(0.1, self.validation_callback)

        self.get_logger().info('Simulation Validator initialized')

    def joint_callback(self, msg):
        """Log joint state data."""
        if self.data_collection_active:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            data_point = {
                'timestamp': timestamp,
                'names': msg.name,
                'positions': list(msg.position),
                'velocities': list(msg.velocity),
                'effort': list(msg.effort)
            }
            self.joint_data_log.append(data_point)

    def imu_callback(self, msg):
        """Log IMU data."""
        if self.data_collection_active:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            data_point = {
                'timestamp': timestamp,
                'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            }
            self.imu_data_log.append(data_point)

    def validation_callback(self):
        """Perform validation analysis."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Stop collecting data after validation window
        if (current_time - self.start_time) > self.validation_window and self.data_collection_active:
            self.data_collection_active = False
            self.perform_validation_analysis()

    def perform_validation_analysis(self):
        """Analyze collected data for validation."""
        self.get_logger().info('Starting validation analysis...')

        # Check joint position ranges
        self.validate_joint_ranges()

        # Check IMU data consistency
        self.validate_imu_data()

        # Check for excessive oscillations
        self.validate_oscillations()

        # Save data to CSV for further analysis
        self.save_validation_data()

        self.get_logger().info('Validation analysis completed!')

    def validate_joint_ranges(self):
        """Validate that joint positions stay within expected ranges."""
        if not self.joint_data_log:
            return

        for data_point in self.joint_data_log:
            for i, pos in enumerate(data_point['positions']):
                if i < len(data_point['names']):
                    joint_name = data_point['names'][i]

                    # Define expected limits for different joints
                    if 'shoulder' in joint_name:
                        limit = 1.57  # 90 degrees
                    elif 'elbow' in joint_name:
                        limit = 1.57
                    elif 'hip' in joint_name:
                        limit = 2.0
                    elif 'knee' in joint_name:
                        limit = 2.0
                    elif 'ankle' in joint_name:
                        limit = 0.5
                    else:
                        limit = 2.0  # Default limit

                    if abs(pos) > limit:
                        self.get_logger().warn(
                            f'Joint {joint_name} exceeded expected limit: {pos} > {limit}'
                        )

    def validate_imu_data(self):
        """Validate IMU data for expected ranges."""
        if not self.imu_data_log:
            return

        for data_point in self.imu_data_log:
            # Check linear acceleration (should be around 9.8 m/s² when stationary)
            acc_norm = np.linalg.norm(data_point['linear_acceleration'])

            if abs(acc_norm - 9.8) > 2.0:  # Allow for movement
                self.get_logger().info(
                    f'Acceleration norm {acc_norm} differs significantly from gravity (9.8)'
                )

            # Check angular velocity ranges
            ang_vel_norm = np.linalg.norm(data_point['angular_velocity'])
            if ang_vel_norm > 10.0:  # Very high angular velocity
                self.get_logger().warn(f'High angular velocity detected: {ang_vel_norm}')

    def validate_oscillations(self):
        """Check for excessive oscillations in joint positions."""
        if len(self.joint_data_log) < 10:
            return

        # For each joint, check for oscillations
        if not self.joint_data_log[0]['names']:
            return

        for joint_idx in range(len(self.joint_data_log[0]['names'])):
            positions = []
            for data_point in self.joint_data_log:
                if joint_idx < len(data_point['positions']):
                    positions.append(data_point['positions'][joint_idx])

            if len(positions) < 10:
                continue

            # Calculate the frequency of oscillations using FFT (simplified)
            # Look for high-frequency oscillations that might indicate instability
            pos_array = np.array(positions)
            diff = np.diff(pos_array)
            velocity_variance = np.var(diff)

            if velocity_variance > 0.1:  # Arbitrary threshold
                joint_name = self.joint_data_log[0]['names'][joint_idx]
                self.get_logger().warn(
                    f'High velocity variance detected in {joint_name}: {velocity_variance}'
                )

    def save_validation_data(self):
        """Save validation data to CSV files."""
        # Save joint data
        if self.joint_data_log:
            with open('joint_validation_data.csv', 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'joint_name', 'position', 'velocity', 'effort']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                for data_point in self.joint_data_log:
                    for i in range(len(data_point['names'])):
                        if i < len(data_point['positions']):
                            writer.writerow({
                                'timestamp': data_point['timestamp'],
                                'joint_name': data_point['names'][i],
                                'position': data_point['positions'][i],
                                'velocity': data_point['velocities'][i] if i < len(data_point['velocities']) else 0,
                                'effort': data_point['effort'][i] if i < len(data_point['effort']) else 0
                            })

        # Save IMU data
        if self.imu_data_log:
            with open('imu_validation_data.csv', 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                             'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                             'linear_acc_x', 'linear_acc_y', 'linear_acc_z']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                for data_point in self.imu_data_log:
                    writer.writerow({
                        'timestamp': data_point['timestamp'],
                        'orientation_x': data_point['orientation'][0],
                        'orientation_y': data_point['orientation'][1],
                        'orientation_z': data_point['orientation'][2],
                        'orientation_w': data_point['orientation'][3],
                        'angular_vel_x': data_point['angular_velocity'][0],
                        'angular_vel_y': data_point['angular_velocity'][1],
                        'angular_vel_z': data_point['angular_velocity'][2],
                        'linear_acc_x': data_point['linear_acceleration'][0],
                        'linear_acc_y': data_point['linear_acceleration'][1],
                        'linear_acc_z': data_point['linear_acceleration'][2]
                    })

        self.get_logger().info('Validation data saved to CSV files')

def main(args=None):
    rclpy.init(args=args)
    validator = SimulationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.perform_validation_analysis()  # Force analysis on interrupt
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Exercises

### Setup Commands
```bash
# 1. Build your ROS 2 workspace
cd ~/physical_ai_ws
colcon build --packages-select your_simulation_package
source install/setup.bash

# 2. Launch Gazebo with your robot
ros2 launch your_package humanoid_gazebo.launch.py

# 3. Run the controller nodes in separate terminals
ros2 run your_package joint_controller
ros2 run your_package perception_node
ros2 run your_package walking_controller
ros2 run your_package sensor_fusion
ros2 run your_package simulation_validator
```

## Assessment Questions

1. How do you configure a camera sensor in Gazebo for a humanoid robot?
2. What are the key parameters to tune for stable walking in simulation?
3. How would you validate that your simulation accurately represents a real robot?
4. What are the trade-offs between simulation accuracy and computational performance?
5. How do you implement safety limits in a simulated humanoid robot?

## Next Steps

After completing these exercises, you should have experience with:
- Setting up Gazebo environments for humanoid robots
- Implementing joint control and sensor integration
- Creating walking controllers and perception systems
- Validating simulation accuracy

Continue to the next module to learn about NVIDIA Isaac platform integration.