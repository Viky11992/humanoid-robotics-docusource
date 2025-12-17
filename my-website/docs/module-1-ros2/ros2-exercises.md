---
sidebar_position: 6
title: ROS 2 Exercises for Humanoid Robotics
---

# ROS 2 Exercises for Humanoid Robotics

## Exercise 1: Basic Publisher-Subscriber Pattern

### Objective
Create a simple ROS 2 publisher that sends joint position commands and a subscriber that logs received positions.

### Tasks
1. Create a publisher node that publishes joint positions to `/joint_commands`
2. Create a subscriber node that listens to `/joint_commands` and logs the positions
3. Test the communication between nodes

### Solution Template

**Publisher (exercise1_publisher.py):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('exercise1_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_commands)
        self.command_counter = 0

    def publish_joint_commands(self):
        msg = Float64MultiArray()
        # TODO: Create joint position array with 6 values (example: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with your values
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint commands: {msg.data}')
        self.command_counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber (exercise1_subscriber.py):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__('exercise1_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

    def joint_command_callback(self, msg):
        # TODO: Log the received joint commands
        self.get_logger().info(f'Received joint commands: {msg.data}')
        # TODO: Add validation to check if joint positions are within safe limits
        for i, pos in enumerate(msg.data):
            if abs(pos) > 2.0:  # Safe limit example
                self.get_logger().warn(f'Joint {i} position {pos} exceeds safe limit!')

def main(args=None):
    rclpy.init(args=args)
    subscriber = JointCommandSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Exercise
```bash
# Terminal 1: Run the publisher
ros2 run your_package exercise1_publisher

# Terminal 2: Run the subscriber
ros2 run your_package exercise1_subscriber
```

## Exercise 2: Service-Based Robot Control

### Objective
Create a service server that accepts robot commands and a client that sends commands.

### Tasks
1. Create a service definition file (RobotCommand.srv)
2. Implement a service server that handles different robot commands
3. Create a service client that sends commands to the server

### Solution Template

**Service Definition (RobotCommand.srv):**
```
# Command type (e.g., "stand", "sit", "walk")
string command
# Optional parameters for the command
float64[] parameters
---
# Success status
bool success
# Result message
string message
```

**Service Server (exercise2_server.py):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from your_package.srv import RobotCommand  # Replace with your service package

class RobotCommandServer(Node):
    def __init__(self):
        super().__init__('exercise2_server')
        self.srv = self.create_service(RobotCommand, 'robot_command', self.handle_robot_command)
        self.get_logger().info('Robot Command Server started')

    def handle_robot_command(self, request, response):
        command = request.command
        params = request.parameters

        self.get_logger().info(f'Received command: {command} with params: {params}')

        # TODO: Implement command handling logic
        if command == 'stand':
            response.success = True
            response.message = 'Standing up...'
        elif command == 'sit':
            response.success = True
            response.message = 'Sitting down...'
        elif command == 'walk':
            if len(params) >= 2:
                distance = params[0]
                speed = params[1]
                response.success = True
                response.message = f'Walking {distance}m at speed {speed}'
            else:
                response.success = False
                response.message = 'Walk command requires distance and speed parameters'
        else:
            response.success = False
            response.message = f'Unknown command: {command}'

        return response

def main(args=None):
    rclpy.init(args=args)
    server = RobotCommandServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client (exercise2_client.py):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from your_package.srv import RobotCommand  # Replace with your service package

class RobotCommandClient(Node):
    def __init__(self):
        super().__init__('exercise2_client')
        self.cli = self.create_client(RobotCommand, 'robot_command')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = RobotCommand.Request()

    def send_command(self, command, params=[]):
        self.req.command = command
        self.req.parameters = params

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info(f'Result: {response.success}, {response.message}')
            return response
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = RobotCommandClient()

    # TODO: Send different commands to test the server
    client.send_command('stand')
    client.send_command('walk', [2.0, 0.5])  # 2m distance, 0.5 speed
    client.send_command('sit')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: Action-Based Walking Controller

### Objective
Create an action server for walking to a goal position and a client that sends goals.

### Tasks
1. Create an action definition file (WalkToGoal.action)
2. Implement an action server that moves the robot to a goal position
3. Create an action client that sends walking goals

### Solution Template

**Action Definition (WalkToGoal.action):**
```
# Goal: target position to walk to
geometry_msgs/Point target_pose
---
# Result: success status and message
bool success
string message
---
# Feedback: current progress
geometry_msgs/Point current_pose
float64 distance_remaining
```

**Action Server (exercise3_server.py):**
```python
#!/usr/bin/env python3
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from your_package.action import WalkToGoal  # Replace with your action package
from geometry_msgs.msg import Point
import math

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('exercise3_server')
        self._action_server = ActionServer(
            self,
            WalkToGoal,
            'walk_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received walk goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing walk to goal...')

        feedback_msg = WalkToGoal.Feedback()
        result_msg = WalkToGoal.Result()

        target_x = goal_handle.request.target_pose.x
        target_y = goal_handle.request.target_pose.y

        # Calculate distance to target
        distance = math.sqrt(target_x**2 + target_y**2)
        self.get_logger().info(f'Walking to ({target_x}, {target_y}), distance: {distance:.2f}m')

        # Simulate walking progress
        steps = int(distance / 0.1)  # 0.1m per step
        for i in range(steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = 'Goal canceled'
                return result_msg

            # Calculate current progress
            progress = float(i) / float(steps) if steps > 0 else 1.0
            current_x = target_x * progress
            current_y = target_y * progress

            # Update feedback
            feedback_msg.current_pose.x = current_x
            feedback_msg.current_pose.y = current_y
            feedback_msg.distance_remaining = distance * (1.0 - progress)

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {progress*100:.1f}%, Remaining: {feedback_msg.distance_remaining:.2f}m')

            # Sleep to simulate walking time
            await rclpy.asyncio.sleep(0.2)

        # Goal reached
        result_msg.success = True
        result_msg.message = f'Successfully reached goal at ({target_x}, {target_y})'
        goal_handle.succeed()

        return result_msg

def main(args=None):
    rclpy.init(args=args)
    server = WalkingActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: URDF Robot Model Creation

### Objective
Create a simple humanoid robot URDF model with basic links and joints.

### Tasks
1. Create a URDF file defining a simple humanoid robot
2. Include torso, head, and two arms with joints
3. Add visual and collision properties
4. Validate the URDF file

### Solution Template

**Simple Humanoid URDF (simple_humanoid.urdf):**
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
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
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
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

### Validation Commands
```bash
# Validate the URDF
check_urdf simple_humanoid.urdf

# Visualize the URDF structure
urdf_to_graphiz simple_humanoid.urdf
```

## Exercise 5: AI-ROS Integration

### Objective
Create a node that receives sensor data, processes it with a simple AI algorithm, and sends commands based on the AI's decision.

### Tasks
1. Create a subscriber for sensor data (e.g., joint states, IMU)
2. Implement a simple decision-making algorithm
3. Publish commands based on the AI's decision
4. Add safety checks to ensure safe operation

### Solution Template

**AI Controller Node (exercise5_ai_controller.py):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class AIController(Node):
    def __init__(self):
        super().__init__('exercise5_ai_controller')

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        # Robot state
        self.joint_positions = []
        self.joint_names = []

        # Timer for AI decision loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)  # 10 Hz

        self.get_logger().info('AI Controller initialized')

    def joint_callback(self, msg):
        """Update joint state from sensor data."""
        self.joint_positions = list(msg.position)
        self.joint_names = list(msg.name)

    def ai_decision_loop(self):
        """Main AI decision-making loop."""
        if not self.joint_names:
            return  # Wait for initial data

        # TODO: Implement simple AI decision making
        # Example: If any joint is near its limit, move away from the limit
        safe_positions = self.ensure_joint_limits(self.joint_positions)

        # TODO: Implement additional AI behaviors
        # Example: Balance control, obstacle avoidance, etc.
        ai_command = self.make_ai_decision(safe_positions)

        # Publish command
        command_msg = Float64MultiArray()
        command_msg.data = ai_command
        self.command_pub.publish(command_msg)

        self.get_logger().info(f'AI command sent: {ai_command}')

    def ensure_joint_limits(self, positions):
        """Ensure joint positions are within safe limits."""
        # Define safe limits (example values)
        min_limit = -1.5
        max_limit = 1.5

        safe_positions = []
        for pos in positions:
            if pos < min_limit:
                safe_positions.append(min_limit + 0.1)  # Add small margin
            elif pos > max_limit:
                safe_positions.append(max_limit - 0.1)  # Add small margin
            else:
                safe_positions.append(pos)

        return safe_positions

    def make_ai_decision(self, positions):
        """Simple AI decision making."""
        # Example: Apply small adjustments to maintain balance
        # This is a very simplified example - real AI would be much more complex
        adjustments = [0.0] * len(positions)

        # Simple balance algorithm: adjust based on current positions
        for i, pos in enumerate(positions):
            # Apply small correction to move toward neutral position
            adjustment = -0.1 * pos  # Proportional correction
            adjustments[i] = pos + adjustment

            # Ensure result is within limits
            adjustments[i] = max(-1.5, min(1.5, adjustments[i]))

        return adjustments

def main(args=None):
    rclpy.init(args=args)
    controller = AIController()

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

## Exercise 6: Launch File Creation

### Objective
Create a launch file that starts multiple ROS 2 nodes for a complete humanoid robot system.

### Tasks
1. Create a launch file that starts joint state publisher
2. Include robot state publisher with URDF
3. Launch your custom nodes from previous exercises
4. Add parameters for different configurations

### Solution Template

**Launch File (humanoid_system.launch.py):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'simple_humanoid.urdf'
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': False},
                {'robot_description': open(urdf_file).read()}
            ],
            remappings=[
                ('/joint_states', 'joint_states')
            ]
        ),

        # Joint State Publisher (GUI for testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # Exercise 1 nodes
        Node(
            package='your_package',
            executable='exercise1_publisher',
            name='joint_command_publisher',
            parameters=[
                {'publish_rate': 1.0}
            ]
        ),

        Node(
            package='your_package',
            executable='exercise1_subscriber',
            name='joint_command_subscriber'
        ),

        # Exercise 5 AI Controller
        Node(
            package='your_package',
            executable='exercise5_ai_controller',
            name='ai_controller',
            parameters=[
                {'control_frequency': 10.0}
            ]
        )
    ])
```

## Exercise Solutions

### Running All Exercises

1. **Build your package:**
```bash
cd ~/physical_ai_ws
colcon build --packages-select your_package
source install/setup.bash
```

2. **Run individual exercises:**
```bash
# Exercise 1
ros2 run your_package exercise1_publisher
ros2 run your_package exercise1_subscriber

# Exercise 2
ros2 run your_package exercise2_server
ros2 run your_package exercise2_client

# Exercise 3
ros2 run your_package exercise3_server
# Use ros2 action send_goal to test

# Exercise 5
ros2 run your_package exercise5_ai_controller
```

3. **Run with launch file:**
```bash
ros2 launch your_package humanoid_system.launch.py
```

## Assessment Questions

1. What is the difference between a ROS 2 topic and a service?
2. How do you ensure safe joint limits in a humanoid robot controller?
3. What are the advantages of using actions over services for long-running tasks?
4. How would you integrate an AI model with a ROS 2 node?
5. What are the key components of a URDF file for a humanoid robot?

## Next Steps

After completing these exercises, you should have a solid understanding of:
- ROS 2 communication patterns (topics, services, actions)
- Robot modeling with URDF
- Basic AI integration with ROS 2
- Launch file creation and management

Continue to the next module to learn about simulation environments with Gazebo and Unity.