---
sidebar_position: 5
title: ROS 2 Examples for Humanoid Robotics
---

# ROS 2 Examples for Humanoid Robotics

## Getting Started Examples

### 1. Basic Publisher/Subscriber

This example demonstrates the fundamental ROS 2 communication pattern using humanoid-specific messages.

**Publisher (joint_command_publisher.py):**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer to send commands periodically
        self.timer = self.create_timer(2.0, self.send_joint_command)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        self.command_counter = 0

    def send_joint_command(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        # Create a simple oscillating pattern
        amplitude = 0.2
        position_offset = amplitude * (self.command_counter % 2)

        point.positions = [position_offset] * 6  # 6 joints
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint positions: {point.positions}')
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

**Subscriber (joint_state_subscriber.py):**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def joint_state_callback(self, msg):
        # Log joint positions with timestamps
        position_dict = dict(zip(msg.name, msg.position))
        self.get_logger().info(f'Joint positions: {position_dict}')

        # Example: Check if robot is in safe position
        if 'left_knee_joint' in position_dict:
            knee_pos = position_dict['left_knee_joint']
            if abs(knee_pos) > 2.0:  # Check for dangerous position
                self.get_logger().warn(f'Dangerous knee position: {knee_pos}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = JointStateSubscriber()

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

### 2. Service Example - Robot Control

**Service Server (robot_control_server.py):**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64MultiArray
from humanoid_msgs.srv import RobotCommand  # Custom service

class RobotControlServer(Node):
    def __init__(self):
        super().__init__('robot_control_server')

        # Service server
        self.srv = self.create_service(
            RobotCommand,
            'robot_command',
            self.handle_robot_command
        )

        # Publishers for different control interfaces
        self.body_publisher = self.create_publisher(
            Float64MultiArray,
            '/body_controller/commands',
            10
        )

        self.get_logger().info('Robot Control Server initialized')

    def handle_robot_command(self, request, response):
        command_type = request.command
        params = request.parameters

        self.get_logger().info(f'Received command: {command_type} with params: {params}')

        try:
            if command_type == 'walk':
                self.execute_walk_command(params)
            elif command_type == 'stand':
                self.execute_stand_command()
            elif command_type == 'sit':
                self.execute_sit_command()
            else:
                response.success = False
                response.message = f'Unknown command: {command_type}'
                return response

            response.success = True
            response.message = f'Command {command_type} executed successfully'

        except Exception as e:
            response.success = False
            response.message = f'Error executing command: {str(e)}'

        return response

    def execute_walk_command(self, params):
        # Extract walk parameters
        step_size = params[0] if len(params) > 0 else 0.1
        step_height = params[1] if len(params) > 1 else 0.05
        speed = params[2] if len(params) > 2 else 0.5

        # Send walk command to controller
        command_msg = Float64MultiArray()
        command_msg.data = [step_size, step_height, speed]
        self.body_publisher.publish(command_msg)

        self.get_logger().info(f'Walking: step_size={step_size}, height={step_height}, speed={speed}')

    def execute_stand_command(self):
        # Send stand command
        command_msg = Float64MultiArray()
        command_msg.data = [0.0, 0.0, 0.0]  # Zero joint positions for standing
        self.body_publisher.publish(command_msg)
        self.get_logger().info('Standing command sent')

    def execute_sit_command(self):
        # Send sit command (simplified)
        command_msg = Float64MultiArray()
        command_msg.data = [-0.5, -1.0, 0.5]  # Example sitting positions
        self.body_publisher.publish(command_msg)
        self.get_logger().info('Sitting command sent')

def main(args=None):
    rclpy.init(args=args)
    server = RobotControlServer()

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

## Advanced Examples

### 3. Action Example - Walking Trajectory

**Action Server (walking_action_server.py):**

```python
#!/usr/bin/env python3
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from humanoid_msgs.action import WalkToGoal  # Custom action
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')

        # Action server
        self._action_server = ActionServer(
            self,
            WalkToGoal,
            'walk_to_goal',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.get_logger().info('Walking Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info(f'Received walk goal: x={goal_request.target_pose.x}, y={goal_request.target_pose.y}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the walking action."""
        self.get_logger().info('Executing walking action...')

        feedback_msg = WalkToGoal.Feedback()
        result_msg = WalkToGoal.Result()

        target_x = goal_handle.request.target_pose.x
        target_y = goal_handle.request.target_pose.y

        # Calculate distance to target
        distance = math.sqrt(target_x**2 + target_y**2)
        steps = int(distance / 0.3)  # 0.3m per step

        self.get_logger().info(f'Walking {distance:.2f}m in {steps} steps')

        # Simulate walking by sending trajectory commands
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = 'Goal canceled'
                return result_msg

            # Calculate current progress
            progress = float(i) / float(steps)
            current_x = target_x * progress
            current_y = target_y * progress

            # Update feedback
            feedback_msg.current_pose.x = current_x
            feedback_msg.current_pose.y = current_y
            feedback_msg.distance_remaining = distance * (1.0 - progress)

            goal_handle.publish_feedback(feedback_msg)

            # Generate and publish walking trajectory
            self.publish_walking_trajectory(current_x, current_y, progress)

            # Sleep to simulate walking time
            await rclpy.asyncio.sleep(0.5)

        # Final position reached
        result_msg.success = True
        result_msg.message = 'Successfully reached target position'
        goal_handle.succeed()

        return result_msg

    def publish_walking_trajectory(self, current_x, current_y, progress):
        """Publish a walking trajectory step."""
        msg = JointTrajectory()
        msg.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        point = JointTrajectoryPoint()

        # Generate walking pattern based on progress
        # This is a simplified example - real walking controllers are much more complex
        walking_pattern = self.calculate_walking_pattern(progress)
        point.positions = walking_pattern
        point.velocities = [0.0] * len(walking_pattern)
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds

        msg.points = [point]
        self.trajectory_publisher.publish(msg)

    def calculate_walking_pattern(self, progress):
        """Calculate joint positions for walking pattern."""
        # Simplified walking pattern - in reality, this would use inverse kinematics
        # and dynamic balance control
        base_pattern = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Add oscillating pattern for walking motion
        oscillation = math.sin(progress * 2 * math.pi) * 0.1
        base_pattern[0] = oscillation  # Left hip
        base_pattern[3] = -oscillation  # Right hip (opposite phase)

        return base_pattern

def main(args=None):
    rclpy.init(args=args)
    server = WalkingActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. AI Integration Example

**AI-ROS Bridge Node (ai_behavior_controller.py):**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf  # Example AI framework
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
from humanoid_msgs.msg import RobotState, BehaviorCommand

class AIBehaviorController(Node):
    def __init__(self):
        super().__init__('ai_behavior_controller')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # AI model (placeholder - load your actual model)
        self.ai_model = self.load_ai_model()

        # Robot state tracking
        self.robot_state = RobotState()
        self.last_image = None
        self.behavior_command = None

        # Subscribers for sensor data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            # Assuming IMU message type - adjust as needed
            # This would be sensor_msgs.msg.Imu in practice
            String, '/imu_data', self.imu_callback, 10  # Placeholder
        )

        # Publishers for commands
        self.command_pub = self.create_publisher(
            BehaviorCommand, '/behavior_commands', 10
        )
        self.velocity_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Timer for AI decision loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)  # 10 Hz

        self.get_logger().info('AI Behavior Controller initialized')

    def load_ai_model(self):
        """Load the AI model for behavior prediction."""
        # Placeholder - replace with actual model loading
        # Example: return tf.keras.models.load_model('path/to/model')
        self.get_logger().info('Loading AI model...')
        return None

    def joint_callback(self, msg):
        """Update joint state in robot state."""
        self.robot_state.joint_positions = msg.position
        self.robot_state.joint_velocities = msg.velocity
        self.robot_state.joint_names = msg.name

    def image_callback(self, msg):
        """Process camera image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process IMU data (placeholder)."""
        # In practice, this would process actual IMU data
        # for balance and orientation information
        pass

    def ai_decision_loop(self):
        """Main AI decision-making loop."""
        if not self.robot_state.joint_names:  # Wait for initial state
            return

        if self.last_image is not None:
            try:
                # Prepare input for AI model
                ai_input = self.prepare_ai_input()

                # Get AI decision (placeholder)
                ai_decision = self.get_ai_decision(ai_input)

                # Convert AI decision to robot command
                command = self.convert_decision_to_command(ai_decision)

                # Publish command
                self.command_pub.publish(command)

                self.get_logger().info(f'AI decision: {ai_decision}')

            except Exception as e:
                self.get_logger().error(f'AI decision error: {e}')
                # Fallback to safe behavior
                self.publish_safe_behavior()

    def prepare_ai_input(self):
        """Prepare sensor data for AI model."""
        # Combine multiple sensor modalities
        input_data = {
            'joint_positions': np.array(self.robot_state.joint_positions),
            'joint_velocities': np.array(self.robot_state.joint_velocities),
            'image': self.last_image,
            'timestamp': self.get_clock().now().nanoseconds
        }
        return input_data

    def get_ai_decision(self, ai_input):
        """Get decision from AI model (placeholder)."""
        # Placeholder implementation
        # In practice, this would call your actual AI model
        if ai_input['image'] is not None:
            # Simple example: if we see something, move toward it
            return {'behavior': 'approach', 'target': [0.5, 0.0, 0.0]}
        else:
            return {'behavior': 'idle', 'target': [0.0, 0.0, 0.0]}

    def convert_decision_to_command(self, decision):
        """Convert AI decision to robot command."""
        command = BehaviorCommand()
        command.behavior_type = decision['behavior']
        command.target_position.x = decision['target'][0]
        command.target_position.y = decision['target'][1]
        command.target_position.z = decision['target'][2]
        return command

    def publish_safe_behavior(self):
        """Publish safe/fallback behavior."""
        # Stop any movement, maintain balance
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0
        self.velocity_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = AIBehaviorController()

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

## Launch Files

### Basic Launch File (humanoid_basic.launch.py)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'rate': 50
            }]
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'publish_frequency': 50.0
            }],
            arguments=[os.path.join(
                get_package_share_directory('humanoid_description'),
                'urdf',
                'humanoid.urdf'
            )]
        ),

        # Joint trajectory controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '-c', '/controller_manager']
        ),

        # Example nodes
        Node(
            package='humanoid_examples',
            executable='joint_command_publisher',
            name='joint_command_publisher'
        ),
        Node(
            package='humanoid_examples',
            executable='joint_state_subscriber',
            name='joint_state_subscriber'
        )
    ])
```

## Running the Examples

### 1. Setup and Dependencies

First, make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/iron/setup.bash
cd ~/physical_ai_ws
source install/setup.bash
```

### 2. Build Your Package

```bash
cd ~/physical_ai_ws
colcon build --packages-select humanoid_examples
source install/setup.bash
```

### 3. Run Individual Examples

```bash
# Run the joint command publisher
ros2 run humanoid_examples joint_command_publisher

# Run the joint state subscriber
ros2 run humanoid_examples joint_state_subscriber

# Run the robot control server
ros2 run humanoid_examples robot_control_server
```

### 4. Run with Launch File

```bash
ros2 launch humanoid_examples humanoid_basic.launch.py
```

## Testing Your Examples

### 1. Check Active Nodes

```bash
ros2 node list
```

### 2. Check Topics

```bash
ros2 topic list
ros2 topic echo /joint_states
```

### 3. Call Services

```bash
ros2 service call /robot_command humanoid_msgs/srv/RobotCommand "{command: 'stand', parameters: []}"
```

### 4. Send Action Goals

```bash
ros2 action send_goal /walk_to_goal humanoid_msgs/action/WalkToGoal "{target_pose: {x: 1.0, y: 0.0, z: 0.0}}"
```

## Troubleshooting Common Issues

### 1. Import Errors

Make sure all dependencies are installed:
```bash
pip3 install opencv-python tensorflow  # For AI examples
```

### 2. Permission Issues

Make sure your Python files are executable:
```bash
chmod +x your_script.py
```

### 3. Topic Connection Issues

Check if publishers and subscribers are on the same topic names and QoS settings.

### 4. URDF Issues

Validate your URDF files:
```bash
check_urdf /path/to/your/robot.urdf
```

## Next Steps

Continue to the next section to learn about creating ROS 2 exercises with solutions.