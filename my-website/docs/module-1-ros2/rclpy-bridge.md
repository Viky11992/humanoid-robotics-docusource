---
sidebar_position: 3
title: Bridging Python AI Agents to ROS Controllers
---

# Bridging Python AI Agents to ROS Controllers

## The AI-ROS Integration Challenge

In humanoid robotics, we need to connect sophisticated AI algorithms running in Python with the ROS 2 control system. This bridge enables:

- AI agents to control robot behavior based on perception data
- Real-time feedback from sensors to inform AI decisions
- Safe and efficient execution of AI-generated commands

## Understanding rclpy

**rclpy** is the Python client library for ROS 2. It provides the interface between Python code and the ROS 2 middleware.

### Basic Setup

```python
import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf  # Example AI framework
```

## AI Agent Integration Pattern

### 1. Perception Pipeline Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class AIPerceptionNode(Node):
    def __init__(self):
        super().__init__('ai_perception_node')

        # ROS 2 components
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # AI model initialization
        self.ai_model = self.load_ai_model()

        # Publisher for AI decisions
        self.ai_publisher = self.create_publisher(String, '/ai_decisions', 10)

    def load_ai_model(self):
        # Load your AI model here
        # Example: return tf.keras.models.load_model('path/to/model')
        pass

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process with AI model
        ai_decision = self.ai_model.predict(cv_image)

        # Publish AI decision
        decision_msg = String()
        decision_msg.data = str(ai_decision)
        self.ai_publisher.publish(decision_msg)
```

### 2. Control Command Bridge

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AIControlBridge(Node):
    def __init__(self):
        super().__init__('ai_control_bridge')

        # Subscribe to AI decisions
        self.ai_subscription = self.create_subscription(
            String,
            '/ai_decisions',
            self.ai_decision_callback,
            10
        )

        # Publish to robot controllers
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

    def ai_decision_callback(self, msg):
        # Parse AI decision
        decision = eval(msg.data)  # In practice, use safer parsing

        # Convert AI decision to robot commands
        trajectory = self.convert_decision_to_trajectory(decision)

        # Publish trajectory
        self.trajectory_publisher.publish(trajectory)

    def convert_decision_to_trajectory(self, decision):
        # Convert AI decision to joint trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']

        point = JointTrajectoryPoint()
        point.positions = decision['joint_positions']
        point.velocities = decision['joint_velocities']
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory.points = [point]
        return trajectory
```

## Advanced Integration: Behavior Trees

For complex humanoid behaviors, integrate AI decision-making with behavior trees:

```python
class AIControlNode(Node):
    def __init__(self):
        super().__init__('ai_control_node')

        # Multiple subscribers for different sensor types
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(ImU, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Publishers for different control interfaces
        self.body_controller = self.create_publisher(JointTrajectory, '/body_controller/joint_trajectory', 10)
        self.arm_controller = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.head_controller = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

        # AI model and state
        self.ai_model = self.initialize_ai_model()
        self.robot_state = {}  # Current robot state
        self.ai_state = {}     # Current AI state

    def image_callback(self, msg):
        # Process visual input
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.robot_state['image'] = cv_image

    def imu_callback(self, msg):
        # Process IMU data for balance
        self.robot_state['imu'] = {
            'orientation': msg.orientation,
            'angular_velocity': msg.angular_velocity,
            'linear_acceleration': msg.linear_acceleration
        }

    def joint_callback(self, msg):
        # Process joint state for feedback
        self.robot_state['joint_positions'] = msg.position
        self.robot_state['joint_velocities'] = msg.velocity

    def timer_callback(self):
        # Main AI control loop
        if self.robot_state:  # If we have sensor data
            ai_command = self.ai_model.predict(self.robot_state)
            self.execute_ai_command(ai_command)

    def execute_ai_command(self, command):
        # Convert AI command to ROS messages
        if command['type'] == 'walk':
            self.publish_walk_command(command['params'])
        elif command['type'] == 'gesture':
            self.publish_gesture_command(command['params'])
        elif command['type'] == 'balance':
            self.publish_balance_command(command['params'])
```

## Safety Considerations

### 1. Command Validation

```python
def validate_command(self, command):
    """Validate AI-generated commands before execution"""
    # Check joint limits
    for pos in command.joint_positions:
        if pos < self.min_joint_limit or pos > self.max_joint_limit:
            self.get_logger().warn(f'Joint position {pos} outside limits')
            return False

    # Check velocity limits
    for vel in command.joint_velocities:
        if abs(vel) > self.max_velocity_limit:
            self.get_logger().warn(f'Joint velocity {vel} exceeds limit')
            return False

    return True
```

### 2. Fallback Behaviors

```python
def ai_with_fallback(self):
    """AI control with safety fallback"""
    try:
        ai_command = self.ai_model.predict(self.robot_state)
        if self.validate_command(ai_command):
            self.execute_command(ai_command)
        else:
            self.execute_safety_fallback()
    except Exception as e:
        self.get_logger().error(f'AI error: {e}')
        self.execute_safety_fallback()
```

## Performance Optimization

### 1. Threading Considerations

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class OptimizedAINode(Node):
    def __init__(self):
        super().__init__('optimized_ai_node')
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Timer for main control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def control_loop(self):
        # Submit AI prediction to thread pool
        future = self.executor.submit(self.ai_prediction, self.robot_state)
        # Process result when ready
```

### 2. Model Optimization

```python
def optimize_ai_model(self):
    """Optimize AI model for real-time inference"""
    # Use TensorRT for NVIDIA GPUs
    # Use ONNX Runtime for cross-platform optimization
    # Apply quantization for faster inference
    pass
```

## Real-World Example: Conversational AI Integration

```python
from std_msgs.msg import String
import speech_recognition as sr
import openai  # or similar

class ConversationalAINode(Node):
    def __init__(self):
        super().__init__('conversational_ai_node')

        # Audio input
        self.audio_sub = self.create_subscription(String, '/audio_input', self.audio_callback, 10)

        # Robot command output
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)

        # Initialize AI components
        self.speech_recognizer = sr.Recognizer()
        self.conversation_model = self.load_conversation_model()

    def audio_callback(self, msg):
        # Process speech command
        user_input = self.process_speech(msg.data)
        ai_response = self.conversation_model.generate_response(user_input)
        robot_command = self.extract_robot_command(ai_response)

        # Publish command to robot
        cmd_msg = String()
        cmd_msg.data = robot_command
        self.command_pub.publish(cmd_msg)
```

## Integration Testing

Test the AI-ROS bridge with:

1. **Unit Tests**: Test individual components in isolation
2. **Integration Tests**: Test AI-ROS communication
3. **Simulation Tests**: Test in Gazebo before real robot deployment
4. **Safety Tests**: Verify safety constraints are enforced

## Next Steps

Continue to the next section to learn about URDF (Unified Robot Description Format) for humanoids.