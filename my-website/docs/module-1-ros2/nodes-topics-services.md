---
sidebar_position: 2
title: ROS 2 Nodes, Topics, and Services
---

# ROS 2 Nodes, Topics, and Services

## Core Communication Architecture

ROS 2 uses a distributed communication architecture based on three primary communication patterns:

1. **Nodes**: Independent processes that perform computation
2. **Topics**: Asynchronous, many-to-many communication via publish/subscribe
3. **Services**: Synchronous, request-response communication

## Nodes

### What is a Node?

A **node** is an executable process that uses ROS 2 to communicate with other nodes. In humanoid robotics, nodes might represent:

- Sensor drivers (camera, LiDAR, IMU)
- Control algorithms (walking, grasping)
- Perception systems (object detection, SLAM)
- Planning modules (path planning, motion planning)

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Publishers/Subscribers

### Publish-Subscribe Pattern

Topics enable asynchronous communication using a publish-subscribe pattern:

- **Publishers**: Send messages to a topic
- **Subscribers**: Receive messages from a topic
- **Many-to-many**: Multiple publishers and subscribers can use the same topic

### Example: Joint State Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.1, 0.2, 0.3]  # radians
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Example: Joint State Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Services

### Request-Response Pattern

Services provide synchronous, bidirectional communication:

- **Service Client**: Sends a request and waits for a response
- **Service Server**: Receives requests and sends responses
- **One-to-one**: Each request gets exactly one response

### Example: Robot Movement Service

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class RobotMovementServer(Node):
    def __init__(self):
        super().__init__('robot_movement_server')
        self.srv = self.create_service(
            Trigger,
            'move_robot',
            self.move_robot_callback)

    def move_robot_callback(self, request, response):
        self.get_logger().info('Moving robot...')
        # Perform movement logic here
        response.success = True
        response.message = 'Robot movement completed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotMovementServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Actions (Advanced Topic Communication)

For long-running tasks, ROS 2 provides **actions**:

- **Goals**: Requests for long-running tasks
- **Results**: Final outcomes of tasks
- **Feedback**: Continuous updates during task execution

### Example: Walking Action

```python
import rclpy
import rclpy.action
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient, ActionServer
from control_msgs.action import FollowJointTrajectory

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'walk_trajectory',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing walking trajectory...')

        # Execute the walking trajectory
        feedback_msg = FollowJointTrajectory.Feedback()

        # Send feedback periodically
        for i in range(0, 100, 10):
            feedback_msg.desired.positions = [i * 0.01] * 6  # 6 joints
            goal_handle.publish_feedback(feedback_msg)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result
```

## Best Practices for Humanoid Robotics

### Node Design
- Keep nodes focused on single responsibilities
- Use composition over inheritance for complex behaviors
- Implement proper error handling and recovery

### Topic Management
- Use appropriate Quality of Service (QoS) settings for real-time requirements
- Avoid high-frequency publishing that could overwhelm the system
- Consider bandwidth limitations for wireless communication

### Service Considerations
- Use services for operations that should complete quickly
- For long operations, consider using actions instead
- Implement timeouts to prevent hanging requests

## Launch Files

To manage multiple nodes, ROS 2 uses launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='humanoid_control',
            executable='walking_controller',
            name='walking_controller'
        )
    ])
```

## Next Steps

Continue to the next section to learn about bridging Python AI agents to ROS controllers.