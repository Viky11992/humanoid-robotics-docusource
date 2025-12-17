---
sidebar_position: 1
title: ROS 2 Introduction - The Robotic Nervous System
---

# ROS 2 Introduction - The Robotic Nervous System

## Overview

ROS 2 (Robot Operating System 2) serves as the middleware foundation for robot control. Think of it as the **nervous system** of your robot, enabling different components to communicate and work together seamlessly.

## Core Concepts

### Nodes
Nodes are individual processes that perform specific functions. In a humanoid robot, you might have:
- Sensor nodes (processing camera, LiDAR, IMU data)
- Control nodes (processing movement commands)
- Perception nodes (identifying objects, people)
- Navigation nodes (path planning and obstacle avoidance)

### Topics
Topics enable asynchronous communication between nodes through a publish-subscribe model:
- Sensor data flows from sensor nodes to processing nodes
- Control commands flow from decision-making nodes to actuator nodes
- Transform data flows between different coordinate frames

### Services
Services provide synchronous request-response communication:
- Request specific actions (e.g., "move arm to position X")
- Get current robot state information
- Execute complex behaviors with guaranteed completion

## ROS 2 Architecture for Humanoids

Humanoid robots require specialized considerations in their ROS 2 architecture:

### URDF Integration
The Unified Robot Description Format (URDF) describes the robot's physical structure:
- Joint configurations for bipedal locomotion
- Link properties for proper physics simulation
- Sensor placements for optimal perception

### Real-time Considerations
Humanoid robots often require real-time performance:
- Deterministic message delivery
- Low-latency control loops
- Priority-based task scheduling

## Practical Implementation

### Setting up a Basic ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from humanoid controller: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_controller = HumanoidController()
    rclpy.spin(humanoid_controller)
    humanoid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

Continue to the next section to learn about Nodes, Topics, and Services in greater detail.