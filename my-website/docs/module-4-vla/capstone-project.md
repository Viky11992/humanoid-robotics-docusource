---
sidebar_position: 4
title: Capstone Project - Integrated VLA System
---

# Capstone Project - Integrated VLA System

## Overview

The capstone project integrates all concepts learned throughout this textbook into a comprehensive Vision-Language-Action system for humanoid robots. You will build a complete system that can receive natural language commands, understand the environment through vision, and execute complex tasks.

## Project Objectives

By completing this capstone project, you will demonstrate:
- Integration of ROS 2, Gazebo simulation, and NVIDIA Isaac
- Implementation of Vision-Language-Action systems
- End-to-end functionality from perception to action
- Human-robot interaction through natural language

## Project Requirements

### Core Functionality
1. **Voice Command Reception**: System must accept and understand spoken commands
2. **Environmental Perception**: System must identify objects and understand spatial relationships
3. **Task Planning**: System must decompose high-level commands into executable actions
4. **Action Execution**: System must execute complex manipulation and navigation tasks
5. **Human Interaction**: System must provide feedback and handle errors gracefully

### Technical Requirements
- ROS 2 Humble Hawksbill or later
- Gazebo simulation environment
- Computer vision pipeline for object detection
- Speech recognition and natural language processing
- Motion planning for manipulation tasks
- Integration with humanoid robot model

## Project Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Voice Command Interface                      │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Speech        │    │  Natural        │    │   Command   │ │
│  │   Recognition   │───▶│  Language       │───▶│   Parser    │ │
│  │                 │    │  Processing     │    │             │ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Cognitive Planning Layer                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Task          │    │  Motion         │    │   Action    │ │
│  │   Decomposition │───▶│  Planning       │───▶│   Executor  │ │
│  │                 │    │                 │    │             │ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Robot Execution Layer                        │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Navigation    │    │  Manipulation   │    │   Feedback  │ │
│  │   Module        │    │  Module         │    │   Module    │ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation Steps

### Step 1: Environment Setup
1. Set up the simulation environment in Gazebo
2. Configure the humanoid robot model with necessary sensors
3. Install and configure required ROS 2 packages

```bash
# Install required packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install Python dependencies
pip install speechrecognition openai transformers torch
```

### Step 2: Perception System
Create a perception node that integrates multiple sensors:

```python
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String
import torch
import torchvision.transforms as T
from transformers import DetrForObjectDetection, DetrImageProcessor

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')

        # Initialize camera interface
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize object detection model
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")

        # Object detection publisher
        self.object_pub = rospy.Publisher('/detected_objects', String, queue_size=10)

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run object detection
        inputs = self.processor(images=cv_image, return_tensors="pt")
        outputs = self.model(**inputs)

        # Process results
        target_sizes = torch.tensor([cv_image.shape[:2]])
        results = self.processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.9)[0]

        # Prepare detection message
        detection_msg = String()
        objects_info = []

        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            object_info = {
                "label": self.model.config.id2label[int(label)],
                "confidence": round(score.item(), 3),
                "bbox": box
            }
            objects_info.append(object_info)

        detection_msg.data = str(objects_info)
        self.object_pub.publish(detection_msg)

        rospy.loginfo(f"Detected objects: {objects_info}")

if __name__ == '__main__':
    node = PerceptionNode()
    rospy.spin()
```

### Step 3: Voice Command Interface
Implement the speech recognition and command processing:

```python
#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_msgs.msg import String
import openai
import json

class VoiceCommandNode:
    def __init__(self):
        rospy.init_node('voice_command_node')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set up ROS publishers
        self.command_pub = rospy.Publisher('/parsed_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_status', String, queue_size=10)

        # Configure OpenAI API (use your actual API key)
        openai.api_key = rospy.get_param('~openai_api_key', 'YOUR_API_KEY_HERE')

        # Calibrate microphone for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        rospy.loginfo("Voice command node initialized")

    def listen_for_commands(self):
        with self.microphone as source:
            rospy.loginfo("Listening for voice commands...")
            self.audio_pub.publish(String(data="listening"))

            # Listen for audio with timeout
            try:
                audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=5.0)
                rospy.loginfo("Audio captured, processing...")

                # Recognize speech
                text = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"Recognized: {text}")

                # Process command with LLM
                self.process_command(text)

            except sr.WaitTimeoutError:
                rospy.loginfo("No speech detected within timeout")
            except sr.UnknownValueError:
                rospy.logerr("Could not understand audio")
                self.audio_pub.publish(String(data="unrecognized"))
            except sr.RequestError as e:
                rospy.logerr(f"Could not request results; {e}")

    def process_command(self, text):
        try:
            # Use OpenAI to parse the command
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": """You are a command parser for a humanoid robot. Parse the user's natural language command into structured JSON format. The JSON should include:
- 'action': The primary action (navigate, pick, place, speak, etc.)
- 'target_object': The object to interact with (if any)
- 'target_location': The location to go to or place object (if any)
- 'parameters': Additional parameters for the action
- 'priority': The priority level (low, medium, high)

Example: 'Please go to the kitchen and bring me the red cup' should return:
{
  "action": "fetch",
  "target_object": "red cup",
  "target_location": "kitchen",
  "parameters": {"delivery_location": "current_position"},
  "priority": "medium"
}"""},
                    {"role": "user", "content": text}
                ]
            )

            # Extract and publish the command
            command_str = response.choices[0].message.content.strip()

            # Clean up potential markdown formatting
            if command_str.startswith('```json'):
                command_str = command_str[7:]
            if command_str.endswith('```'):
                command_str = command_str[:-3]

            # Validate JSON
            command_json = json.loads(command_str)

            # Publish the parsed command
            self.command_pub.publish(String(data=json.dumps(command_json)))
            rospy.loginfo(f"Published command: {command_json}")

        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")
            self.audio_pub.publish(String(data="error"))

if __name__ == '__main__':
    node = VoiceCommandNode()

    # Main loop
    rate = rospy.Rate(0.2)  # Check every 5 seconds
    while not rospy.is_shutdown():
        node.listen_for_commands()
        rate.sleep()
```

### Step 4: Task Planning and Execution
Create the main orchestrator that coordinates all components:

```python
#!/usr/bin/env python3
import rospy
import actionlib
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from humanoid_robot_msgs.msg import TaskAction, TaskGoal

class CapstoneOrchestrator:
    def __init__(self):
        rospy.init_node('capstone_orchestrator')

        # Subscribe to parsed commands
        self.command_sub = rospy.Subscriber('/parsed_command', String, self.command_callback)

        # Subscribe to object detections
        self.object_sub = rospy.Subscriber('/detected_objects', String, self.object_callback)

        # Publishers
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=10)

        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.task_client = actionlib.SimpleActionClient('task_execution', TaskAction)

        # Wait for servers
        rospy.loginfo("Waiting for action servers...")
        self.move_base_client.wait_for_server()
        self.task_client.wait_for_server()

        # System state
        self.detected_objects = {}
        self.current_location = None

        rospy.loginfo("Capstone orchestrator initialized")

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
            rospy.loginfo(f"Received command: {command}")

            # Execute the command
            self.execute_command(command)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid JSON in command: {e}")

    def object_callback(self, msg):
        try:
            objects = eval(msg.data)  # Note: In production, use json.loads
            self.detected_objects = {obj['label']: obj for obj in objects}
            rospy.loginfo(f"Updated object detections: {list(self.detected_objects.keys())}")
        except Exception as e:
            rospy.logerr(f"Error processing object detection: {e}")

    def execute_command(self, command):
        """Execute the parsed command."""
        action = command.get('action', '').lower()

        if action == 'navigate':
            self.execute_navigation(command)
        elif action == 'fetch':
            self.execute_fetch(command)
        elif action == 'pick':
            self.execute_pick(command)
        elif action == 'place':
            self.execute_place(command)
        elif action == 'speak':
            self.execute_speak(command)
        else:
            rospy.logerr(f"Unknown action: {action}")

    def execute_navigation(self, command):
        """Navigate to a specified location."""
        target_location = command.get('target_location', '')

        # Define location poses (in a real system, these would be loaded from map)
        location_poses = {
            'kitchen': Pose(position=Point(x=2.0, y=1.0, z=0.0)),
            'living room': Pose(position=Point(x=-1.0, y=2.0, z=0.0)),
            'bedroom': Pose(position=Point(x=3.0, y=-2.0, z=0.0)),
            'office': Pose(position=Point(x=0.0, y=-3.0, z=0.0))
        }

        if target_location in location_poses:
            pose = location_poses[target_location]
            self.navigate_to_pose(pose)
        else:
            rospy.logerr(f"Unknown location: {target_location}")

    def execute_fetch(self, command):
        """Fetch an object from a location and bring it to the user."""
        target_object = command.get('target_object', '')
        target_location = command.get('target_location', '')

        # Navigate to location
        if target_location:
            self.execute_navigation({'target_location': target_location})

        # Find and pick up the object
        rospy.sleep(2.0)  # Wait for environment to be perceived
        if target_object in self.detected_objects:
            object_info = self.detected_objects[target_object]
            # In a real system, this would use the object's pose for picking
            rospy.loginfo(f"Attempting to pick up {target_object}")
            # Call manipulation service here
        else:
            rospy.logerr(f"Object {target_object} not found in current view")

    def navigate_to_pose(self, pose):
        """Navigate to a specific pose using move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        rospy.loginfo(f"Sending navigation goal: {pose}")
        self.move_base_client.send_goal(goal)

        # Wait for result with timeout
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(30.0))

        if not finished_within_time:
            self.move_base_client.cancel_goal()
            rospy.logerr("Navigation timed out")
        else:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation succeeded")
            else:
                rospy.logerr(f"Navigation failed with state: {state}")

    def execute_speak(self, command):
        """Execute a speak command."""
        text = command.get('parameters', {}).get('text', '')
        rospy.loginfo(f"Speaking: {text}")
        # In a real system, this would call a text-to-speech service

if __name__ == '__main__':
    orchestrator = CapstoneOrchestrator()
    rospy.spin()
```

### Step 5: Launch File
Create a launch file to start all components:

```xml
<launch>
  <!-- Start Gazebo simulation -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find humanoid_robot_description)/worlds/indoor.world"/>
  </include>

  <!-- Spawn the humanoid robot -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-file $(find humanoid_robot_description)/robots/humanoid.urdf -urdf -model humanoid_robot" />

  <!-- Start the perception node -->
  <node name="perception_node" pkg="capstone_project" type="perception_node.py" output="screen" />

  <!-- Start the voice command node -->
  <node name="voice_command_node" pkg="capstone_project" type="voice_command_node.py" output="screen">
    <param name="openai_api_key" value="YOUR_API_KEY_HERE" />
  </node>

  <!-- Start the capstone orchestrator -->
  <node name="capstone_orchestrator" pkg="capstone_project" type="capstone_orchestrator.py" output="screen" />

  <!-- Start navigation stack -->
  <include file="$(find nav2_bringup)/launch/navigation_launch.py" />

  <!-- Start MoveIt! for manipulation -->
  <include file="$(find humanoid_moveit_config)/launch/move_group.launch" />
</launch>
```

## Testing Scenarios

### Scenario 1: Simple Navigation
- Command: "Go to the kitchen"
- Expected: Robot navigates to kitchen location

### Scenario 2: Object Fetching
- Command: "Go to the living room and bring me the blue cup"
- Expected: Robot navigates to living room, detects blue cup, grasps it, and returns

### Scenario 3: Complex Task
- Command: "Go to the office, pick up the book, and place it on the table in the living room"
- Expected: Robot performs multi-step task with navigation, manipulation, and placement

## Evaluation Criteria

### Functionality (50%)
- Correct interpretation of voice commands
- Accurate object detection and localization
- Successful navigation to specified locations
- Proper execution of manipulation tasks

### Integration (30%)
- Seamless communication between components
- Proper error handling and recovery
- Real-time performance

### Robustness (20%)
- Handling of ambiguous commands
- Recovery from failed actions
- Adaptation to environmental changes

## Extensions and Improvements

### Advanced Features
1. **Multi-modal Interaction**: Combine voice, gesture, and visual attention
2. **Learning from Demonstration**: Learn new tasks from human demonstrations
3. **Collaborative Tasks**: Coordinate with multiple robots or humans
4. **Long-term Memory**: Remember object locations and user preferences

### Performance Improvements
1. **Optimized Perception**: Use more efficient neural networks
2. **Real-time Planning**: Implement faster planning algorithms
3. **Adaptive Interaction**: Adjust to user communication style

## Troubleshooting

### Common Issues
- **Speech Recognition Failures**: Check microphone permissions and audio quality
- **Object Detection Errors**: Verify camera calibration and lighting conditions
- **Navigation Failures**: Check map accuracy and obstacle detection
- **Communication Issues**: Verify ROS topic connections

### Debugging Tips
- Use `rostopic echo` to monitor message flow
- Use `rqt_graph` to visualize node connections
- Check log files with `roslog`
- Use RViz for visualization of robot state

## Next Steps

After completing this capstone project, you should:
1. Document your implementation and lessons learned
2. Evaluate the system's performance quantitatively
3. Identify areas for improvement and future work
4. Consider deploying the system on a physical robot platform