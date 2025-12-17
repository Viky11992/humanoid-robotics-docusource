---
sidebar_position: 5
title: Glossary of Terms
---

# Glossary of Terms

## Overview

This glossary provides definitions for key terms, acronyms, and concepts used throughout the Physical AI and Humanoid Robotics textbook. Understanding these terms is essential for effective communication and implementation of robotics concepts.

## A

**Actuator**: A component of a robot that converts control signals into physical movement. Examples include servo motors, stepper motors, and hydraulic/pneumatic cylinders.

**AI (Artificial Intelligence)**: The simulation of human intelligence processes by machines, especially computer systems. In robotics, AI enables perception, decision-making, and learning capabilities.

**Autonomous**: Operating independently without human intervention. An autonomous robot can perceive its environment and make decisions to achieve goals without external control.

## B

**Behavior-Based Robotics**: An approach to robotics that structures control systems as collections of individual behaviors that interact to produce complex behavior.

**Bipedal**: Having two legs. In humanoid robotics, bipedal locomotion refers to walking on two legs like humans.

**Body Schema**: A representation of the robot's physical configuration including joint angles, link positions, and spatial relationships between body parts.

## C

**C.O.M. (Center of Mass)**: The point where the total mass of the robot body is considered to be concentrated, crucial for balance and stability analysis.

**Computer Vision**: A field of AI that trains computers to interpret and understand the visual world. In robotics, it enables robots to identify objects, navigate, and understand their environment.

**Control Theory**: The interdisciplinary mathematical study of systems that maintain values and behaviors. In robotics, it governs how robots achieve desired movements and behaviors.

**Covariance**: A statistical measure of how two variables change together. In robotics, often used in sensor fusion and state estimation (e.g., Extended Kalman Filter).

**CV (Computer Vision)**: See Computer Vision.

## D

**Degrees of Freedom (DOF)**: The number of independent movements a mechanical system can make. For a humanoid robot, DOF refers to the number of independent joints and their possible movements.

**DH Parameters (Denavit-Hartenberg Parameters)**: A method to define coordinate frames on the links of a robot manipulator, used for kinematic analysis.

**Dynamics**: The study of forces and torques and their effect on motion. In robotics, dynamics models how forces affect robot movement.

**Dynamic Walking**: A walking pattern where the robot is never statically stable; it must constantly move to maintain balance, similar to human walking.

## E

**Embodied AI**: AI systems that function in physical space and comprehend physical laws. Unlike traditional AI that operates in digital spaces, embodied AI must interact with the physical world.

**End Effector**: The device at the end of a robotic arm that interacts with the environment. For humanoid robots, this typically refers to the hand.

**Episodic Memory**: Memory of specific events or episodes. In robotics, this could be memory of specific tasks or interactions that occurred in the past.

**Extended Kalman Filter (EKF)**: A nonlinear version of the Kalman filter that linearizes around the current estimate. Used in robotics for sensor fusion and state estimation.

## F

**Forward Kinematics**: The process of calculating the position and orientation of the robot's end effector given the joint angles.

**Fiducial Markers**: Visual markers with known geometry and patterns used for computer vision applications to determine position and orientation in 3D space.

**Foveated Vision**: A visual system that focuses high-resolution sensing on a small region of interest while maintaining lower resolution for peripheral vision, similar to human vision.

## G

**Gazebo**: A physics-based simulation environment commonly used in robotics for testing algorithms, robot designs, and scenarios without real hardware.

**Generative AI**: AI systems that can generate new content such as text, images, or other data. In robotics, this includes systems that generate motion plans or behaviors.

**Gripper**: A device that grasps objects. In humanoid robots, this refers to the hand mechanism used for manipulation.

## H

**Humanoid**: Having human-like characteristics or form. In robotics, humanoid robots are designed with human-like body structures including torso, head, arms, and legs.

**Human-Robot Interaction (HRI)**: The field of study focused on the design and implementation of robots that can interact effectively with humans.

## I

**IK (Inverse Kinematics)**: The mathematical process of determining joint angles required to achieve a desired end-effector position and orientation.

**IMU (Inertial Measurement Unit)**: A device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers, gyroscopes, and magnetometers.

**Intentional Behavior**: Robot actions that are goal-directed and purposeful, rather than reactive.

**Inverse Dynamics**: Calculating the forces and torques required to achieve a given motion trajectory.

## J

**Joint**: A connection between two links in a robot that allows relative motion between them.

**Joint Limits**: The physical or software constraints that define the minimum and maximum allowed positions for a joint.

**Jacobian Matrix**: A matrix that contains first-order partial derivatives of a vector function. In robotics, it relates joint velocities to end-effector velocities.

## K

**Kinematics**: The study of motion without considering the forces that cause it. Forward and inverse kinematics are fundamental to robot control.

**Kalman Filter**: An algorithm that uses a series of measurements observed over time to estimate unknown variables, often used in robotics for sensor fusion.

**Kinesthetic Teaching**: A method of programming robots by physically guiding them through desired motions.

## L

**Legged Locomotion**: Movement using legs. In humanoid robotics, this refers to walking or running on two legs.

**LIDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser to measure distances and create 3D maps of the environment.

**Local Planning**: Path planning that focuses on short-term navigation decisions based on immediate sensor data.

**Locomotion**: The act of moving from one place to another. In humanoid robotics, this refers to walking, running, or other forms of movement.

## M

**Manipulation**: The ability to purposefully control and move objects in the environment, typically using hands or end effectors.

**Manipulator**: A robot arm or other device designed to manipulate objects in the environment.

**Marker Tracking**: The process of identifying and following specific visual markers in a video stream for positioning and orientation purposes.

**Mechanical Design**: The process of designing the physical structure, joints, and components of a robot.

**Motor Primitives**: Basic movement patterns that can be combined to produce complex behaviors. In robotics, these are often parameterized movement patterns.

**Motion Capture**: The process of recording the movement of objects or people, often used to teach robots human-like movements.

**Motion Planning**: The process of determining a sequence of valid configurations that moves a robot from a start state to a goal state while avoiding obstacles.

## N

**Navigation**: The ability to move through an environment to reach a goal location while avoiding obstacles.

**Neuromorphic Computing**: Computing systems that mimic the neural structure and functioning of the human brain, potentially useful for robotic control.

**Node**: In ROS (Robot Operating System), a process that performs computation. Nodes are organized into packages and communicate via topics, services, or actions.

## O

**Odometry**: Estimation of change in position over time, typically by using data from motion sensors.

**Open-Loop Control**: Control system that does not use feedback. The system does not adjust based on the actual output.

**Omnidirectional**: Ability to move in any direction. An omnidirectional robot can move forward, backward, left, right, and rotate independently.

**Operational Space**: The space in which the robot's end effector operates, typically 3D Cartesian space with orientation (position and orientation of the end effector).

## P

**Path Planning**: The process of determining a geometric path from start to goal, without considering the robot's dynamics or timing.

**Perception**: The ability to interpret sensory information to understand the environment. In robotics, this includes computer vision, audio processing, and other sensing modalities.

**PID Controller (Proportional-Integral-Derivative Controller)**: A control loop mechanism that uses feedback to continuously calculate an error value as the difference between a desired setpoint and measured process variable.

**Point Cloud**: A set of data points in space, typically produced by 3D scanning systems like LIDAR or stereo vision systems.

**Pose**: The position and orientation of a robot or object in space, typically described by x, y, z coordinates and rotation angles.

**Posture**: The configuration of a robot's joints at a particular moment, describing how the robot is positioned.

## R

**Real-time Control**: Control systems that must respond to inputs within a guaranteed time constraint, essential for stable robot control.

**Reactive Control**: Control strategies that respond directly to sensory inputs without planning or prediction.

**Reference Frame**: A coordinate system used to describe the position and orientation of objects in space.

**Remote Operation**: Operating a robot from a distance, often using teleoperation techniques.

**Robot Operating System (ROS)**: Flexible framework for writing robot software that provides services like hardware abstraction, device drivers, libraries, visualizers, and message-passing.

**Robotics Middleware**: Software that provides common services and capabilities for robot applications, like ROS, ROS 2, or YARP.

**ROS 2**: The second generation of the Robot Operating System, designed for production robotics applications with improved security and real-time capabilities.

## S

**Sensors**: Devices that measure physical properties and convert them into signals that can be read by computers. Examples include cameras, IMUs, force sensors, and range finders.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**Simulation**: The process of creating a virtual model of a system to test and validate algorithms before deployment on real hardware.

**Stereopsis**: The perception of depth and 3D structure from visual information provided by two eyes.

**SVM (Support Vector Machine)**: A supervised learning model used for classification and regression analysis, applicable in robot perception tasks.

## T

**Task Planning**: High-level planning that determines the sequence of actions needed to achieve a goal, often involving symbolic reasoning.

**Teleoperation**: Controlling a robot remotely, typically with direct human control over robot movements.

**Trajectory**: A time-parameterized path that specifies not just positions but also velocities and accelerations at each point in time.

**Trajectory Planning**: The process of determining a smooth, feasible path through space that includes timing information.

**Transform**: A mathematical operation that converts coordinates from one reference frame to another.

**Twist**: In ROS, a message type representing velocity in free space specified by its linear and angular parts.

## U

**Ubiquitous Computing**: Computing that is integrated into everyday objects and activities, making robotics part of the environment rather than separate devices.

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model, including kinematic and dynamic properties, visual appearance, and collision models.

**USD (Universal Scene Description)**: NVIDIA's scene description format for 3D graphics and simulation applications.

## V

**VLA (Vision-Language-Action)**: Systems that integrate visual perception, language understanding, and physical action in robotic systems.

**Visual Servoing**: A control strategy that uses visual feedback to control robot motion.

**Voice Command**: Natural language commands given to a robot using speech recognition systems.

**Vision System**: The combination of hardware and software used for visual perception in robotics.

## W

**Whole Body Control**: A control approach that coordinates all degrees of freedom in a robot simultaneously to achieve multiple tasks.

**Workspace**: The set of all positions that the end effector of a robot can reach.

**Walking Gait**: The pattern of leg movements used for locomotion. In humanoid robots, this involves coordinated movement of legs, torso, and arms for stable walking.

## X, Y, Z

**Yaw**: Rotation around the vertical axis of a robot or coordinate system.

**ZMP (Zero Moment Point)**: A concept in robotics and biomechanics that is useful for assessing the stability of legged robots during walking.

## Acronyms and Abbreviations

**AI**: Artificial Intelligence

**API**: Application Programming Interface

**CPU**: Central Processing Unit

**CV**: Computer Vision

**DDS**: Data Distribution Service

**DOF**: Degrees of Freedom

**EKF**: Extended Kalman Filter

**GPU**: Graphics Processing Unit

**HRI**: Human-Robot Interaction

**HTTP**: HyperText Transfer Protocol

**I/O**: Input/Output

**ICP**: Iterative Closest Point

**IMU**: Inertial Measurement Unit

**IoT**: Internet of Things

**IP**: Internet Protocol

**IPS**: Inverse Positioning System

**JSON**: JavaScript Object Notation

**LIDAR**: Light Detection and Ranging

**LLM**: Large Language Model

**LOD**: Level of Detail

**NLP**: Natural Language Processing

**NVIDIA**: Originally "NV" (next version) and "IDA" (Interactive Digital Arts)

**OD**: Odometry

**PID**: Proportional-Integral-Derivative

**QoS**: Quality of Service

**RAM**: Random Access Memory

**R&D**: Research and Development

**RCS**: Real-time Control System

**RMW**: ROS Middleware

**ROS**: Robot Operating System

**RTF**: Real-Time Factor

**Rviz**: ROS Visualization Tool

**SDK**: Software Development Kit

**SLAM**: Simultaneous Localization and Mapping

**SNN**: Spiking Neural Network

**SVM**: Support Vector Machine

**TCS**: Tactile Control System

**UDP**: User Datagram Protocol

**USD**: Universal Scene Description

**URDF**: Unified Robot Description Format

**USB**: Universal Serial Bus

**VLA**: Vision-Language-Action

**VR**: Virtual Reality

**VRAM**: Video Random Access Memory

**WiFi**: Wireless Fidelity

**XML**: eXtensible Markup Language

**YAML**: YAML Ain't Markup Language

**ZMP**: Zero Moment Point

## Mathematical Notation

**∇ (Nabla)**: Del operator, used in vector calculus

**θ (Theta)**: Commonly used for joint angles

**τ (Tau)**: Torque or time constant

**ω (Omega)**: Angular velocity

**φ (Phi)**: Phase angle or rotation angle

**λ (Lambda)**: Eigenvalue or wavelength

**σ (Sigma)**: Standard deviation or conductivity

**ρ (Rho)**: Density or correlation coefficient

**∂ (Partial)**: Partial derivative

**∫ (Integral)**: Integration operator

**∞ (Infinity)**: Mathematical concept of unbounded quantity

## Next Steps

This glossary serves as a reference throughout your study of humanoid robotics. As you progress through the textbook, you may want to revisit these definitions to deepen your understanding. Continue to the next section to learn about advanced topics and emerging trends in humanoid robotics.