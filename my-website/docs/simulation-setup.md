---
title: "Simulation Environment Setup Guide"
description: "A detailed guide for setting up and configuring robot simulation environments, with a focus on PyBullet."
sidbar_label: "Simulation Setup"
slug: /simulation-setup
position: 6
id: simulation-setup
---

# Simulation Environment Setup Guide

This guide provides detailed instructions for setting up and configuring robot simulation environments, with a primary focus on PyBullet, the chosen physics engine for basic to intermediate simulations in this textbook.

## 1. Prerequisites

Before setting up any simulation environment, ensure you have the following installed:

*   **Python 3.9+**: All simulation examples are developed using Python.
*   **pip (Python Package Installer)**: Used to install Python libraries.

## 2. PyBullet Installation

PyBullet is a versatile physics engine with a user-friendly Python API.

1.  **Install PyBullet and NumPy**:
    Open your terminal or command prompt and run the following command:
    ```bash
    pip install pybullet numpy
    ```
    *   `pybullet`: The core PyBullet library.
    *   `numpy`: A fundamental library for numerical computing in Python, often used with PyBullet for array manipulations.

2.  **Verify Installation**:
    You can quickly check if PyBullet is installed correctly by opening a Python interpreter and importing it:
    ```python
    import pybullet as p
    print("PyBullet installed successfully!")
    p.disconnect() # Disconnect if a connection was implicitly made
    ```
    If no errors occur, PyBullet is ready to use.

## 3. Basic PyBullet Simulation Structure

A typical PyBullet simulation involves the following steps:

1.  **Initialize the Physics Client**:
    ```python
    import pybullet as p
    physicsClient = p.connect(p.GUI) # p.GUI for graphical interface, p.DIRECT for non-graphical
    ```
    *   `p.GUI`: Starts a graphical user interface where you can visualize the simulation.
    *   `p.DIRECT`: Runs the simulation without a GUI, useful for faster computations or headless environments.

2.  **Set Up the Environment**:
    ```python
    p.setAdditionalSearchPath(p.getDataPath()) # Add path for built-in models like 'plane.urdf'
    p.setGravity(0, 0, -9.8) # Set gravity (x, y, z)
    planeId = p.loadURDF("plane.urdf") # Load a ground plane
    ```

3.  **Load or Create Robot Models**:
    Robots can be loaded from URDF (Unified Robot Description Format) or SDF (Simulation Description Format) files, or created programmatically using primitive shapes.
    ```python
    # Example: Loading a built-in URDF model
    robotId = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

    # Example: Creating a simple box (link)
    box_half_extents = [0.1, 0.1, 0.1]
    box_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents)
    box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
    box_body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=box_collision_shape,
                                 baseVisualShapeIndex=box_visual_shape, basePosition=[0, 0, 1])
    ```

4.  **Control Robot Joints (if applicable)**:
    You can control joints using position, velocity, or torque control modes.
    ```python
    # Example: Setting a joint to a target position
    joint_index = 0 # Assuming joint 0
    target_angle_rad = math.radians(45)
    p.setJointMotorControl2(robotId, joint_index, p.POSITION_CONTROL, targetPosition=target_angle_rad, force=500)
    ```

5.  **Run the Simulation Loop**:
    Advance the simulation step by step.
    ```python
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1/240.0) # Optional: sleep to visualize in real-time (if GUI is enabled)
    ```

6.  **Disconnect from Physics Server**:
    ```python
    p.disconnect()
    ```

## 4. Other Simulation Environments (Overview)

While PyBullet is our primary focus, other powerful simulation environments exist:

*   **MuJoCo**: Known for its advanced contact dynamics and ideal for model-based control and reinforcement learning. Requires a license (though free for academic use).
*   **Gazebo**: A very popular open-source 3D robot simulator, often integrated with ROS (Robot Operating System). Offers high-fidelity physics and graphics.
*   **ROS (Robot Operating System)**: A flexible framework for writing robot software. While not a simulator itself, it integrates well with simulators like Gazebo and provides tools for robot control, perception, and navigation.

Detailed setup for these environments is beyond the scope of this quick guide but can be explored in their respective documentation.

## 5. Next Steps

Refer to the specific chapters for code examples and detailed usage of these simulation environments in the context of physical AI and humanoid robotics.