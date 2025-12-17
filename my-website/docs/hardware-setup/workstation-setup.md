---
sidebar_position: 1
title: Workstation Setup for Physical AI
---

# Workstation Setup for Physical AI

## Hardware Requirements Overview

This course is technically demanding. It sits at the intersection of three heavy computational loads:
- **Physics Simulation**: Isaac Sim/Gazebo
- **Visual Perception**: SLAM/Computer Vision
- **Generative AI**: LLMs/VLA models

Because the capstone involves a "Simulated Humanoid," the primary investment must be in **High-Performance Workstations**.

## Required Components

### GPU (The Critical Bottleneck)
- **Minimum**: NVIDIA RTX 4070 Ti (12GB VRAM)
- **Recommended**: RTX 3090 or 4090 (24GB VRAM) for smoother "Sim-to-Real" training
- **Why**: You need high VRAM to load USD (Universal Scene Description) assets for robots and environments, plus run VLA (Vision-Language-Action) models simultaneously

### CPU
- **Minimum**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **Why**: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive

### RAM
- **Minimum**: 32 GB DDR5 (will struggle with complex scenes)
- **Recommended**: 64 GB DDR5
- **Why**: Complex scene rendering and AI model loading require substantial memory

### Operating System
- **Required**: Ubuntu 22.04 LTS
- **Note**: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience

## Software Installation

### ROS 2 Installation
```bash
# Update packages
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Iron
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron/rpm.repos | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-iron-desktop
```

### Gazebo Installation
```bash
sudo apt install ros-iron-gazebo-*
```

### NVIDIA Isaac Sim Setup
1. Install NVIDIA Omniverse Launcher
2. Download Isaac Sim from NVIDIA Developer website
3. Follow the installation guide for your specific OS

## Alternative: Cloud-Based Workstation

If you don't have access to RTX-enabled workstations, you can use cloud instances:

### AWS Options
- **Instance Type**: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge
- **Software**: NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI)
- **Cost**: ~$1.50/hour (spot/on-demand mix)

### Cost Calculation Example
- Usage: 10 hours/week × 12 weeks = 120 hours
- Storage (EBS volumes): ~$25/quarter
- Total Cloud Bill: ~$205 per quarter

## Edge Device Setup

For the "Physical AI" deployment phase, you'll also need:

### The "Brain"
- **Device**: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
- **Role**: Industry standard for embodied AI. Students deploy ROS 2 nodes here to understand resource constraints.

### The "Eyes"
- **Device**: Intel RealSense D435i or D455
- **Role**: Provides RGB (Color) and Depth (Distance) data essential for VSLAM and Perception modules.

### The "Voice Interface"
- **Device**: Simple USB Microphone/Speaker array (e.g., ReSpeaker)
- **Role**: For the "Voice-to-Action" Whisper integration.

## Budget-Friendly Student Kit

### Complete Setup (~$700)
| Component | Model | Price |
|-----------|-------|-------|
| The Brain | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 |
| The Eyes | Intel RealSense D435i | $349 |
| The Ears | ReSpeaker USB Mic Array v2.0 | $69 |
| Power/Misc | SD Card (128GB) + Jumper Wires | $30 |
| **TOTAL** | | **~$700** |

## Troubleshooting Common Issues

### GPU Memory Errors
- **Symptom**: Simulation crashes during complex scenes
- **Solution**: Reduce scene complexity or increase swap space temporarily

### ROS 2 Connection Issues
- **Symptom**: Nodes cannot communicate across machines
- **Solution**: Check network configuration and ROS_DOMAIN_ID settings

### Isaac Sim Performance
- **Symptom**: Slow rendering or frame drops
- **Solution**: Update graphics drivers and verify RTX capabilities

## Next Steps

Continue to the next section to learn about Edge Computing Kits and deployment options.