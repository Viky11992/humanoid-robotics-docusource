# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites
- Ubuntu 22.04 LTS (recommended for ROS 2 compatibility)
- Node.js LTS (18.x or higher) for Docusaurus documentation
- Git
- GitHub account
- NVIDIA GPU with RTX capabilities (for Isaac Sim - minimum RTX 4070 Ti with 12GB VRAM recommended)
- ROS 2 Humble Hawksbill or Iron Irwini

## Initial Setup
1. Clone the repository:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. Install Docusaurus dependencies:
   ```bash
   cd my-website
   npm install
   ```

3. Verify the documentation setup:
   ```bash
   npm run start
   ```

## Setting Up ROS 2 Environment
1. Install ROS 2 (Humble Hawksbill):
   ```bash
   # Follow official ROS 2 installation guide for Ubuntu 22.04
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron/rpm.repos | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-iron-desktop
   ```

2. Source ROS 2 environment:
   ```bash
   source /opt/ros/iron/setup.bash
   ```

## Accessing Textbook Content
The textbook is organized into modules:
- `docs/intro/` - Introduction to Physical AI and embodied intelligence
- `docs/module-1-ros2/` - ROS 2 fundamentals and humanoid control
- `docs/module-2-simulation/` - Gazebo and Unity simulation environments
- `docs/module-3-isaac/` - NVIDIA Isaac platform for advanced robotics
- `docs/module-4-vla/` - Vision-Language-Action systems with conversational AI
- `docs/hardware-setup/` - Hardware requirements and setup guides

## Working with ROS 2 Examples
1. Each chapter includes ROS 2 code examples in Python
2. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/physical_ai_ws/src
   cd ~/physical_ai_ws
   colcon build
   source install/setup.bash
   ```

3. Run examples following the textbook instructions

## Simulation Setup
1. Install Gazebo Harmonic:
   ```bash
   sudo apt install ros-iron-gazebo-*
   ```

2. For NVIDIA Isaac Sim, follow the NVIDIA Omniverse setup instructions in the textbook

## Building and Deploying Documentation
1. Build the textbook site locally:
   ```bash
   npm run build
   ```

2. Test the build locally:
   ```bash
   npm run serve
   ```

3. Documentation deploys to GitHub Pages automatically when pushed to main branch

## Hardware Requirements
For full textbook experience, you'll need:
- **Workstation**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher, Intel Core i7 (13th Gen+) or AMD Ryzen 9, 64GB RAM DDR5
- **Edge Device**: NVIDIA Jetson Orin Nano (8GB) for deployment
- **Sensors**: Intel RealSense D435i for vision and depth data
- **Robot Platform**: Unitree Go2 Edu or similar for physical implementation

## Next Steps
- Start with the Introduction module to understand Physical AI fundamentals
- Progress through ROS 2 basics before moving to simulation
- Set up your development environment following the hardware requirements
- Work through examples progressively from basic to advanced humanoid capabilities