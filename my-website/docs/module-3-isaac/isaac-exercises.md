---
sidebar_position: 5
title: Isaac ROS Exercises for Humanoid Robotics
---

# Isaac ROS Exercises for Humanoid Robotics

## Exercise 1: Isaac Sim Environment Setup for Humanoid Robots

### Objective
Set up Isaac Sim with a humanoid robot model and configure basic sensors for perception and navigation.

### Tasks
1. Install Isaac Sim using Omniverse
2. Create a humanoid robot model with appropriate URDF/USD format
3. Add camera, IMU, and depth sensors to the humanoid robot
4. Create a simulation environment with obstacles and navigation challenges
5. Verify the simulation runs correctly with stable humanoid physics

### Solution Template

#### 1. Basic Isaac Sim Setup Script for Humanoid Robotics
```python
# setup_humanoid_isaac_sim.py
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Configure simulation for humanoid robotics
config = {
    "headless": False,  # Set to True for headless operation during training
    "enable_cameras": True,
    "use_fabric": True,
    "carb_settings_path": "./settings.carb"
}

# Start simulation
simulation_app = SimulationApp(config)

def setup_humanoid_robot_environment():
    """Set up the humanoid robot simulation environment."""
    # Create world with appropriate scale for humanoid
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    from omni.isaac.core.objects import GroundPlane
    world.scene.add(GroundPlane(prim_path="/World/Ground", size=1000.0))

    # Get assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Could not find Isaac Sim assets. Please check your Isaac Sim installation.")
        return None

    # Add humanoid robot (using Isaac's built-in humanoid or import your own)
    humanoid_asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"

    # Add robot to stage
    add_reference_to_stage(
        prim_path="/World/Humanoid",
        reference_path=humanoid_asset_path
    )

    # Add the robot to the scene
    from omni.isaac.humanoid.robots import Humanoid
    humanoid_robot = world.scene.add(
        Humanoid(
            prim_path="/World/Humanoid",
            name="humanoid_robot",
            position=np.array([0, 0, 1.5]),  # Start above ground
            orientation=np.array([0, 0, 0, 1])
        )
    )

    # Add obstacles for navigation training
    obstacle1 = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Obstacle1",
            name="obstacle1",
            position=np.array([2.0, 0.0, 0.2]),
            size=0.4,
            color=np.array([0.5, 0.5, 0.5])  # Gray obstacle
        )
    )

    obstacle2 = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Obstacle2",
            name="obstacle2",
            position=np.array([3.0, 1.0, 0.2]),
            size=0.3,
            color=np.array([0.8, 0.2, 0.2])  # Red obstacle
        )
    )

    return world, humanoid_robot, [obstacle1, obstacle2]

def main():
    """Main function to run the humanoid simulation setup."""
    print("Setting up Isaac Sim environment for humanoid robotics...")

    result = setup_humanoid_robot_environment()
    if result is None:
        print("Failed to set up simulation environment")
        simulation_app.close()
        return

    world, humanoid, obstacles = result

    # Reset the world to initialize everything
    world.reset()

    print("Humanoid simulation environment created successfully!")
    print("Robot positioned at:", humanoid.get_world_poses()[0][0].tolist())

    # Run simulation for a few steps to verify setup
    for i in range(100):
        world.step(render=True)

        if i % 50 == 0:
            print(f"Simulation step {i} completed")

    print("Simulation setup verification complete!")

if __name__ == "__main__":
    main()
    simulation_app.close()
```

#### 2. Isaac Sim Configuration for Humanoid Robotics
```json
// config/humanoid_sim_config.json
{
  "simulation": {
    "headless": false,
    "enable_cameras": true,
    "use_fabric": true,
    "stage_units_in_meters": 1.0,
    "render_frequency": 60,
    "physics_frequency": 60,
    "physics_solver": "PhysX"
  },
  "robot": {
    "model_path": "/Isaac/Robots/Humanoid/humanoid_instanceable.usd",
    "start_position": [0, 0, 1.5],
    "start_orientation": [0, 0, 0, 1],
    "sensors": [
      {
        "type": "camera",
        "mount_point": "head",
        "resolution": [640, 480],
        "fov": 60
      },
      {
        "type": "imu",
        "mount_point": "torso",
        "update_rate": 100
      },
      {
        "type": "depth_camera",
        "mount_point": "head",
        "resolution": [640, 480],
        "fov": 60
      }
    ]
  },
  "environment": {
    "ground_plane": true,
    "objects": [
      {
        "type": "cube",
        "position": [2.0, 0.0, 0.2],
        "size": 0.4,
        "color": [0.5, 0.5, 0.5]
      },
      {
        "type": "cylinder",
        "position": [3.0, 1.0, 0.3],
        "radius": 0.2,
        "height": 0.6,
        "color": [0.8, 0.2, 0.2]
      }
    ]
  }
}
```

### Running the Exercise
```bash
# 1. Make sure Isaac Sim is installed and running via Omniverse
# 2. Run the setup script
python3 setup_humanoid_isaac_sim.py

# 3. Verify the simulation opens and shows the humanoid robot with sensors
# 4. Check that physics simulation is stable and robot maintains balance
```

## Exercise 2: Isaac ROS Perception Pipeline for Humanoid Navigation

### Objective
Create a perception pipeline using Isaac ROS packages specifically for humanoid navigation scenarios, including object detection and depth estimation.

### Tasks
1. Set up Isaac ROS Apriltag detection for landmark navigation
2. Configure Isaac ROS stereo processing for depth estimation
3. Create a perception node that processes sensor data for humanoid navigation
4. Validate the perception pipeline works correctly in simulation

### Solution Template

#### 1. Isaac ROS Launch File for Humanoid Perception
```xml
<!-- launch/humanoid_perception_pipeline.launch.xml -->
<launch>
  <!-- Isaac ROS AprilTag Detection for Landmark Navigation -->
  <node pkg="isaac_ros_apriltag" exec="isaac_ros_apriltag" name="apriltag_detector" output="screen">
    <param name="family" value="t36h11"/>
    <param name="max_tags" value="64"/>
    <param name="tile_size" value="0.032"/>
    <param name="tag_rows" value="6"/>
    <param name="tag_cols" value="6"/>
    <param name="quads_decimate" value="1.0"/>
    <param name="quads_blur" value="0.0"/>
    <param name="refine_edges" value="1"/>
    <param name="refine_decode" value="0"/>
    <param name="refine_pose" value="0"/>
    <param name="decode_sharpening" value="0.25"/>
    <param name="max_hamming" value="1"/>
    <param name="quad_contours" value="1"/>
    <param name="quad_decimate" value="1.0"/>
    <param name="min_tag_perimeter" value="3"/>
    <param name="min_side_length_pixels" value="6"/>
    <param name="error_correction_mode" value="0"/>
    <param name="black_border_bits" value="1"/>
    <param name="border_clip_distance" value="0.0"/>
    <param name="first_tag_id" value="0"/>
    <param name="min_marker_area" value="0.001"/>
    <param name="max_marker_area" value="1.0"/>
    <param name="publish_tf" value="true"/>
  </node>

  <!-- Isaac ROS Stereo Processing for Depth Estimation -->
  <node pkg="isaac_ros_stereo_image_proc" exec="isaac_ros_stereo_rectify_node" name="stereo_rectify" output="screen">
    <param name="left_namespace" value="left"/>
    <param name="right_namespace" value="right"/>
    <param name="use_color" value="true"/>
  </node>

  <!-- Isaac ROS Point Cloud to Laser Scan for Navigation -->
  <node pkg="isaac_ros_pointcloud_utils" exec="isaac_ros_pointcloud_to_laserscan_node" name="pointcloud_to_laserscan" output="screen">
    <param name="input_topic" value="/depth_registered/points"/>
    <param name="output_frame_id" value="base_link"/>
    <param name="scan_height" value="10"/>
    <param name="range_min" value="0.3"/>
    <param name="range_max" value="10.0"/>
    <param name="angle_min" value="-1.5708"/> <!-- -90 degrees in radians -->
    <param name="angle_max" value="1.5708"/>  <!-- 90 degrees in radians -->
  </node>

  <!-- Custom Humanoid Perception Node -->
  <node pkg="humanoid_navigation" exec="humanoid_perception_node.py" name="humanoid_perception" output="screen">
    <param name="camera_topic" value="/camera/image_rect_color"/>
    <param name="depth_topic" value="/depth_registered/image_rect"/>
    <param name="detection_topic" value="/apriltag_detector/detections"/>
    <param name="pointcloud_topic" value="/points_map"/>
    <param name="navigation_topic" value="/humanoid/navigation_targets"/>
  </node>
</launch>
```

#### 2. Humanoid Perception Processing Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

class HumanoidPerceptionNode(Node):
    def __init__(self):
        super().__init__('humanoid_perception_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Get parameters
        self.camera_topic = self.get_parameter_or('camera_topic', '/camera/image_rect_color').value
        self.depth_topic = self.get_parameter_or('depth_topic', '/depth_registered/image_rect').value
        self.detection_topic = self.get_parameter_or('detection_topic', '/apriltag_detector/detections').value
        self.pointcloud_topic = self.get_parameter_or('pointcloud_topic', '/points_map').value
        self.navigation_topic = self.get_parameter_or('navigation_topic', '/humanoid/navigation_targets').value

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.navigation_target_pub = self.create_publisher(PoseStamped, self.navigation_topic, 10)
        self.obstacle_map_pub = self.create_publisher(LaserScan, '/humanoid/obstacle_scan', 10)
        self.perception_status_pub = self.create_publisher(String, '/humanoid/perception_status', 10)

        # Perception data storage
        self.latest_image = None
        self.latest_depth = None
        self.latest_detections = []
        self.obstacle_distances = []

        # Navigation parameters
        self.safe_distance = 0.8  # meters
        self.detection_confidence_threshold = 0.7
        self.landmark_detection_enabled = True

        # Timer for perception processing
        self.perception_timer = self.create_timer(0.1, self.perception_processing)  # 10 Hz

        self.get_logger().info('Humanoid Perception Node initialized')

    def image_callback(self, msg):
        """Process camera image for humanoid perception."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def depth_callback(self, msg):
        """Process depth image for distance estimation."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth: {e}')

    def detection_callback(self, msg):
        """Process landmark detections for navigation."""
        self.latest_detections = msg.detections
        self.get_logger().info(f'Detected {len(msg.detections)} landmarks')

    def pointcloud_callback(self, msg):
        """Process point cloud for obstacle detection."""
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        if points:
            points = np.array(points)

            # Calculate distances to robot (assuming robot at origin)
            xy_distances = np.linalg.norm(points[:, :2], axis=1)
            self.obstacle_distances = xy_distances.tolist()

    def perception_processing(self):
        """Main perception processing loop."""
        perception_status = String()

        # Process landmark detections for navigation
        navigation_targets = self.process_landmark_detections()

        # Process obstacle data for safe navigation
        safe_path_available = self.check_safe_navigation_path()

        # Update perception status
        if navigation_targets:
            perception_status.data = "LANDMARK_NAVIGATION_AVAILABLE"
        elif safe_path_available:
            perception_status.data = "SAFE_PATH_DETECTED"
        else:
            perception_status.data = "PERCEPTION_PROCESSING"

        # Publish perception results
        self.perception_status_pub.publish(perception_status)

        if navigation_targets and len(navigation_targets) > 0:
            # Publish first navigation target
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = 'map'
            target_msg.pose.position.x = navigation_targets[0][0]
            target_msg.pose.position.y = navigation_targets[0][1]
            target_msg.pose.position.z = 0.0
            target_msg.pose.orientation.w = 1.0

            self.navigation_target_pub.publish(target_msg)

        # Publish obstacle scan
        self.publish_obstacle_scan()

    def process_landmark_detections(self):
        """Process landmark detections to identify navigation targets."""
        navigation_targets = []

        for detection in self.latest_detections:
            if len(detection.results) > 0:
                # Get the first result's confidence
                confidence = detection.results[0].score

                if confidence > self.detection_confidence_threshold:
                    # Calculate 3D position from 2D detection and depth
                    center_x = int(detection.bbox.center.x)
                    center_y = int(detection.bbox.center.y)

                    if (self.latest_depth is not None and
                        0 <= center_y < self.latest_depth.shape[0] and
                        0 <= center_x < self.latest_depth.shape[1]):

                        depth_value = self.latest_depth[center_y, center_x]

                        if depth_value > 0 and depth_value < 10.0:  # Valid depth
                            # Convert pixel coordinates + depth to 3D position
                            # This is simplified - in practice, use camera intrinsics
                            target_x = (center_x - 320) * depth_value / 500  # Focal length approximation
                            target_y = (center_y - 240) * depth_value / 500

                            navigation_targets.append((target_x, target_y, depth_value))

        return navigation_targets

    def check_safe_navigation_path(self):
        """Check if there's a safe path based on obstacle detection."""
        if not self.obstacle_distances:
            return True  # No obstacles detected, path is safe

        # Check if any obstacles are within safe distance
        close_obstacles = [d for d in self.obstacle_distances if 0 < d < self.safe_distance]

        return len(close_obstacles) == 0  # Safe if no close obstacles

    def publish_obstacle_scan(self):
        """Publish processed obstacle data as laser scan."""
        if not self.obstacle_distances:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -math.pi / 2  # -90 degrees
        scan_msg.angle_max = math.pi / 2   # 90 degrees
        scan_msg.angle_increment = math.pi / 180  # 1 degree increments
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Create ranges array (simplified - in practice, would map 3D points to 2D scan angles)
        num_angles = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [float('inf')] * num_angles

        # Process obstacle points and map to scan ranges
        for distance in self.obstacle_distances:
            if 0.1 <= distance <= 10.0:
                # Calculate angle (simplified - assuming points are in front of robot)
                # In practice, you'd calculate proper angle from point coordinates
                angle_idx = int((0 - scan_msg.angle_min) / scan_msg.angle_increment)
                if 0 <= angle_idx < len(scan_msg.ranges):
                    scan_msg.ranges[angle_idx] = min(scan_msg.ranges[angle_idx], float(distance))

        self.obstacle_map_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Exercise
```bash
# 1. Build your ROS 2 workspace with the perception package
cd ~/isaac_ros_ws
colcon build --packages-select humanoid_navigation
source install/setup.bash

# 2. Launch the perception pipeline
ros2 launch humanoid_navigation humanoid_perception_pipeline.launch.xml

# 3. Test with Isaac Sim running the humanoid environment
# 4. Verify perception data is being published to the navigation stack
```

## Exercise 3: Isaac ROS Navigation Integration for Humanoid Robots

### Objective
Integrate Isaac ROS perception with Nav2 for humanoid-specific navigation, incorporating balance-aware path planning and execution.

### Tasks
1. Set up Nav2 with Isaac ROS perception integration
2. Configure costmap layers to utilize 3D perception data
3. Create a humanoid-specific navigation node that uses perception and balance data
4. Test navigation in Isaac Sim with humanoid robot model

### Solution Template

#### 1. Humanoid-Specific Navigation Configuration
```yaml
# config/humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /humanoid/obstacle_scan  # Use Isaac ROS processed scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_feedback_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_are_error_recovery_attempts_exceeded_condition_bt_node
    - nav2_would_a_controller_recovery_help_condition_bt_node
    - nav2_would_a_path_planner_recovery_help_condition_bt_node
    - nav2_would_a_spin_recovery_help_condition_bt_node
    - nav2_costmap_filter_info_message_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller with Isaac integration
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICtrl"
      debug_visualizations: true
      rate_limiting: true
      min_speed_xy: 0.05  # Slower for stability
      max_speed_xy: 0.3   # Limited for balance
      min_speed_theta: 0.1
      max_speed_theta: 0.5
      # Isaac-enhanced parameters
      step_size: 0.15     # Smaller steps for humanoid stability
      balance_margin: 0.8 # Required balance margin from Isaac
      perception_timeout: 1.0  # Timeout for Isaac perception data

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6  # Larger for humanoid (wider stance)
      height: 6
      resolution: 0.05  # Higher resolution for precision
      robot_radius: 0.35  # Larger for humanoid footprint
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid/obstacle_scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0  # Extended range for Isaac perception
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0  # Extended range for safety
          obstacle_min_range: 0.0

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7  # Larger for humanoid safety

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.35  # Larger for humanoid
      resolution: 0.05    # Higher resolution
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /humanoid/obstacle_scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7  # Larger for humanoid safety

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size: 0.05  # Finer resolution for humanoid navigation
      min_distance_from_robot: 0.5  # Keep path further from obstacles for stability
```

#### 2. Isaac-Enhanced Navigation Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
import numpy as np

class IsaacEnhancedNavigator(Node):
    def __init__(self):
        super().__init__('isaac_enhanced_navigator')

        # Subscribers for Isaac-enhanced perception and balance
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/humanoid/obstacle_scan',
            self.scan_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.balance_state_pub = self.create_publisher(Float64, '/balance_state', 10)
        self.navigation_status_pub = self.create_publisher(String, '/navigation_status', 10)

        # Isaac-enhanced navigation parameters
        self.step_frequency = 1.0  # Steps per second
        self.max_step_length = 0.3  # Maximum step length (m)
        self.balance_threshold = 0.1  # Balance error threshold (m)
        self.turn_threshold = 0.2     # Turn threshold (rad)
        self.safety_margin = 0.5      # Safety margin for obstacles (m)

        # Robot state
        self.current_pose = None
        self.current_goal = None
        self.current_imu = None
        self.obstacle_ranges = []
        self.in_motion = False
        self.balance_ok = True

        # Timer for Isaac-enhanced navigation control
        self.nav_timer = self.create_timer(0.05, self.navigation_control)  # 20 Hz

        self.get_logger().info('Isaac Enhanced Navigator initialized')

    def odom_callback(self, msg):
        """Receive robot odometry."""
        self.current_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Receive IMU data for Isaac balance monitoring."""
        self.current_imu = msg
        # Check balance state based on IMU data
        self.balance_ok = self.check_balance_from_imu()

    def scan_callback(self, msg):
        """Receive processed obstacle scan from Isaac ROS."""
        self.obstacle_ranges = list(msg.ranges)

    def goal_callback(self, msg):
        """Receive navigation goal."""
        self.current_goal = msg.pose
        self.get_logger().info(f'Received navigation goal: x={msg.pose.position.x}, y={msg.pose.position.y}')

    def navigation_control(self):
        """Main Isaac-enhanced navigation control loop."""
        if not all([self.current_pose, self.current_goal, self.current_imu]):
            return

        # Check for obstacles
        min_obstacle_dist = min([r for r in self.obstacle_ranges if 0 < r < float('inf')], default=float('inf'))

        # Check balance state from Isaac-enhanced IMU processing
        if not self.balance_ok:
            # Emergency stop if balance is compromised
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.in_motion = False

            status_msg = String()
            status_msg.data = "BALANCE_COMPROMISED_EMERGENCY_STOP"
            self.navigation_status_pub.publish(status_msg)

            self.get_logger().warn('Balance compromised - emergency stop activated')
            return

        # Calculate distance and heading to goal
        dx = self.current_goal.position.x - self.current_pose.position.x
        dy = self.current_goal.position.y - self.current_pose.position.y
        distance_to_goal = np.sqrt(dx*dx + dy*dy)

        # Calculate desired heading
        desired_heading = np.arctan2(dy, dx)

        # Get current orientation from pose
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate heading error
        heading_error = self.normalize_angle(desired_heading - current_yaw)

        # Isaac-enhanced navigation commands
        cmd_vel = Twist()

        if distance_to_goal < 0.2:  # Close to goal
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.in_motion = False

            status_msg = String()
            status_msg.data = "GOAL_REACHED"
            self.navigation_status_pub.publish(status_msg)

            self.get_logger().info('Goal reached!')
        elif min_obstacle_dist < self.safety_margin:  # Obstacle too close
            # Stop and plan alternative route
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.in_motion = False

            status_msg = String()
            status_msg.data = "OBSTACLE_AVOIDANCE_ACTIVE"
            self.navigation_status_pub.publish(status_msg)

            self.get_logger().warn(f'Obstacle detected at {min_obstacle_dist:.2f}m - stopping')
        else:
            # Calculate Isaac-enhanced navigation commands
            linear_vel = self.calculate_balanced_linear_velocity(distance_to_goal)
            angular_vel = self.calculate_balanced_angular_velocity(heading_error)

            cmd_vel.linear.x = max(0.0, min(linear_vel, 0.3))  # Limit to 0.3 m/s for stability
            cmd_vel.angular.z = max(-0.5, min(angular_vel, 0.5))  # Limit angular velocity

            self.in_motion = True

        # Publish navigation commands
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish balance state
        balance_msg = Float64()
        balance_msg.data = 1.0 if self.balance_ok else 0.0
        self.balance_state_pub.publish(balance_msg)

        # Publish navigation status
        if self.in_motion:
            status_msg = String()
            status_msg.data = "NAVIGATING_TO_GOAL"
            self.navigation_status_pub.publish(status_msg)

    def check_balance_from_imu(self):
        """Check balance state using Isaac-enhanced IMU processing."""
        if not self.current_imu:
            return False

        # Extract angular velocity and linear acceleration
        ang_vel = np.array([
            self.current_imu.angular_velocity.x,
            self.current_imu.angular_velocity.y,
            self.current_imu.angular_velocity.z
        ])

        lin_acc = np.array([
            self.current_imu.linear_acceleration.x,
            self.current_imu.linear_acceleration.y,
            self.current_imu.linear_acceleration.z
        ])

        # Check for excessive angular velocity (indicating loss of balance)
        ang_vel_magnitude = np.linalg.norm(ang_vel)
        if ang_vel_magnitude > 1.0:  # Threshold in rad/s
            return False

        # Check for unusual linear acceleration (indicating falling)
        gravity = 9.81
        acc_magnitude = np.linalg.norm(lin_acc)
        if abs(acc_magnitude - gravity) > 3.0:  # Deviation from expected gravity
            return False

        return True

    def calculate_balanced_linear_velocity(self, distance_to_goal):
        """Calculate linear velocity considering Isaac-enhanced balance constraints."""
        # Maximum velocity when far from goal, reduce as approaching
        max_vel = 0.2  # Reduced for humanoid stability
        approach_vel = 0.05  # Even slower when approaching goal

        if distance_to_goal < 0.5:
            # Slow down when approaching goal
            vel = approach_vel
        else:
            # Normal speed when far from goal
            vel = max_vel

        # Reduce velocity if balance margin is low
        balance_factor = 0.8 if self.balance_ok else 0.3
        return vel * balance_factor

    def calculate_balanced_angular_velocity(self, heading_error):
        """Calculate angular velocity considering Isaac-enhanced balance constraints."""
        # Use PID-like control for heading with balance awareness
        kp = 0.8  # proportional gain
        max_angular_vel = 0.3  # Reduced for balance (was 0.5)

        angular_vel = kp * heading_error
        angular_vel = max(-max_angular_vel, min(angular_vel, max_angular_vel))

        # Reduce angular velocity if balance is marginal
        balance_factor = 0.7 if self.balance_ok else 0.3
        return angular_vel * balance_factor

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = IsaacEnhancedNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Exercise
```bash
# 1. Create the launch file for integrated navigation
# launch/integrated_humanoid_navigation.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('humanoid_navigation'),
        'config',
        'humanoid_nav2_params.yaml'
    ])

    return LaunchDescription([
        # Isaac ROS perception nodes
        Node(
            package='isaac_ros_stereo_image_proc',
            executable='isaac_ros_stereo_rectify_node',
            name='stereo_rectify',
            parameters=[{
                'left_namespace': 'left',
                'right_namespace': 'right',
                'use_color': True,
            }]
        ),

        Node(
            package='isaac_ros_pointcloud_utils',
            executable='isaac_ros_pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'input_topic': '/depth_registered/points',
                'output_frame_id': 'base_link',
                'scan_height': 10,
                'range_min': 0.3,
                'range_max': 10.0,
                'angle_min': -1.5708,  # -90 degrees
                'angle_max': 1.5708,   # 90 degrees
            }]
        ),

        # Isaac-enhanced navigation node
        Node(
            package='humanoid_navigation',
            executable='isaac_enhanced_navigator',
            name='isaac_enhanced_navigator',
            parameters=[config_path],
            output='screen'
        )
    ])

# 2. Build and run the integrated system
cd ~/isaac_ros_ws
colcon build --packages-select humanoid_navigation
source install/setup.bash

# 3. Launch the integrated system
ros2 launch humanoid_navigation integrated_humanoid_navigation.launch.py

# 4. Test with Isaac Sim humanoid environment running
# 5. Send navigation goals via RViz or command line
```

## Exercise 4: Isaac Sim Reinforcement Learning for Humanoid Locomotion

### Objective
Create an Isaac Sim reinforcement learning environment for training humanoid locomotion policies with Isaac Lab integration.

### Tasks
1. Set up Isaac Sim for RL training with humanoid robot
2. Create reward functions for stable bipedal locomotion
3. Implement curriculum learning for progressive skill acquisition
4. Train a basic walking policy using Isaac Lab

### Solution Template

#### 1. Isaac Sim RL Environment for Humanoid Locomotion
```python
# humanoid_locomotion_env.py
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.torch.maths import torch
import numpy as np

# Simulation configuration for RL training
CONFIG = {
    "headless": True,  # Enable for training, disable for visualization
    "enable_cameras": False,
    "use_fabric": True,
    "enable_scene_query_support": True
}

simulation_app = SimulationApp(CONFIG)

class HumanoidLocomotionEnv:
    def __init__(self):
        # Initialize Isaac Sim world for RL
        self.world = World(stage_units_in_meters=1.0,
                          physics_dt=1.0/60.0,
                          rendering_dt=1.0/60.0)

        # Add ground plane
        from omni.isaac.core.objects import GroundPlane
        self.world.scene.add(GroundPlane(prim_path="/World/Ground", size=1000.0))

        # Initialize humanoid robot for RL
        self.setup_humanoid_robot()

        # RL parameters
        self.episode_length = 500  # steps per episode
        self.current_step = 0
        self.episode_reward = 0.0

        # Action and observation spaces
        self.action_dim = 21  # Example: 21 DOF humanoid
        self.observation_dim = 48  # Example: joint positions, velocities, etc.

        # Locomotion goals
        self.target_velocity = 0.5  # m/s target forward velocity
        self.max_episode_distance = 10.0  # maximum distance per episode

    def setup_humanoid_robot(self):
        """Set up the humanoid robot for RL training."""
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Could not find Isaac Sim assets. Please check your Isaac Sim installation.")

        # Load humanoid asset
        humanoid_asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
        add_reference_to_stage(
            prim_path="/World/Humanoid",
            reference_path=humanoid_asset_path
        )

        # Add humanoid to scene
        from omni.isaac.humanoid.robots import Humanoid
        self.humanoid = self.world.scene.add(
            Humanoid(
                prim_path="/World/Humanoid",
                name="humanoid_robot",
                position=np.array([0, 0, 1.5]),
                orientation=np.array([0, 0, 0, 1])
            )
        )

        # Create articulation view for control
        self.humanoid_view = ArticulationView(
            prim_path_regex="/World/Humanoid/.*",
            name="humanoid_view"
        )
        self.world.scene.add(self.humanoid_view)

    def reset(self):
        """Reset the environment to initial state for RL."""
        self.world.reset()
        self.current_step = 0
        self.episode_reward = 0.0
        self.distance_traveled = 0.0

        # Randomize initial state for robust training
        initial_pos = np.array([0, 0, 1.5]) + np.random.uniform(-0.1, 0.1, 3)
        initial_rot = np.array([0, 0, 0, 1])  # Keep upright initially

        # Apply initial state
        self.humanoid.set_world_poses(
            positions=torch.tensor([initial_pos]),
            orientations=torch.tensor([initial_rot])
        )

        # Reset joint positions to neutral
        neutral_positions = torch.zeros((1, self.humanoid_view.num_dof))
        self.humanoid_view.set_joint_positions(neutral_positions)

        return self.get_observation()

    def step(self, action):
        """Execute one step in the RL environment."""
        # Apply action to humanoid robot
        self.apply_action(action)

        # Step simulation
        self.world.step(render=False)  # Don't render during training

        # Get observation
        obs = self.get_observation()

        # Calculate reward based on locomotion performance
        reward = self.calculate_locomotion_reward()

        # Check termination conditions
        done = self.check_termination()

        # Update step counter and distance traveled
        self.current_step += 1
        self.episode_reward += reward

        # Update distance traveled
        root_pos, _ = self.humanoid.get_world_poses()
        self.distance_traveled = abs(root_pos[0].item())  # X-axis displacement

        return obs, reward, done, {}

    def apply_action(self, action):
        """Apply action to the humanoid robot."""
        # Convert action to joint commands
        # This is a simplified example - real implementation would use more sophisticated control

        if isinstance(action, np.ndarray):
            action_tensor = torch.tensor(action, dtype=torch.float32).unsqueeze(0)
        else:
            action_tensor = action.unsqueeze(0)

        # Apply joint position targets (PD control would be more realistic)
        current_positions = self.humanoid_view.get_joint_positions()
        target_positions = current_positions + action_tensor * 0.01  # Small delta for stability

        self.humanoid_view.set_joint_positions(target_positions)

    def get_observation(self):
        """Get current observation from the environment."""
        # Get joint positions and velocities
        joint_pos = self.humanoid_view.get_joint_positions().squeeze().cpu().numpy()
        joint_vel = self.humanoid_view.get_joint_velocities().squeeze().cpu().numpy()

        # Get root pose and velocity
        root_pos, root_quat = self.humanoid.get_world_poses()
        root_lin_vel, root_ang_vel = self.humanoid.get_velocities()

        # Get IMU-like data (simplified)
        # In practice, you'd use actual IMU sensor data
        roll, pitch, yaw = self.quaternion_to_euler(root_quat.squeeze().cpu().numpy())
        imu_data = np.array([roll, pitch, yaw,
                            root_ang_vel[0, 0].item(), root_ang_vel[0, 1].item(), root_ang_vel[0, 2].item()])

        # Pack observation vector
        obs = np.concatenate([
            joint_pos,           # Joint positions
            joint_vel,           # Joint velocities
            root_lin_vel[0].cpu().numpy(),  # Root linear velocity
            imu_data             # IMU data (orientation + angular velocity)
        ])

        return obs

    def calculate_locomotion_reward(self):
        """Calculate reward for bipedal locomotion."""
        # Get current state
        root_pos, root_quat = self.humanoid.get_world_poses()
        root_lin_vel, root_ang_vel = self.humanoid.get_velocities()

        # Reward for forward movement
        forward_vel = root_lin_vel[0, 0].item()  # X-axis velocity
        forward_reward = forward_vel * 10.0  # Weight for forward movement

        # Penalty for falling (based on height)
        height = root_pos[0, 2].item()  # Z-axis position (height)
        fall_penalty = max(0, (0.8 - height) * 50.0)  # Strong penalty for falling below 0.8m

        # Reward for upright posture
        target_quat = np.array([0, 0, 0, 1])  # Upright orientation
        quat_diff = self.quaternion_difference(root_quat.squeeze().cpu().numpy(), target_quat)
        upright_reward = max(0, (1.0 - quat_diff) * 5.0)

        # Penalty for excessive joint velocities (energy efficiency)
        joint_vel = self.humanoid_view.get_joint_velocities()
        energy_penalty = torch.sum(joint_vel.abs()).item() * 0.01

        # Small reward for maintaining contact with ground (realistic walking)
        # Simplified: check if feet are approximately at ground level
        left_foot_pos, _ = self.humanoid.get_link_poses(link_name="left_foot")
        right_foot_pos, _ = self.humanoid.get_link_poses(link_name="right_foot")

        ground_contact_reward = 0.0
        if left_foot_pos is not None:
            left_height = left_foot_pos[0, 2].item()
            ground_contact_reward += max(0, (0.1 - abs(left_height)) * 2.0)
        if right_foot_pos is not None:
            right_height = right_foot_pos[0, 2].item()
            ground_contact_reward += max(0, (0.1 - abs(right_height)) * 2.0)

        # Total reward
        total_reward = (forward_reward - fall_penalty + upright_reward - energy_penalty
                       + ground_contact_reward)

        # Clamp reward to reasonable range
        total_reward = max(-10.0, min(total_reward, 10.0))

        return total_reward

    def check_termination(self):
        """Check if episode should terminate."""
        # Terminate if fallen
        root_pos, _ = self.humanoid.get_world_poses()
        height = root_pos[0, 2].item()
        if height < 0.5:  # Fallen below 0.5m
            return True

        # Terminate if maximum distance reached
        if self.distance_traveled > self.max_episode_distance:
            return True

        # Terminate if episode length exceeded
        if self.current_step >= self.episode_length:
            return True

        return False

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles."""
        # Simplified conversion - in practice, use more robust method
        w, x, y, z = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_difference(self, q1, q2):
        """Calculate difference between two quaternions."""
        # Dot product (measure of similarity)
        dot_product = np.dot(q1, q2)
        # Return 1 - |dot_product| as a measure of difference (0 = identical, 1 = opposite)
        return 1.0 - abs(dot_product)

def main():
    """Main RL training loop example."""
    env = HumanoidLocomotionEnv()

    print("Starting Isaac Sim RL environment for humanoid locomotion...")

    # Example training loop (without actual RL algorithm)
    for episode in range(10):
        obs = env.reset()
        episode_reward = 0.0
        step_count = 0

        print(f"Starting episode {episode}")

        while True:
            # Random action for demonstration (replace with RL policy)
            action = np.random.uniform(-1, 1, env.action_dim)

            obs, reward, done, info = env.step(action)
            episode_reward += reward
            step_count += 1

            if done:
                print(f"Episode {episode}: Steps={step_count}, Total reward={episode_reward:.2f}, Distance={env.distance_traveled:.2f}m")
                break

        print(f"Episode {episode} completed with reward: {episode_reward:.2f}")

    print("RL environment test completed!")
    simulation_app.close()

if __name__ == '__main__':
    main()
```

## Exercise 5: Isaac ROS Manipulation for Humanoid Robots

### Objective
Create a manipulation pipeline using Isaac ROS for humanoid robot arm control and object interaction.

### Tasks
1. Set up Isaac ROS manipulation nodes
2. Configure Isaac ROS 3D object detection and pose estimation
3. Create grasp planning pipeline using Isaac's physics understanding
4. Integrate with humanoid arm control

### Solution Template

#### 1. Isaac ROS Manipulation Pipeline
```xml
<!-- launch/humanoid_manipulation.launch.xml -->
<launch>
  <!-- Isaac ROS 3D Object Detection -->
  <node pkg="isaac_ros_detectnet" exec="isaac_ros_detectnet" name="object_detector" output="screen">
    <param name="model_name" value="resnet34_planar_bottle"/>
    <param name="input_topic" value="/camera/image_rect_color"/>
    <param name="output_topic" value="/object_detector/detections"/>
    <param name="confidence_threshold" value="0.7"/>
    <param name="publish_topic" value="true"/>
  </node>

  <!-- Isaac ROS 3D Pose Estimation -->
  <node pkg="isaac_ros_pose_estimators" exec="isaac_ros_posenet" name="pose_estimator" output="screen">
    <param name="input_topic" value="/camera/image_rect_color"/>
    <param name="output_topic" value="/pose_estimator/poses"/>
  </node>

  <!-- Isaac ROS Point Cloud Processing -->
  <node pkg="isaac_ros_pointcloud_utils" exec="isaac_ros_image_to_pcl_node" name="image_to_pcl" output="screen">
    <param name="image_topic" value="/camera/image_rect_color"/>
    <param name="depth_topic" value="/camera/depth/image_rect_raw"/>
    <param name="pcl_topic" value="/camera/points"/>
    <param name="camera_info_topic" value="/camera/camera_info"/>
  </node>

  <!-- Isaac ROS Manipulation Pipeline -->
  <node pkg="humanoid_manipulation" exec="manipulation_pipeline.py" name="manipulation_pipeline" output="screen">
    <param name="camera_topic" value="/camera/image_rect_color"/>
    <param name="depth_topic" value="/camera/depth/image_rect_raw"/>
    <param name="detection_topic" value="/object_detector/detections"/>
    <param name="pose_topic" value="/pose_estimator/poses"/>
    <param name="pointcloud_topic" value="/camera/points"/>
    <param name="arm_command_topic" value="/humanoid/left_arm/command"/>
  </node>
</launch>
```

#### 2. Manipulation Pipeline Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge

class HumanoidManipulationPipeline(Node):
    def __init__(self):
        super().__init__('humanoid_manipulation_pipeline')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Get parameters
        self.camera_topic = self.get_parameter_or('camera_topic', '/camera/image_rect_color').value
        self.depth_topic = self.get_parameter_or('depth_topic', '/camera/depth/image_rect_raw').value
        self.detection_topic = self.get_parameter_or('detection_topic', '/object_detector/detections').value
        self.pose_topic = self.get_parameter_or('pose_topic', '/pose_estimator/poses').value
        self.pointcloud_topic = self.get_parameter_or('pointcloud_topic', '/camera/points').value
        self.arm_command_topic = self.get_parameter_or('arm_command_topic', '/humanoid/left_arm/command').value

        # Subscribers
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.detection_sub = self.create_subscription(Detection2DArray, self.detection_topic, self.detection_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)

        # Publishers
        self.grasp_pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.manipulation_status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # Internal state
        self.latest_image = None
        self.latest_depth = None
        self.latest_detections = []
        self.latest_poses = []
        self.latest_pointcloud = None

        # Manipulation state
        self.target_object = None
        self.grasp_candidate = None
        self.manipulation_active = False

        # Timer for manipulation processing
        self.manip_timer = self.create_timer(0.5, self.manipulation_processing)

        self.get_logger().info('Humanoid Manipulation Pipeline initialized')

    def image_callback(self, msg):
        """Process camera image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def depth_callback(self, msg):
        """Process depth image."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth: {e}')

    def detection_callback(self, msg):
        """Process object detections."""
        self.latest_detections = msg.detections

    def pose_callback(self, msg):
        """Process pose estimations."""
        self.latest_poses.append(msg)

    def pointcloud_callback(self, msg):
        """Process point cloud for 3D object positioning."""
        self.latest_pointcloud = msg

    def manipulation_processing(self):
        """Main manipulation processing loop."""
        if not all([self.latest_image, self.latest_detections, self.latest_pointcloud]):
            return

        # Process detections to find manipulable objects
        manipulable_objects = self.identify_manipulable_objects()

        if manipulable_objects:
            # Select target object (for now, pick the closest one)
            self.target_object = self.select_target_object(manipulable_objects)

            # Plan grasp for target object
            grasp_pose = self.plan_grasp_for_object(self.target_object)

            if grasp_pose:
                self.grasp_candidate = grasp_pose
                self.grasp_pose_pub.publish(grasp_pose)

                # Publish manipulation status
                status_msg = String()
                status_msg.data = f"GRASP_PLANNED_FOR_OBJECT_{self.target_object['class']}"
                self.manipulation_status_pub.publish(status_msg)

                self.get_logger().info(f'Grasp planned for {self.target_object["class"]} at {grasp_pose.pose.position}')

                # Activate manipulation mode
                self.manipulation_active = True
            else:
                self.get_logger().warn(f'Could not plan grasp for {self.target_object["class"]}')
        else:
            # No manipulable objects found
            status_msg = String()
            status_msg.data = "NO_MANIPULABLE_OBJECTS_FOUND"
            self.manipulation_status_pub.publish(status_msg)

            self.manipulation_active = False

    def identify_manipulable_objects(self):
        """Identify objects that can be manipulated by the humanoid."""
        if not self.latest_detections:
            return []

        manipulable_objects = []

        for detection in self.latest_detections:
            if len(detection.results) > 0:
                # Check if detection is of a manipulable object class
                # This is simplified - in practice, you'd have specific class IDs for manipulable objects
                class_id = detection.results[0].id
                confidence = detection.results[0].score

                # For demonstration, assume certain class IDs are manipulable
                # (e.g., bottles, cans, boxes, cups)
                if class_id in [40, 41, 42, 43, 44, 45, 46, 47, 48, 49] and confidence > 0.7:
                    # Get 3D position from bounding box center and depth
                    center_x = int(detection.bbox.center.x)
                    center_y = int(detection.bbox.center.y)

                    if (self.latest_depth is not None and
                        0 <= center_y < self.latest_depth.shape[0] and
                        0 <= center_x < self.latest_depth.shape[1]):

                        depth_value = self.latest_depth[center_y, center_x]

                        if 0.1 < depth_value < 2.0:  # Valid depth range for manipulation
                            # Convert pixel coordinates + depth to 3D position
                            # This is simplified - use actual camera intrinsics in practice
                            pos_x = (center_x - 320) * depth_value / 500  # Focal length approx
                            pos_y = (center_y - 240) * depth_value / 500
                            pos_z = depth_value

                            object_info = {
                                'class': self.get_coco_class_name(class_id),
                                'position': (pos_x, pos_y, pos_z),
                                'confidence': confidence,
                                'bbox': detection.bbox,
                                'detection': detection
                            }

                            manipulable_objects.append(object_info)

        return manipulable_objects

    def select_target_object(self, objects):
        """Select the most appropriate target object."""
        if not objects:
            return None

        # For now, select the closest object
        # In practice, you might consider other factors like object value, ease of grasp, etc.
        closest_obj = min(objects, key=lambda obj: np.sqrt(obj['position'][0]**2 + obj['position'][1]**2))
        return closest_obj

    def plan_grasp_for_object(self, object_info):
        """Plan a grasp pose for the given object."""
        if not object_info or not self.latest_pointcloud:
            return None

        # Convert object position to PoseStamped
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'camera_link'  # or appropriate frame

        # Set position to object location
        grasp_pose.pose.position.x = float(object_info['position'][0])
        grasp_pose.pose.position.y = float(object_info['position'][1])
        grasp_pose.pose.position.z = float(object_info['position'][2])

        # For now, set a simple grasp orientation (overhead grasp)
        # In practice, this would use Isaac's grasp planning algorithms
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = 0.707  # 90-degree pitch for overhead grasp
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 0.707

        return grasp_pose

    def get_coco_class_name(self, class_id):
        """Get COCO dataset class name for the given class ID."""
        coco_classes = {
            1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane',
            6: 'bus', 7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light',
            11: 'fire hydrant', 13: 'stop sign', 14: 'parking meter', 15: 'bench',
            16: 'bird', 17: 'cat', 18: 'dog', 19: 'horse', 20: 'sheep',
            21: 'cow', 22: 'elephant', 23: 'bear', 24: 'zebra', 25: 'giraffe',
            27: 'backpack', 28: 'umbrella', 31: 'handbag', 32: 'tie', 33: 'suitcase',
            34: 'frisbee', 35: 'skis', 36: 'snowboard', 37: 'sports ball', 38: 'kite',
            39: 'baseball bat', 40: 'baseball glove', 41: 'skateboard', 42: 'surfboard',
            43: 'tennis racket', 44: 'bottle', 45: 'wine glass', 46: 'cup',
            47: 'fork', 48: 'knife', 49: 'spoon', 50: 'bowl',
            51: 'banana', 52: 'apple', 53: 'sandwich', 54: 'orange', 55: 'broccoli',
            56: 'carrot', 57: 'hot dog', 58: 'pizza', 59: 'donut', 60: 'cake',
            61: 'chair', 62: 'couch', 63: 'potted plant', 64: 'bed', 65: 'dining table',
            66: 'toilet', 67: 'tv', 68: 'laptop', 69: 'mouse', 70: 'remote',
            71: 'keyboard', 72: 'cell phone', 73: 'microwave', 74: 'oven', 75: 'toaster',
            76: 'sink', 77: 'refrigerator', 78: 'book', 79: 'clock', 80: 'vase',
            81: 'scissors', 82: 'teddy bear', 83: 'hair drier', 84: 'toothbrush'
        }
        return coco_classes.get(class_id, f'unknown_{class_id}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidManipulationPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Assessment Questions

1. How do you configure Isaac Sim for humanoid robot simulation with proper physics parameters?
2. What are the key components of an Isaac ROS perception pipeline for humanoid navigation?
3. How do you integrate Isaac ROS perception with Nav2 for human-aware navigation?
4. What are the main challenges in Isaac Sim RL environment setup for bipedal locomotion?
5. How do you create a manipulation pipeline using Isaac ROS for humanoid arm control?

## Best Practices for Humanoid Navigation with Isaac

1. **Simulation Fidelity**: Ensure physics parameters match real humanoid robot characteristics
2. **Perception Integration**: Combine Isaac ROS perception with navigation for robust operation
3. **Balance Awareness**: Incorporate balance constraints in all navigation decisions
4. **Safety First**: Implement comprehensive safety checks and emergency stops
5. **Validation**: Regularly validate simulation results against real-world humanoid capabilities

## Next Steps

After completing these exercises, you should have hands-on experience with:
- Isaac Sim setup for humanoid robotics
- Isaac ROS perception pipelines for navigation
- Integration with Nav2 for humanoid-aware navigation
- Reinforcement learning environments for locomotion
- Manipulation pipeline development

Continue to the next module to learn about Vision-Language-Action (VLA) systems for humanoid robotics.