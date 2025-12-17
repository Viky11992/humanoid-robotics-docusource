---
sidebar_position: 4
title: Isaac ROS Navigation for Bipedal Humanoid Robots
---

# Isaac ROS Navigation for Bipedal Humanoid Robots

## Introduction to Isaac ROS Navigation for Humanoid Robots

Navigation for bipedal humanoid robots with Isaac ROS presents unique challenges that combine advanced perception with complex locomotion:

- **Dynamic balance**: Maintaining stability during movement using Isaac's balance algorithms
- **Step planning**: Calculating safe foot placements using Isaac's physics-aware planning
- **Terrain adaptation**: Handling stairs, slopes, and uneven surfaces with Isaac's 3D perception
- **Human-aware navigation**: Navigating in human environments safely using Isaac's perception
- **3D obstacle avoidance**: Considering the robot's full body dimensions with Isaac's spatial understanding
- **Isaac Sim integration**: Training navigation policies in photorealistic simulation

## Nav2 Architecture for Humanoids

### 1. Navigation Stack Components

#### Standard Nav2 Components
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Behavior      │    │   Controller    │    │   Planner       │
│   Tree          │───▶│   Server        │───▶│   Server        │
│   Server        │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Recovery      │    │   Velocity      │    │   Global        │
│   Server        │    │   Smoothers     │    │   Planner       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

#### Humanoid-Specific Enhancements
- **Step Planner**: Specialized planner for footstep sequences
- **Balance Controller**: Maintains COG during navigation
- **Terrain Analyzer**: Processes 3D data for traversability
- **Gait Generator**: Creates stable walking patterns

### 2. Coordinate Frames for Humanoid Navigation

#### Frame Hierarchy
```yaml
tf_tree:
  map:                    # Global map frame
    └── odom:             # Odometry frame (drift corrected)
        └── base_link:    # Robot base (between feet)
            ├── torso:    # Torso frame (center of mass)
            ├── head:     # Head frame
            ├── left_foot:   # Left foot frame
            ├── right_foot:  # Right foot frame
            └── camera_link: # Camera frame
```

## Humanoid-Specific Navigation Challenges

### 1. Dynamic Balance Maintenance

#### Center of Gravity (COG) Management
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.balance_state_pub = self.create_publisher(Float64, '/balance_state', 10)

        # Balance parameters
        self.balance_threshold = 0.1  # meters
        self.stance_width = 0.25      # distance between feet (meters)
        self.com_height = 0.8         # center of mass height (meters)

        # Robot state
        self.current_pose = None
        self.current_goal = None
        self.in_motion = False

        # Timer for balance control
        self.balance_timer = self.create_timer(0.05, self.balance_control)  # 20 Hz

        self.get_logger().info('Balance Controller initialized')

    def goal_callback(self, msg):
        """Receive navigation goal."""
        self.current_goal = msg
        self.get_logger().info(f'Received goal: x={msg.pose.position.x}, y={msg.pose.position.y}')

    def odom_callback(self, msg):
        """Receive robot odometry."""
        self.current_pose = msg

    def balance_control(self):
        """Main balance control loop."""
        if not self.current_pose or not self.current_goal:
            return

        # Calculate distance to goal
        dx = self.current_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.pose.position.y
        distance_to_goal = np.sqrt(dx*dx + dy*dy)

        # Calculate desired velocity based on balance
        cmd_vel = Twist()

        if distance_to_goal > 0.1:  # If not close to goal
            # Calculate heading to goal
            desired_yaw = np.arctan2(dy, dx)

            # Calculate current orientation (simplified)
            current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)

            # Calculate heading error
            heading_error = self.normalize_angle(desired_yaw - current_yaw)

            # Balance-aware velocity calculation
            linear_vel = self.calculate_balanced_linear_velocity(distance_to_goal)
            angular_vel = self.calculate_balanced_angular_velocity(heading_error)

            # Apply velocity limits based on balance state
            cmd_vel.linear.x = max(0.0, min(linear_vel, 0.3))  # Limit to 0.3 m/s
            cmd_vel.angular.z = max(-0.5, min(angular_vel, 0.5))  # Limit angular velocity

            # Check balance state
            balance_ok = self.check_balance_state()

            if balance_ok:
                self.cmd_vel_pub.publish(cmd_vel)
                self.in_motion = True
            else:
                # Stop robot if balance is compromised
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                self.in_motion = False
                self.get_logger().warn('Balance compromised - stopping robot')

        # Publish balance state
        balance_msg = Float64()
        balance_msg.data = float(self.calculate_balance_metric())
        self.balance_state_pub.publish(balance_msg)

    def calculate_balanced_linear_velocity(self, distance_to_goal):
        """Calculate linear velocity considering balance constraints."""
        # Maximum velocity when far from goal, reduce as approaching
        max_vel = 0.3  # m/s
        approach_vel = 0.1  # m/s when approaching goal

        if distance_to_goal < 0.5:
            # Slow down when approaching goal
            vel = approach_vel
        else:
            # Normal speed when far from goal
            vel = max_vel

        # Reduce velocity if balance is marginal
        balance_factor = min(1.0, self.calculate_balance_margin() * 5)
        return vel * balance_factor

    def calculate_balanced_angular_velocity(self, heading_error):
        """Calculate angular velocity considering balance constraints."""
        # Use PID-like control for heading
        kp = 0.8  # proportional gain
        max_angular_vel = 0.5  # rad/s

        angular_vel = kp * heading_error
        angular_vel = max(-max_angular_vel, min(angular_vel, max_angular_vel))

        # Reduce angular velocity if balance is marginal
        balance_factor = min(1.0, self.calculate_balance_margin() * 3)
        return angular_vel * balance_factor

    def check_balance_state(self):
        """Check if robot is in a balanced state."""
        # This is a simplified check - in reality, you'd check:
        # - ZMP (Zero Moment Point) position
        # - COM position relative to support polygon
        # - Joint angles and torques
        # - IMU data for stability

        # For now, return True (this would be more complex in practice)
        return True

    def calculate_balance_margin(self):
        """Calculate balance margin (0.0 to 1.0)."""
        # Simplified balance margin calculation
        # In practice, this would use ZMP, COM position, etc.
        return 0.8  # Default to 80% balance margin

    def calculate_balance_metric(self):
        """Calculate overall balance metric."""
        # Combine multiple balance factors
        balance_margin = self.calculate_balance_margin()

        # Could include other factors like joint torques, IMU data, etc.
        return balance_margin

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
    controller = BalanceController()

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

### 2. Step Planning for Bipedal Locomotion

#### Footstep Planner Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Subscribers
        self.global_plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.global_plan_callback,
            10
        )

        # Publishers
        self.left_foot_plan_pub = self.create_publisher(Path, '/left_foot_plan', 10)
        self.right_foot_plan_pub = self.create_publisher(Path, '/right_foot_plan', 10)
        self.footstep_markers_pub = self.create_publisher(MarkerArray, '/footstep_markers', 10)

        # Footstep parameters
        self.step_length = 0.3      # Distance between consecutive steps (meters)
        self.step_width = 0.25      # Distance between left and right feet (meters)
        self.step_height = 0.05     # Clearance height for stepping (meters)
        self.turn_step_threshold = 0.2  # Threshold for turn steps (radians)

        # Robot state
        self.global_plan = None
        self.left_foot_pose = None
        self.right_foot_pose = None

        self.get_logger().info('Footstep Planner initialized')

    def global_plan_callback(self, msg):
        """Receive global path plan and generate footstep plan."""
        self.global_plan = msg
        self.get_logger().info(f'Received global plan with {len(msg.poses)} poses')

        if len(msg.poses) > 1:
            # Generate footstep plan from global path
            left_foot_plan, right_foot_plan = self.generate_footstep_plan(msg)

            # Publish footstep plans
            self.left_foot_plan_pub.publish(left_foot_plan)
            self.right_foot_plan_pub.publish(right_foot_plan)

            # Publish visualization markers
            self.publish_footstep_markers(left_foot_plan, right_foot_plan)

    def generate_footstep_plan(self, global_path):
        """Generate footstep plan from global path."""
        left_foot_path = Path()
        right_foot_path = Path()

        left_foot_path.header = global_path.header
        right_foot_path.header = global_path.header

        if len(global_path.poses) < 2:
            return left_foot_path, right_foot_path

        # Initialize foot poses (assuming robot starts with left foot forward)
        start_pose = global_path.poses[0].pose
        self.left_foot_pose = self.offset_pose(start_pose, 0.1, 0.125)   # Left foot slightly forward and right
        self.right_foot_pose = self.offset_pose(start_pose, 0.1, -0.125)  # Right foot slightly forward and left

        # Add initial foot poses
        left_foot_path.poses.append(self.create_pose_stamped(self.left_foot_pose))
        right_foot_path.poses.append(self.create_pose_stamped(self.right_foot_pose))

        # Process each segment of the global path
        current_left_pose = self.left_foot_pose
        current_right_pose = self.right_foot_pose
        left_support = False  # Start with right foot support

        for i in range(1, len(global_path.poses)):
            target_pose = global_path.poses[i].pose

            # Calculate required step
            dx = target_pose.position.x - current_left_pose.position.x if left_support else target_pose.position.x - current_right_pose.position.x
            dy = target_pose.position.y - current_left_pose.position.y if left_support else target_pose.position.y - current_right_pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)

            # Calculate number of steps needed
            num_steps = max(1, int(np.ceil(dist / self.step_length)))

            for step in range(num_steps):
                if left_support:
                    # Move right foot toward target
                    step_progress = (step + 1) / num_steps
                    new_right_x = current_left_pose.position.x + dx * step_progress
                    new_right_y = current_left_pose.position.y + dy * step_progress

                    # Add slight offset for stability
                    new_right_y += (-1 if left_support else 1) * self.step_width / 2

                    new_pose = Pose()
                    new_pose.position.x = new_right_x
                    new_pose.position.y = new_right_y
                    new_pose.position.z = 0.0
                    new_pose.orientation = target_pose.orientation

                    right_foot_path.poses.append(self.create_pose_stamped(new_pose))
                    current_right_pose = new_pose
                    left_support = False
                else:
                    # Move left foot toward target
                    step_progress = (step + 1) / num_steps
                    new_left_x = current_right_pose.position.x + dx * step_progress
                    new_left_y = current_right_pose.position.y + dy * step_progress

                    # Add slight offset for stability
                    new_left_y += (-1 if left_support else 1) * self.step_width / 2

                    new_pose = Pose()
                    new_pose.position.x = new_left_x
                    new_pose.position.y = new_left_y
                    new_pose.position.z = 0.0
                    new_pose.orientation = target_pose.orientation

                    left_foot_path.poses.append(self.create_pose_stamped(new_pose))
                    current_left_pose = new_pose
                    left_support = True

        return left_foot_path, right_foot_path

    def offset_pose(self, pose, dx, dy):
        """Offset a pose by dx, dy."""
        new_pose = Pose()
        new_pose.position.x = pose.position.x + dx
        new_pose.position.y = pose.position.y + dy
        new_pose.position.z = pose.position.z
        new_pose.orientation = pose.orientation
        return new_pose

    def create_pose_stamped(self, pose):
        """Create PoseStamped from Pose."""
        from geometry_msgs.msg import PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def publish_footstep_markers(self, left_path, right_path):
        """Publish visualization markers for footsteps."""
        marker_array = MarkerArray()

        # Left foot markers
        for i, pose_stamped in enumerate(left_path.poses):
            marker = Marker()
            marker.header = pose_stamped.header
            marker.ns = "left_footsteps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose_stamped.pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # Blue for left foot
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        # Right foot markers
        for i, pose_stamped in enumerate(right_path.poses):
            marker = Marker()
            marker.header = pose_stamped.header
            marker.ns = "right_footsteps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose_stamped.pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0  # Red for right foot
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        self.footstep_markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    planner = FootstepPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nav2 Configuration for Humanoid Robots

### 1. Navigation Parameters

#### Humanoid Navigation Configuration (`nav2_params.yaml`)
```yaml
amcl:
  ros__parameters:
    use_sim_time: False
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
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: False
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
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICtrl"
      debug_visualizations: true
      rate_limiting: true
      min_speed_xy: 0.05
      max_speed_xy: 0.3
      min_speed_theta: 0.1
      max_speed_theta: 0.5
      # Humanoid-specific parameters
      step_size: 0.15  # Smaller steps for stability
      balance_margin: 0.8  # Required balance margin

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Humanoid robot radius (larger than typical)
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size: 0.05  # Finer resolution for humanoid navigation
      min_distance_from_robot: 0.4  # Keep path further from obstacles for stability

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1
```

### 2. Humanoid-Specific Controllers

#### Bipedal Motion Controller
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

class BipedalMotionController(Node):
    def __init__(self):
        super().__init__('bipedal_motion_controller')

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
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
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.motion_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.balance_state_pub = self.create_publisher(PoseStamped, '/balance_state', 10)

        # Humanoid-specific parameters
        self.step_frequency = 1.0  # Steps per second
        self.max_step_length = 0.3  # Maximum step length (m)
        self.balance_threshold = 0.1  # Balance error threshold (m)
        self.turn_threshold = 0.2     # Turn threshold (rad)

        # Robot state
        self.desired_twist = Twist()
        self.current_imu = None
        self.obstacle_distances = []
        self.in_motion = False

        # Timer for motion control
        self.motion_timer = self.create_timer(0.1, self.motion_control)  # 10 Hz

        self.get_logger().info('Bipedal Motion Controller initialized')

    def cmd_vel_callback(self, msg):
        """Receive navigation velocity commands."""
        self.desired_twist = msg
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def imu_callback(self, msg):
        """Receive IMU data for balance control."""
        self.current_imu = msg

    def scan_callback(self, msg):
        """Receive laser scan for obstacle detection."""
        self.obstacle_distances = list(msg.ranges)

    def motion_control(self):
        """Main motion control loop."""
        if not self.current_imu:
            return

        # Check for obstacles
        min_obstacle_dist = min([d for d in self.obstacle_distances if 0 < d < float('inf')], default=float('inf'))

        # Check balance state
        balance_ok = self.check_balance_from_imu()

        # Generate motion commands based on desired velocity
        motion_cmd = Twist()

        if min_obstacle_dist < 0.5:  # Obstacle within 0.5m
            # Emergency stop for safety
            motion_cmd.linear.x = 0.0
            motion_cmd.angular.z = 0.0
            self.get_logger().warn('Obstacle detected - stopping')
        elif not balance_ok:
            # Stop if balance is compromised
            motion_cmd.linear.x = 0.0
            motion_cmd.angular.z = 0.0
            self.get_logger().warn('Balance compromised - stopping')
        else:
            # Generate bipedal motion commands
            motion_cmd = self.generate_bipedal_commands(self.desired_twist)

        # Publish motion commands
        self.motion_cmd_pub.publish(motion_cmd)

        # Publish balance state
        balance_state = self.get_balance_state()
        self.balance_state_pub.publish(balance_state)

    def check_balance_from_imu(self):
        """Check balance using IMU data."""
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

    def generate_bipedal_commands(self, desired_twist):
        """Generate bipedal motion commands from desired velocity."""
        cmd = Twist()

        # Convert continuous velocity to discrete step commands
        # This is a simplified approach - real implementation would use
        # inverse kinematics and gait generators

        # Linear velocity: convert to step-based movement
        if abs(desired_twist.linear.x) > 0.01:  # Minimum threshold
            # Calculate step timing based on desired speed
            step_duration = 1.0 / self.step_frequency
            desired_step_length = desired_twist.linear.x * step_duration

            # Limit step length to safe values
            actual_step_length = max(-self.max_step_length,
                                   min(desired_step_length, self.max_step_length))

            # Convert step length back to velocity
            cmd.linear.x = actual_step_length * self.step_frequency
        else:
            cmd.linear.x = 0.0

        # Angular velocity: convert to turning steps
        if abs(desired_twist.angular.z) > 0.01:  # Minimum threshold
            # For turning, adjust step pattern to rotate
            cmd.angular.z = desired_twist.angular.z
        else:
            cmd.angular.z = 0.0

        return cmd

    def get_balance_state(self):
        """Get current balance state."""
        balance_state = PoseStamped()
        balance_state.header.stamp = self.get_clock().now().to_msg()
        balance_state.header.frame_id = 'base_link'

        # For now, return a simple balance indicator
        # In practice, this would include ZMP, COM position, etc.
        balance_state.pose.position.z = 1.0 if self.check_balance_from_imu() else 0.0

        return balance_state

def main(args=None):
    rclpy.init(args=args)
    controller = BipedalMotionController()

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

## Terrain Analysis for Humanoid Navigation

### 1. Traversability Analysis

#### 3D Terrain Analyzer
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class TerrainAnalyzer(Node):
    def __init__(self):
        super().__init__('terrain_analyzer')

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points_map',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.traversability_pub = self.create_publisher(Image, '/traversability_map', 10)
        self.terrain_markers_pub = self.create_publisher(MarkerArray, '/terrain_analysis', 10)

        # Terrain analysis parameters
        self.grid_resolution = 0.1  # 10cm grid cells
        self.local_window_size = 5.0  # 5m x 5m analysis window
        self.max_slope = 0.3  # Maximum traversable slope (rise/run)
        self.min_step_height = 0.02  # Minimum detectable step (2cm)
        self.max_step_height = 0.15  # Maximum traversable step (15cm)

        self.get_logger().info('Terrain Analyzer initialized')

    def pointcloud_callback(self, msg):
        """Analyze point cloud for terrain traversability."""
        # Convert point cloud to numpy array
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        if len(points) == 0:
            return

        points = np.array(points)

        # Create local grid around robot
        robot_x, robot_y = 0.0, 0.0  # This would come from localization
        local_points = self.filter_local_points(points, robot_x, robot_y)

        if len(local_points) == 0:
            return

        # Analyze terrain properties
        traversability_grid = self.analyze_terrain_traversability(local_points)

        # Publish results
        self.publish_traversability_map(traversability_grid)
        self.publish_terrain_markers(local_points, traversability_grid)

    def filter_local_points(self, all_points, robot_x, robot_y):
        """Filter points within local analysis window."""
        min_x, max_x = robot_x - self.local_window_size/2, robot_x + self.local_window_size/2
        min_y, max_y = robot_y - self.local_window_size/2, robot_y + self.local_window_size/2

        mask = (
            (all_points[:, 0] >= min_x) & (all_points[:, 0] <= max_x) &
            (all_points[:, 1] >= min_y) & (all_points[:, 1] <= max_y)
        )

        return all_points[mask]

    def analyze_terrain_traversability(self, points):
        """Analyze terrain for traversability."""
        # Create grid
        x_min, y_min = points[:, 0].min(), points[:, 1].min()
        x_max, y_max = points[:, 0].max(), points[:, 1].max()

        x_cells = int((x_max - x_min) / self.grid_resolution) + 1
        y_cells = int((y_max - y_min) / self.grid_resolution) + 1

        # Initialize traversability grid (0 = impassable, 1 = traversable)
        traversability = np.ones((y_cells, x_cells))

        # Group points by grid cell
        for i in range(len(points)):
            x_idx = int((points[i, 0] - x_min) / self.grid_resolution)
            y_idx = int((points[i, 1] - y_min) / self.grid_resolution)

            if 0 <= x_idx < x_cells and 0 <= y_idx < y_cells:
                # Analyze cell properties
                cell_traversable = self.analyze_cell_traversability(
                    points, x_idx, y_idx, x_min, y_min
                )

                if not cell_traversable:
                    traversability[y_idx, x_idx] = 0.0

        return traversability

    def analyze_cell_traversability(self, points, x_idx, y_idx, x_min, y_min):
        """Analyze a single grid cell for traversability."""
        # Get points in this cell
        cell_x_min = x_min + x_idx * self.grid_resolution
        cell_y_min = y_min + y_idx * self.grid_resolution
        cell_x_max = cell_x_min + self.grid_resolution
        cell_y_max = cell_y_min + self.grid_resolution

        cell_mask = (
            (points[:, 0] >= cell_x_min) & (points[:, 0] < cell_x_max) &
            (points[:, 1] >= cell_y_min) & (points[:, 1] < cell_y_max)
        )

        cell_points = points[cell_mask]

        if len(cell_points) == 0:
            return False  # No data - assume impassable

        # Calculate height variation (roughness)
        height_variation = np.std(cell_points[:, 2])
        if height_variation > 0.05:  # Too rough
            return False

        # Calculate local slope (simplified)
        if len(cell_points) >= 3:
            # Fit plane to points to estimate slope
            A = np.column_stack([cell_points[:, 0], cell_points[:, 1], np.ones(len(cell_points))])
            coeffs, residuals, rank, singular_values = np.linalg.lstsq(A, cell_points[:, 2], rcond=None)

            # Calculate slope from plane coefficients
            slope = np.sqrt(coeffs[0]**2 + coeffs[1]**2)
            if slope > self.max_slope:
                return False

        return True

    def publish_traversability_map(self, traversability_grid):
        """Publish traversability map as image."""
        # Convert to image format
        from cv_bridge import CvBridge
        import cv2

        # Normalize to 0-255 range
        img_data = (traversability_grid * 255).astype(np.uint8)

        # Create and publish image
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img_data, encoding="mono8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'map'

        self.traversability_pub.publish(img_msg)

    def publish_terrain_markers(self, points, traversability_grid):
        """Publish terrain analysis visualization markers."""
        marker_array = MarkerArray()

        # Create markers for non-traversable areas
        for i in range(traversability_grid.shape[0]):
            for j in range(traversability_grid.shape[1]):
                if traversability_grid[i, j] < 0.5:  # Non-traversable
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "terrain_analysis"
                    marker.id = i * traversability_grid.shape[1] + j
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Position at center of grid cell
                    resolution = self.grid_resolution
                    marker.pose.position.x = j * resolution + resolution/2
                    marker.pose.position.y = i * resolution + resolution/2
                    marker.pose.position.z = 0.05  # Height above ground

                    marker.scale.x = resolution
                    marker.scale.y = resolution
                    marker.scale.z = 0.1  # Height of marker

                    marker.color.r = 1.0  # Red for non-traversable
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5

                    marker_array.markers.append(marker)

        self.terrain_markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    analyzer = TerrainAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Isaac ROS

### 1. Isaac ROS Perception Integration

#### Perception-Enhanced Navigation
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import numpy as np

class PerceptionEnhancedNavigator(Node):
    def __init__(self):
        super().__init__('perception_enhanced_navigator')

        # Isaac ROS perception subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/isaac_ros_pointcloud',
            self.pointcloud_callback,
            10
        )

        # Navigation publishers
        self.navigation_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.avoidance_target_pub = self.create_publisher(PoseStamped, '/avoidance_target', 10)

        # Perception data
        self.latest_detections = []
        self.humanoid_detected = False
        self.closest_obstacle = float('inf')

        # Navigation state
        self.avoidance_mode = False
        self.avoidance_target = None

        # Timer for perception-based navigation decisions
        self.perception_timer = self.create_timer(0.5, self.perception_navigation_logic)

        self.get_logger().info('Perception Enhanced Navigator initialized')

    def rgb_callback(self, msg):
        """Process RGB image."""
        # This would be handled by Isaac ROS perception nodes
        pass

    def depth_callback(self, msg):
        """Process depth image."""
        # This would be handled by Isaac ROS depth processing
        pass

    def detection_callback(self, msg):
        """Process object detections."""
        self.latest_detections = msg.detections

        # Check for humans in the environment
        self.humanoid_detected = any(
            result.id == 1 for detection in msg.detections  # Assuming person class ID is 1
            for result in detection.results
        )

        if self.humanoid_detected:
            self.get_logger().info('Human detected in environment - adjusting navigation')

    def pointcloud_callback(self, msg):
        """Process point cloud for obstacle detection."""
        # Convert to numpy array
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        if points:
            points = np.array(points)

            # Calculate distances to robot (assuming robot at origin)
            distances = np.linalg.norm(points[:, :2], axis=1)
            self.closest_obstacle = np.min(distances) if len(distances) > 0 else float('inf')

    def perception_navigation_logic(self):
        """Main perception-based navigation logic."""
        navigation_status = String()

        if self.humanoid_detected and self.closest_obstacle < 1.0:
            # Activate human-aware navigation
            navigation_status.data = "HUMAN_AWARE_NAVIGATION_ACTIVE"

            # Calculate avoidance behavior
            avoidance_needed = self.calculate_avoidance_behavior()

            if avoidance_needed:
                self.activate_avoidance_mode()
            else:
                self.deactivate_avoidance_mode()
        else:
            # Standard navigation
            navigation_status.data = "STANDARD_NAVIGATION"
            self.deactivate_avoidance_mode()

        self.navigation_status_pub.publish(navigation_status)

    def calculate_avoidance_behavior(self):
        """Calculate if avoidance behavior is needed."""
        # Check if obstacle is close enough to warrant avoidance
        return self.closest_obstacle < 0.8  # 80cm threshold

    def activate_avoidance_mode(self):
        """Activate obstacle avoidance mode."""
        if not self.avoidance_mode:
            self.get_logger().info('Activating obstacle avoidance mode')
            self.avoidance_mode = True

    def deactivate_avoidance_mode(self):
        """Deactivate obstacle avoidance mode."""
        if self.avoidance_mode:
            self.get_logger().info('Deactivating obstacle avoidance mode')
            self.avoidance_mode = False

def main(args=None):
    rclpy.init(args=args)
    navigator = PerceptionEnhancedNavigator()

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

## Best Practices for Humanoid Navigation

### 1. Safety Considerations
- **Emergency stops**: Implement immediate stopping for balance loss
- **Safe trajectories**: Plan paths that maintain balance at all times
- **Fallback behaviors**: Have safe recovery plans for navigation failures
- **Human safety**: Maintain safe distances in populated areas

### 2. Performance Optimization
- **Efficient planning**: Use hierarchical planning for complex environments
- **Predictive control**: Anticipate balance adjustments
- **Adaptive parameters**: Adjust gait based on terrain and obstacles
- **Resource management**: Optimize for real-time performance on robot hardware

### 3. Validation and Testing
- **Simulation testing**: Extensively test in simulation before real robot deployment
- **Incremental complexity**: Start with simple environments and increase complexity
- **Edge case testing**: Test on challenging terrains and scenarios
- **Long-term autonomy**: Test sustained navigation over extended periods

## Next Steps

Continue to the next section to learn about Isaac ROS exercises and practical implementations.