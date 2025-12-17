---
sidebar_position: 4
title: Sensor Modeling in Simulation
---

# Sensor Modeling in Simulation

## Overview of Robot Sensors

In humanoid robotics, accurate sensor modeling is crucial for developing robust perception and control systems. Simulation must accurately represent:

- **Vision sensors**: Cameras for object recognition and navigation
- **Depth sensors**: RGB-D cameras for 3D perception
- **Inertial sensors**: IMUs for balance and orientation
- **Force/torque sensors**: For manipulation and contact detection
- **LiDAR**: For environment mapping and obstacle detection

## Camera and Vision Sensors

### 1. RGB Camera Configuration

In Gazebo, camera sensors are defined with specific parameters to match real hardware:

```xml
<!-- RGB Camera sensor -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees in radians -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/image_raw:=/camera/image_raw</remapping>
        <remapping>~/camera_info:=/camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <image_topic_name>image_raw</image_topic_name>
      <camera_info_topic_name>camera_info</camera_info_topic_name>
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Depth Camera Configuration

```xml
<!-- RGB-D Camera sensor -->
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <camera name="depth_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>5.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/rgb/image_raw:=/camera/rgb/image_raw</remapping>
        <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
        <remapping>~/depth/camera_info:=/camera/depth/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <image_topic_name>rgb/image_raw</image_topic_name>
      <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
      <point_cloud_topic_name>depth/points</point_cloud_topic_name>
      <camera_info_topic_name>rgb/camera_info</camera_info_topic_name>
      <frame_name>camera_depth_optical_frame</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.5</point_cloud_cutoff>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
      <Cx_prime>0</Cx_prime>
      <Cx>320.5</Cx>
      <Cy>240.5</Cy>
      <focal_length>525</focal_length>
      <hack_baseline>0</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

## IMU (Inertial Measurement Unit) Sensors

### 1. IMU Configuration for Balance

```xml
<!-- IMU sensor for torso -->
<gazebo reference="torso">
  <sensor name="torso_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
            <bias_mean>0.0001</bias_mean>
            <bias_stddev>0.00001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
      <frame_name>torso_imu_frame</frame_name>
      <body_name>torso</body_name>
      <update_rate>100</update_rate>
      <gaussian_noise>0.0017</gaussian_noise>
      <topic>/imu/data</topic>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Multiple IMU Setup for Humanoid Balance

```xml
<!-- Head IMU for orientation -->
<gazebo reference="head">
  <sensor name="head_imu" type="imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>0.001</stddev></noise></x>
        <y><noise type="gaussian"><stddev>0.001</stddev></noise></y>
        <z><noise type="gaussian"><stddev>0.001</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>0.017</stddev></noise></x>
        <y><noise type="gaussian"><stddev>0.017</stddev></noise></y>
        <z><noise type="gaussian"><stddev>0.017</stddev></noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>

<!-- Foot IMUs for contact detection -->
<gazebo reference="left_foot">
  <sensor name="left_foot_imu" type="imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity><x><noise type="gaussian"><stddev>0.001</stddev></noise></x></angular_velocity>
      <linear_acceleration><z><noise type="gaussian"><stddev>0.017</stddev></noise></z></linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## LiDAR Sensors

### 1. 2D LiDAR Configuration

```xml
<!-- 2D LiDAR on head -->
<gazebo reference="lidar_mount">
  <sensor name="head_lidar" type="ray">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle> <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=/scan</remapping>
      </ros>
      <frame_name>lidar_mount</frame_name>
      <topic_name>scan</topic_name>
      <gaussian_noise>0.005</gaussian_noise>
      <update_rate>10</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

### 2. 3D LiDAR Configuration

```xml
<!-- 3D LiDAR (Velodyne-style) -->
<gazebo reference="velodyne_mount">
  <sensor name="velodyne_sensor" type="ray">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- 180 degrees -->
        </horizontal>
        <vertical>
          <samples>32</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_laser.so">
      <ros>
        <namespace>/humanoid</namespace>
      </ros>
      <topic_name>velodyne_points</topic_name>
      <frame_name>velodyne_mount</frame_name>
      <min_range>0.1</min_range>
      <max_range>100.0</max_range>
      <gaussian_noise>0.008</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

## Force/Torque Sensors

### 1. Force/Torque Sensor Configuration

```xml
<!-- Force/Torque sensor in wrist -->
<gazebo reference="left_wrist">
  <sensor name="left_wrist_force_torque" type="force_torque">
    <update_rate>100</update_rate>
    <always_on>true</always_on>
    <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/wrench:=/left_wrist/force_torque</remapping>
      </ros>
      <frame_name>left_wrist</frame_name>
      <topic>/left_wrist/force_torque</topic>
      <gaussian_noise>0.01</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Fusion in Simulation

### 1. Multi-Sensor Integration Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize sensor data storage
        self.camera_data = None
        self.imu_data = None
        self.lidar_data = None
        self.camera_info = None

        # Subscribers for different sensors
        self.camera_sub = self.create_subscription(
            Image, '/humanoid/camera/image_raw', self.camera_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/humanoid/camera/camera_info', self.camera_info_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid/imu/data', self.imu_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid/scan', self.lidar_callback, 10
        )

        # Publisher for fused data
        self.fused_data_pub = self.create_publisher(
            PointStamped, '/humanoid/fused_sensor_data', 10
        )

        # TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for fusion processing
        self.timer = self.create_timer(0.1, self.fusion_callback)  # 10 Hz

    def camera_callback(self, msg):
        """Process camera data."""
        self.camera_data = msg
        # Process image data for object detection, etc.

    def camera_info_callback(self, msg):
        """Process camera info."""
        self.camera_info = msg

    def imu_callback(self, msg):
        """Process IMU data."""
        self.imu_data = msg
        # Process orientation and acceleration data

    def lidar_callback(self, msg):
        """Process LiDAR data."""
        self.lidar_data = msg
        # Process distance measurements for obstacle detection

    def fusion_callback(self):
        """Main fusion callback."""
        if not all([self.camera_data, self.imu_data, self.lidar_data, self.camera_info]):
            return

        # Example: Combine sensor data to detect obstacles in camera view
        obstacles_3d = self.lidar_to_camera_frame()
        orientation = self.get_robot_orientation()

        # Create fused result
        fused_result = self.create_fused_data(obstacles_3d, orientation)

        # Publish fused data
        self.fused_data_pub.publish(fused_result)

    def lidar_to_camera_frame(self):
        """Transform LiDAR points to camera frame."""
        try:
            # Get transform from LiDAR to camera
            transform = self.tf_buffer.lookup_transform(
                'camera_optical_frame',
                'lidar_mount',
                rclpy.time.Time()
            )

            # Transform LiDAR points to camera frame
            # This is a simplified example
            obstacles = []
            if self.lidar_data:
                for i, range_val in enumerate(self.lidar_data.ranges):
                    if 0.1 < range_val < 5.0:  # Valid range
                        angle = self.lidar_data.angle_min + i * self.lidar_data.angle_increment
                        # Convert polar to Cartesian
                        x = range_val * np.cos(angle)
                        y = range_val * np.sin(angle)
                        z = 0.0  # Assuming 2D scan

                        # Transform to camera frame
                        point = PointStamped()
                        point.point.x = x
                        point.point.y = y
                        point.point.z = z
                        point.header.frame_id = 'lidar_mount'

                        # Transform to camera frame
                        camera_point = tf2_geometry_msgs.do_transform_point(point, transform)
                        obstacles.append(camera_point.point)

            return obstacles
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
            return []

    def get_robot_orientation(self):
        """Get robot orientation from IMU data."""
        if self.imu_data:
            # Extract orientation from IMU quaternion
            orientation = self.imu_data.orientation
            return [orientation.x, orientation.y, orientation.z, orientation.w]
        return [0.0, 0.0, 0.0, 1.0]  # Default identity

    def create_fused_data(self, obstacles_3d, orientation):
        """Create fused sensor data message."""
        result = PointStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'fused_sensor_frame'

        # Example: return center of detected obstacles
        if obstacles_3d:
            avg_x = sum(p.x for p in obstacles_3d) / len(obstacles_3d)
            avg_y = sum(p.y for p in obstacles_3d) / len(obstacles_3d)
            avg_z = sum(p.z for p in obstacles_3d) / len(obstacles_3d)

            result.point.x = avg_x
            result.point.y = avg_y
            result.point.z = avg_z

        return result

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

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

## Sensor Noise and Realism

### 1. Adding Realistic Noise Models

```xml
<!-- Camera with realistic noise -->
<sensor name="noisy_camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <!-- Add realistic camera noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
</sensor>
```

### 2. Environmental Effects

```xml
<!-- Add environmental effects to sensors -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <!-- ... camera configuration ... -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <!-- ... existing configuration ... -->
      <!-- Add environmental effects -->
      <distortion_k1>0.001</distortion_k1>
      <distortion_k2>-0.002</distortion_k2>
      <distortion_k3>0.0005</distortion_k3>
      <distortion_t1>0.0001</distortion_t1>
      <distortion_t2>-0.0001</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Validation and Testing

### 1. Sensor Data Validation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
import numpy as np

class SensorValidatorNode(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Initialize validation parameters
        self.imu_bias_threshold = 0.1  # rad/s for gyroscope
        self.lidar_range_limits = (0.1, 10.0)  # meters
        self.camera_exposure_limits = (1000, 100000)  # microseconds

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/humanoid/imu/data', self.validate_imu, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/humanoid/scan', self.validate_lidar, 10)
        self.camera_sub = self.create_subscription(Image, '/humanoid/camera/image_raw', self.validate_camera, 10)

    def validate_imu(self, msg):
        """Validate IMU data."""
        # Check for extreme values (potential sensor errors)
        gyro_norm = np.sqrt(msg.angular_velocity.x**2 +
                           msg.angular_velocity.y**2 +
                           msg.angular_velocity.z**2)

        if gyro_norm > 10.0:  # Unusually high angular velocity
            self.get_logger().warn(f'High IMU angular velocity detected: {gyro_norm}')

        # Check for bias (stationary robot should have low values)
        if gyro_norm > self.imu_bias_threshold:
            self.get_logger().info(f'IMU bias detected: {gyro_norm}')

    def validate_lidar(self, msg):
        """Validate LiDAR data."""
        # Check range limits
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        invalid_ratio = 1.0 - (len(valid_ranges) / len(msg.ranges))

        if invalid_ratio > 0.1:  # More than 10% invalid ranges
            self.get_logger().warn(f'High ratio of invalid LiDAR readings: {invalid_ratio:.2%}')

        # Check for sudden changes (potential sensor noise)
        for i in range(1, len(msg.ranges)-1):
            if (abs(msg.ranges[i] - msg.ranges[i-1]) > 1.0 and
                abs(msg.ranges[i] - msg.ranges[i+1]) > 1.0):
                self.get_logger().warn(f'Potential LiDAR noise at index {i}')

    def validate_camera(self, msg):
        """Validate camera data."""
        # Check for completely black or white images (potential issues)
        image_data = np.frombuffer(msg.data, dtype=np.uint8)
        mean_intensity = np.mean(image_data)

        if mean_intensity < 10:  # Too dark
            self.get_logger().warn('Camera image appears too dark')
        elif mean_intensity > 245:  # Too bright
            self.get_logger().warn('Camera image appears too bright')

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidatorNode()

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

## Simulation-Specific Sensor Considerations

### 1. Performance vs. Accuracy Trade-offs

```xml
<!-- For real-time simulation, reduce sensor fidelity -->
<sensor name="fast_camera" type="camera">
  <update_rate>15</update_rate>  <!-- Lower update rate for performance -->
  <camera name="head_camera">
    <image>
      <width>320</width>  <!-- Lower resolution for performance -->
      <height>240</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>5.0</far>  <!-- Shorter range for performance -->
    </clip>
  </camera>
</sensor>
```

### 2. Sensor Synchronization

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters

class SensorSynchronizerNode(Node):
    def __init__(self):
        super().__init__('sensor_synchronizer')

        # Create subscribers for different sensors
        image_sub = Subscriber(self, Image, '/humanoid/camera/image_raw')
        imu_sub = Subscriber(self, Imu, '/humanoid/imu/data')
        joint_sub = Subscriber(self, JointState, '/humanoid/joint_states')

        # Synchronize sensor messages (approximate time synchronization)
        ats = ApproximateTimeSynchronizer(
            [image_sub, imu_sub, joint_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        ats.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, imu_msg, joint_msg):
        """Called when synchronized sensor data is available."""
        self.get_logger().info(f'Synchronized data: Image time {image_msg.header.stamp}, '
                              f'IMU time {imu_msg.header.stamp}, '
                              f'Joints time {joint_msg.header.stamp}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSynchronizerNode()

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

## Best Practices for Sensor Modeling

### 1. Realistic Noise Parameters

- Use noise parameters based on real sensor specifications
- Include bias, drift, and temperature effects where applicable
- Consider cross-coupling between sensor axes

### 2. Computational Efficiency

- Use appropriate update rates for simulation performance
- Consider using simplified sensor models for high-level planning
- Implement sensor data compression for network efficiency

### 3. Validation and Calibration

- Regularly validate simulation sensors against real hardware
- Implement automated sensor calibration procedures
- Monitor sensor performance metrics during simulation

## Next Steps

Continue to the next section to learn about creating Gazebo setup and configuration guides.