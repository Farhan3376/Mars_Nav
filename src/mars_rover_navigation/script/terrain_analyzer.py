#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
import json
from collections import deque
import time

class TerrainFeature:
    def __init__(self, feature_type, position, confidence, size=1.0, description=""):
        self.type = feature_type  # 'rock', 'crater', 'hill', 'flat', 'rough'
        self.position = position  # (x, y, z)
        self.confidence = confidence  # 0.0 to 1.0
        self.size = size
        self.description = description
        self.timestamp = time.time()

class TerrainAnalyzer(Node):
    def __init__(self):
        super().__init__('terrain_analyzer')
        
        # Parameters
        self.declare_parameter('analysis_frequency', 5.0)
        self.declare_parameter('terrain_map_resolution', 0.1)
        self.declare_parameter('terrain_map_size', 100)
        self.declare_parameter('rock_detection_threshold', 0.3)
        self.declare_parameter('roughness_window_size', 10)
        
        self.analysis_freq = self.get_parameter('analysis_frequency').value
        self.map_resolution = self.get_parameter('terrain_map_resolution').value
        self.map_size = self.get_parameter('terrain_map_size').value
        self.rock_threshold = self.get_parameter('rock_detection_threshold').value
        self.roughness_window = self.get_parameter('roughness_window_size').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # QoS profiles
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/mars_rover/scan', self.laser_callback, sensor_qos)
        self.image_subscriber = self.create_subscription(
            Image, '/mars_rover/camera/image_raw', self.image_callback, sensor_qos)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/mars_rover/odom', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, '/mars_rover/imu', self.imu_callback, sensor_qos)
        
        # Publishers
        self.terrain_map_publisher = self.create_publisher(
            OccupancyGrid, '/mars_rover/terrain_map', 10)
        self.features_publisher = self.create_publisher(
            MarkerArray, '/mars_rover/terrain_features', 10)
        self.analysis_publisher = self.create_publisher(
            String, '/mars_rover/terrain_analysis', 10)
        self.processed_image_publisher = self.create_publisher(
            Image, '/mars_rover/processed_terrain_image', 10)
        
        # Analysis timer
        self.analysis_timer = self.create_timer(
            1.0 / self.analysis_freq, self.analyze_terrain)
        
        # Data storage
        self.latest_laser = None
        self.latest_image = None
        self.current_pose = None
        self.current_orientation = None
        self.terrain_features = []
        self.laser_history = deque(maxlen=100)
        self.position_history = deque(maxlen=200)
        
        # Terrain mapping
        self.terrain_map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.roughness_map = np.zeros((self.map_size, self.map_size), dtype=np.float32)
        self.traversability_map = np.zeros((self.map_size, self.map_size), dtype=np.float32)
        
        # Initialize map center
        self.map_origin_x = -self.map_size * self.map_resolution / 2
        self.map_origin_y = -self.map_size * self.map_resolution / 2
        
        self.get_logger().info('Terrain Analyzer initialized')

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.latest_laser = msg
        self.laser_history.append({
            'timestamp': time.time(),
            'ranges': np.array(msg.ranges),
            'angles': np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        })

    def image_callback(self, msg):
        """Process camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        
        self.position_history.append({
            'timestamp': time.time(),
            'x': self.current_pose.x,
            'y': self.current_pose.y,
            'z': self.current_pose.z
        })

    def imu_callback(self, msg):
        """Process IMU data for terrain roughness estimation"""
        if len(self.position_history) > 0:
            # Use IMU acceleration to estimate terrain roughness
            accel = msg.linear_acceleration
            accel_magnitude = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
            
            # Store acceleration data with current position
            current_pos = self.position_history[-1]
            self.update_roughness_map(current_pos['x'], current_pos['y'], accel_magnitude)

    def analyze_terrain(self):
        """Main terrain analysis function"""
        if self.current_pose is None:
            return
        
        # Analyze different aspects of terrain
        self.analyze_laser_terrain()
        self.analyze_visual_terrain()
        self.update_traversability_map()
        self.detect_terrain_features()
        
        # Publish results
        self.publish_terrain_map()
        self.publish_terrain_features()
        self.publish_terrain_analysis()

    def analyze_laser_terrain(self):
        """Analyze terrain using laser scan data"""
        if self.latest_laser is None or self.current_pose is None:
            return
        
        ranges = np.array(self.latest_laser.ranges)
        angles = np.linspace(
            self.latest_laser.angle_min, 
            self.latest_laser.angle_max, 
            len(ranges)
        )
        
        # Filter valid ranges
        valid_idx = (ranges >= self.latest_laser.range_min) & (ranges <= self.latest_laser.range_max)
        valid_ranges = ranges[valid_idx]
        valid_angles = angles[valid_idx]
        
        if len(valid_ranges) == 0:
            return
        
        # Convert to world coordinates
        current_yaw = self.get_current_yaw()
        
        for range_val, angle in zip(valid_ranges, valid_angles):
            world_angle = current_yaw + angle
            world_x = self.current_pose.x + range_val * math.cos(world_angle)
            world_y = self.current_pose.y + range_val * math.sin(world_angle)
            
            # Update terrain map
            self.update_terrain_map(world_x, world_y, occupied=True)
        
        # Analyze terrain roughness from laser
        self.analyze_laser_roughness(valid_ranges, valid_angles)

    def analyze_laser_roughness(self, ranges, angles):
        """Analyze terrain roughness from laser data"""
        if len(ranges) < 3:
            return
        
        # Calculate height variations
        heights = []
        for i in range(len(ranges) - 1):
            # Estimate height difference between adjacent points
            r1, a1 = ranges[i], angles[i]
            r2, a2 = ranges[i + 1], angles[i + 1]
            
            # Simple height estimation (assuming flat ground as reference)
            h1 = r1 * math.sin(a1) if abs(a1) < math.pi/6 else 0
            h2 = r2 * math.sin(a2) if abs(a2) < math.pi/6 else 0
            heights.append(abs(h2 - h1))
        
        if heights:
            roughness = np.std(heights)
            # Update roughness map around current position
            self.update_roughness_map(self.current_pose.x, self.current_pose.y, roughness)

    def analyze_visual_terrain(self):
        """Analyze terrain using camera image"""
        if self.latest_image is None:
            return
        
        # Create processed image for visualization
        processed_image = self.latest_image.copy()
        
        # Convert to different color spaces for analysis
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        
        # Detect rocks and obstacles
        rocks = self.detect_rocks(hsv, processed_image)
        
        # Analyze terrain texture
        texture_features = self.analyze_texture(gray)
        
        # Detect terrain slopes
        slopes = self.detect_slopes(gray, processed_image)
        
        # Publish processed image
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.processed_image_publisher.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish processed image: {e}')

    def detect_rocks(self, hsv_image, processed_image):
        """Detect rocks in the image"""
        # Define rock color ranges (Mars-like colors)
        lower_rock1 = np.array([0, 30, 30])
        upper_rock1 = np.array([25, 255, 200])
        lower_rock2 = np.array([160, 30, 30])
        upper_rock2 = np.array([180, 255, 200])
        
        # Create masks
        mask1 = cv2.inRange(hsv_image, lower_rock1, upper_rock1)
        mask2 = cv2.inRange(hsv_image, lower_rock2, upper_rock2)
        rock_mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        rock_mask = cv2.morphologyEx(rock_mask, cv2.MORPH_CLOSE, kernel)
        rock_mask = cv2.morphologyEx(rock_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(rock_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rocks = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum rock size
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Estimate world position (simple approximation)
                distance = self.estimate_distance_from_image_position(y + h//2)
                angle = self.pixel_to_angle(x + w//2)
                
                if distance > 0:
                    world_x = self.current_pose.x + distance * math.cos(self.get_current_yaw() + angle)
                    world_y = self.current_pose.y + distance * math.sin(self.get_current_yaw() + angle)
                    
                    confidence = min(1.0, area / 1000.0)
                    rock = TerrainFeature(
                        'rock', 
                        (world_x, world_y, 0), 
                        confidence,
                        size=area/500.0,
                        description=f"Rock detected at distance {distance:.1f}m"
                    )
                    rocks.append(rock)
                
                # Draw on processed image
                cv2.rectangle(processed_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(processed_image, f'Rock {confidence:.2f}', 
                           (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return rocks

    def analyze_texture(self, gray_image):
        """Analyze terrain texture to determine roughness"""
        # Calculate local texture features
        # Use Gabor filters for texture analysis
        features = {}
        
        # Calculate local standard deviation (roughness indicator)
        kernel_size = 15
        kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size * kernel_size)
        mean = cv2.filter2D(gray_image.astype(np.float32), -1, kernel)
        sqr_diff = (gray_image.astype(np.float32) - mean) ** 2
        std_dev = np.sqrt(cv2.filter2D(sqr_diff, -1, kernel))
        
        features['roughness'] = np.mean(std_dev)
        features['texture_variance'] = np.var(gray_image)
        
        return features

    def detect_slopes(self, gray_image, processed_image):
        """Detect terrain slopes in the image"""
        # Calculate gradients
        grad_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
        
        # Calculate gradient magnitude
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # Threshold for slope detection
        slope_threshold = np.percentile(gradient_magnitude, 90)
        slope_mask = gradient_magnitude > slope_threshold
        
        # Find slope regions
        contours, _ = cv2.findContours(
            slope_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        slopes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                # Draw slope regions
                cv2.drawContours(processed_image, [contour], -1, (0, 0, 255), 2)
                
                # Estimate slope position
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    distance = self.estimate_distance_from_image_position(cy)
                    angle = self.pixel_to_angle(cx)
                    
                    if distance > 0:
                        world_x = self.current_pose.x + distance * math.cos(self.get_current_yaw() + angle)
                        world_y = self.current_pose.y + distance * math.sin(self.get_current_yaw() + angle)
                        
                        slope = TerrainFeature(
                            'slope',
                            (world_x, world_y, 0),
                            0.7,
                            size=area/1000.0,
                            description="Terrain slope detected"
                        )
                        slopes.append(slope)
        
        return slopes

    def estimate_distance_from_image_position(self, pixel_y):
        """Estimate distance based on pixel position in image"""
        # Simple perspective model - this would need camera calibration for accuracy
        image_height = self.latest_image.shape[0] if self.latest_image is not None else 480
        
        # Assume camera is 0.3m high and tilted slightly down
        camera_height = 0.3
        tilt_angle = -0.1  # radians, slightly down
        
        # Convert pixel to angle
        vertical_fov = math.pi / 3  # 60 degrees
        pixel_angle = tilt_angle + (pixel_y - image_height/2) * vertical_fov / image_height
        
        # Calculate distance using simple trigonometry
        if pixel_angle < 0:  # Looking down
            distance = camera_height / abs(math.tan(pixel_angle))
            return min(distance, 10.0)  # Cap at 10 meters
        
        return 0.0

    def pixel_to_angle(self, pixel_x):
        """Convert pixel x-coordinate to angle"""
        image_width = self.latest_image.shape[1] if self.latest_image is not None else 640
        horizontal_fov = math.pi / 3  # 60 degrees
        
        return (pixel_x - image_width/2) * horizontal_fov / image_width

    def update_terrain_map(self, world_x, world_y, occupied=True):
        """Update occupancy grid terrain map"""
        # Convert world coordinates to map coordinates
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
            if occupied:
                self.terrain_map[map_y, map_x] = 100  # Occupied
            else:
                self.terrain_map[map_y, map_x] = 0   # Free

    def update_roughness_map(self, world_x, world_y, roughness):
        """Update terrain roughness map"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        
        if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
            # Use exponential moving average
            alpha = 0.3
            self.roughness_map[map_y, map_x] = (
                alpha * roughness + (1 - alpha) * self.roughness_map[map_y, map_x]
            )

    def update_traversability_map(self):
        """Update traversability map based on terrain analysis"""
        # Combine occupancy and roughness for traversability
        for i in range(self.map_size):
            for j in range(self.map_size):
                if self.terrain_map[i, j] == 100:  # Occupied
                    self.traversability_map[i, j] = 0.0  # Not traversable
                else:
                    # Calculate traversability based on roughness
                    roughness = self.roughness_map[i, j]
                    traversability = max(0.0, 1.0 - roughness / 10.0)  # Scale roughness
                    self.traversability_map[i, j] = traversability

    def detect_terrain_features(self):
        """Detect and classify terrain features"""
        # Clear old features (keep only recent ones)
        current_time = time.time()
        self.terrain_features = [
            f for f in self.terrain_features 
            if current_time - f.timestamp < 60.0  # Keep for 1 minute
        ]
        
        # Add new features from visual analysis
        if self.latest_image is not None:
            hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
            processed_image = self.latest_image.copy()
            
            rocks = self.detect_rocks(hsv, processed_image)
            self.terrain_features.extend(rocks)

    def get_current_yaw(self):
        """Get current yaw angle from orientation"""
        if self.current_orientation is None:
            return 0.0
        
        q = self.current_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_terrain_map(self):
        """Publish occupancy grid terrain map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        
        # Flatten map and convert to list
        map_msg.data = self.terrain_map.flatten().tolist()
        
        self.terrain_map_publisher.publish(map_msg)

    def publish_terrain_features(self):
        """Publish detected terrain features as markers"""
        marker_array = MarkerArray()
        
        for i, feature in enumerate(self.terrain_features):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = feature.position[0]
            marker.pose.position.y = feature.position[1]
            marker.pose.position.z = feature.position[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = feature.size
            marker.scale.y = feature.size
            marker.scale.z = feature.size
            
            # Color based on feature type
            if feature.type == 'rock':
                marker.color.r = 0.8
                marker.color.g = 0.4
                marker.color.b = 0.2
            elif feature.type == 'slope':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            
            marker.color.a = feature.confidence
            marker.lifetime.sec = 60  # Marker lifetime
            
            marker_array.markers.append(marker)
        
        self.features_publisher.publish(marker_array)

    def publish_terrain_analysis(self):
        """Publish terrain analysis summary"""
        analysis = {
            'timestamp': time.time(),
            'features_detected': len(self.terrain_features),
            'terrain_types': {},
            'traversability_summary': {
                'mean': float(np.mean(self.traversability_map)),
                'min': float(np.min(self.traversability_map)),
                'max': float(np.max(self.traversability_map))
            },
            'roughness_summary': {
                'mean': float(np.mean(self.roughness_map)),
                'max': float(np.max(self.roughness_map))
            }
        }
        
        # Count feature types
        for feature in self.terrain_features:
            if feature.type in analysis['terrain_types']:
                analysis['terrain_types'][feature.type] += 1
            else:
                analysis['terrain_types'][feature.type] = 1
        
        # Publish as JSON string
        analysis_msg = String()
        analysis_msg.data = json.dumps(analysis, indent=2)
        self.analysis_publisher.publish(analysis_msg)

def main(args=None):
    rclpy.init(args=args)
    
    terrain_analyzer = TerrainAnalyzer()
    
    try:
        rclpy.spin(terrain_analyzer)
    except KeyboardInterrupt:
        terrain_analyzer.get_logger().info('Shutting down terrain analyzer')
    finally:
        terrain_analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
