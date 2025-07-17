#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
import math
import time
from enum import Enum

class RoverState(Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    RETURNING_HOME = "returning_home"
    EMERGENCY_STOP = "emergency_stop"

class MarsRoverController(Node):
    def __init__(self):
        super().__init__('mars_rover_controller')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_distance_threshold', 1.5)
        self.declare_parameter('exploration_radius', 10.0)
        self.declare_parameter('home_position_tolerance', 0.5)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.home_tolerance = self.get_parameter('home_position_tolerance').value
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/mars_rover/odom', self.odom_callback, 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/mars_rover/scan', self.laser_callback, sensor_qos)
        self.imu_subscriber = self.create_subscription(
            Imu, '/mars_rover/imu', self.imu_callback, sensor_qos)
        self.obstacle_alert_subscriber = self.create_subscription(
            Bool, '/mars_rover/obstacle_detected', self.obstacle_alert_callback, 10)
        self.command_subscriber = self.create_subscription(
            String, '/mars_rover/command', self.command_callback, 10)
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_publisher = self.create_publisher(String, '/mars_rover/status', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # State variables
        self.current_state = RoverState.IDLE
        self.current_pose = None
        self.current_velocity = None
        self.home_position = None
        self.target_goal = None
        self.laser_data = None
        self.imu_data = None
        self.obstacle_detected = False
        self.emergency_stop = False
        
        # Exploration variables
        self.exploration_start_time = None
        self.last_direction_change = 0
        self.current_exploration_angle = 0
        self.stuck_counter = 0
        self.last_position = None
        self.position_history = []
        
        # Safety variables
        self.min_obstacle_distance = float('inf')
        self.last_emergency_time = 0
        
        self.get_logger().info('Mars Rover Controller initialized')
        self.publish_status(f"Rover controller started - State: {self.current_state.value}")

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Set home position on first odometry message
        if self.home_position is None:
            self.home_position = self.current_pose.position
            self.get_logger().info(f'Home position set: x={self.home_position.x:.2f}, y={self.home_position.y:.2f}')
        
        # Update position history for stuck detection
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        self.position_history.append(current_pos)
        if len(self.position_history) > 50:  # Keep last 5 seconds at 10Hz
            self.position_history.pop(0)

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg
        
        # Calculate minimum obstacle distance in front
        front_angles = []
        ranges = []
        
        for i, range_val in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Focus on front sector (±45 degrees)
            if abs(angle) <= math.pi/4 and msg.range_min <= range_val <= msg.range_max:
                front_angles.append(angle)
                ranges.append(range_val)
        
        if ranges:
            self.min_obstacle_distance = min(ranges)
        else:
            self.min_obstacle_distance = float('inf')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg

    def obstacle_alert_callback(self, msg):
        """Process obstacle detection alerts"""
        self.obstacle_detected = msg.data
        
        if self.obstacle_detected and self.current_state != RoverState.EMERGENCY_STOP:
            self.get_logger().warn('Obstacle detected by vision system')

    def command_callback(self, msg):
        """Process external commands"""
        command = msg.data.lower()
        
        if command == 'start_exploration':
            self.start_exploration()
        elif command == 'return_home':
            self.return_home()
        elif command == 'stop':
            self.emergency_stop_rover()
        elif command == 'resume':
            self.resume_operation()
        elif command == 'idle':
            self.set_state(RoverState.IDLE)
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None:
            return
        
        # Check for emergency conditions
        if self.check_emergency_conditions():
            if self.current_state != RoverState.EMERGENCY_STOP:
                self.emergency_stop_rover()
            return
        
        # Check if stuck
        if self.is_stuck():
            self.handle_stuck_situation()
        
        # State machine
        if self.current_state == RoverState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RoverState.EXPLORING:
            self.handle_exploration_state()
        elif self.current_state == RoverState.AVOIDING_OBSTACLE:
            self.handle_obstacle_avoidance_state()
        elif self.current_state == RoverState.RETURNING_HOME:
            self.handle_return_home_state()
        elif self.current_state == RoverState.EMERGENCY_STOP:
            self.handle_emergency_stop_state()

    def check_emergency_conditions(self):
        """Check for emergency stop conditions"""
        # Very close obstacle
        if self.min_obstacle_distance < 0.3:
            return True
        
        # IMU indicates rover is tilted dangerously
        if self.imu_data:
            # Convert quaternion to euler angles
            orientation = self.imu_data.orientation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
            cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            
            # Check for dangerous tilt (more than 30 degrees)
            if abs(roll) > math.pi/6 or abs(pitch) > math.pi/6:
                return True
        
        return False

    def is_stuck(self):
        """Detect if rover is stuck"""
        if len(self.position_history) < 30:  # Need enough history
            return False
        
        # Calculate movement in last 3 seconds
        recent_positions = self.position_history[-30:]
        distances = []
        
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i][0] - recent_positions[i-1][0]
            dy = recent_positions[i][1] - recent_positions[i-1][1]
            distances.append(math.sqrt(dx*dx + dy*dy))
        
        total_movement = sum(distances)
        
        # If moved less than 10cm in 3 seconds while trying to move
        if total_movement < 0.1 and self.current_state == RoverState.EXPLORING:
            self.stuck_counter += 1
            if self.stuck_counter > 10:  # Stuck for 1 second
                return True
        else:
            self.stuck_counter = 0
        
        return False

    def handle_stuck_situation(self):
        """Handle when rover is stuck"""
        self.get_logger().warn('Rover appears to be stuck, attempting recovery')
        
        # Try backing up and turning
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2  # Backup slowly
        cmd_vel.angular.z = 0.5  # Turn while backing up
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Reset stuck counter
        self.stuck_counter = 0

    def handle_idle_state(self):
        """Handle idle state"""
        cmd_vel = Twist()  # Zero velocity
        self.cmd_vel_publisher.publish(cmd_vel)

    def handle_exploration_state(self):
        """Handle autonomous exploration"""
        cmd_vel = Twist()
        
        # Check for obstacles
        if self.min_obstacle_distance < self.obstacle_threshold:
            self.set_state(RoverState.AVOIDING_OBSTACLE)
            return
        
        # Simple exploration behavior
        current_time = time.time()
        
        # Change direction every 10 seconds or when obstacle detected
        if (current_time - self.last_direction_change) > 10.0:
            self.current_exploration_angle = np.random.uniform(-math.pi, math.pi)
            self.last_direction_change = current_time
        
        # Move towards exploration angle
        target_yaw = self.current_exploration_angle
        current_yaw = self.get_current_yaw()
        
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        # Set velocities
        if abs(angle_diff) > 0.1:
            cmd_vel.angular.z = np.clip(angle_diff * 2.0, -self.max_angular_speed, self.max_angular_speed)
            cmd_vel.linear.x = self.max_linear_speed * 0.3  # Slow forward while turning
        else:
            cmd_vel.linear.x = self.max_linear_speed
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(cmd_vel)

    def handle_obstacle_avoidance_state(self):
        """Handle obstacle avoidance"""
        cmd_vel = Twist()
        
        if self.min_obstacle_distance > self.obstacle_threshold * 1.5:
            # Obstacle cleared, return to exploration
            self.set_state(RoverState.EXPLORING)
            return
        
        # Simple obstacle avoidance: stop and turn right
        if self.min_obstacle_distance < self.obstacle_threshold * 0.7:
            # Too close, backup
            cmd_vel.linear.x = -0.2
            cmd_vel.angular.z = 0.5
        else:
            # Turn to avoid
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.8
        
        self.cmd_vel_publisher.publish(cmd_vel)

    def handle_return_home_state(self):
        """Handle returning to home position"""
        if self.home_position is None:
            self.get_logger().error('No home position set!')
            self.set_state(RoverState.IDLE)
            return
        
        # Calculate distance to home
        dx = self.home_position.x - self.current_pose.position.x
        dy = self.home_position.y - self.current_pose.position.y
        distance_to_home = math.sqrt(dx*dx + dy*dy)
        
        # Check if arrived at home
        if distance_to_home < self.home_tolerance:
            self.get_logger().info('Arrived at home position')
            self.set_state(RoverState.IDLE)
            return
        
        # Navigate towards home
        cmd_vel = Twist()
        
        # Check for obstacles on the way
        if self.min_obstacle_distance < self.obstacle_threshold:
            # Avoid obstacle while returning home
            cmd_vel.angular.z = 0.8  # Turn to avoid
        else:
            # Calculate direction to home
            target_yaw = math.atan2(dy, dx)
            current_yaw = self.get_current_yaw()
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            # Set velocities
            if abs(angle_diff) > 0.1:
                cmd_vel.angular.z = np.clip(angle_diff * 2.0, -self.max_angular_speed, self.max_angular_speed)
                cmd_vel.linear.x = self.max_linear_speed * 0.5
            else:
                cmd_vel.linear.x = self.max_linear_speed * 0.8
        
        self.cmd_vel_publisher.publish(cmd_vel)

    def handle_emergency_stop_state(self):
        """Handle emergency stop"""
        cmd_vel = Twist()  # Zero velocity
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Check if emergency condition cleared
        if not self.check_emergency_conditions():
            current_time = time.time()
            if current_time - self.last_emergency_time > 2.0:  # Wait 2 seconds
                self.get_logger().info('Emergency condition cleared, resuming operation')
                self.set_state(RoverState.IDLE)

    def start_exploration(self):
        """Start autonomous exploration"""
        self.set_state(RoverState.EXPLORING)
        self.exploration_start_time = time.time()
        self.last_direction_change = time.time()
        self.current_exploration_angle = 0  # Start moving forward
        self.get_logger().info('Starting autonomous exploration')

    def return_home(self):
        """Return to home position"""
        if self.home_position is None:
            self.get_logger().error('No home position set!')
            return
        
        self.set_state(RoverState.RETURNING_HOME)
        self.get_logger().info('Returning to home position')

    def emergency_stop_rover(self):
        """Emergency stop the rover"""
        self.set_state(RoverState.EMERGENCY_STOP)
        self.last_emergency_time = time.time()
        self.get_logger().warn('EMERGENCY STOP activated!')

    def resume_operation(self):
        """Resume normal operation"""
        if self.current_state == RoverState.EMERGENCY_STOP:
            self.set_state(RoverState.IDLE)
            self.get_logger().info('Resuming normal operation')

    def set_state(self, new_state):
        """Change rover state"""
        if new_state != self.current_state:
            self.get_logger().info(f'State change: {self.current_state.value} -> {new_state.value}')
            self.current_state = new_state
            self.publish_status(f"State: {new_state.value}")

    def get_current_yaw(self):
        """Get current yaw angle from pose"""
        if self.current_pose is None:
            return 0.0
        
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    rover_controller = MarsRoverController()
    
    try:
        rclpy.spin(rover_controller)
    except KeyboardInterrupt:
        rover_controller.get_logger().info('Shutting down rover controller')
    finally:
        rover_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
