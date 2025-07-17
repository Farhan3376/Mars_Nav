#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import math
import time
import json
from collections import deque
from enum import Enum

class NavigationStatus(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    OBSTACLE_DETECTED = "obstacle_detected"
    STUCK = "stuck"
    GOAL_REACHED = "goal_reached"
    NAVIGATION_FAILED = "navigation_failed"
    EMERGENCY_STOP = "emergency_stop"

class NavigationMonitor(Node):
    def __init__(self):
        super().__init__('navigation_monitor')
        
        # Parameters
        self.declare_parameter('monitoring_frequency', 10.0)
        self.declare_parameter('stuck_detection_threshold', 0.1)
        self.declare_parameter('stuck_time_threshold', 5.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_navigation_time', 300.0)
        
        self.monitoring_freq = self.get_parameter('monitoring_frequency').value
        self.stuck_threshold = self.get_parameter('stuck_detection_threshold').value
        self.stuck_time_threshold = self.get_parameter('stuck_time_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_nav_time = self.get_parameter('max_navigation_time').value
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/mars_rover/odom', self.odom_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/mars_rover/scan', self.laser_callback, 10)
        self.obstacle_subscriber = self.create_subscription(
            Bool, '/mars_rover/obstacle_detected', self.obstacle_callback, 10)
        self.waypoint_status_subscriber = self.create_subscription(
            String, '/mars_rover/waypoint_status', self.waypoint_status_callback, 10)
        self.rover_status_subscriber = self.create_subscription(
            String, '/mars_rover/status', self.rover_status_callback, 10)
        
        # Publishers
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.nav_status_publisher = self.create_publisher(
            String, '/mars_rover/navigation_status', 10)
        self.performance_publisher = self.create_publisher(
            String, '/mars_rover/navigation_performance', 10)
        self.alerts_publisher = self.create_publisher(
            String, '/mars_rover/navigation_alerts', 10)
        
        # Action client for monitoring navigation goals
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Monitoring timer
        self.monitor_timer = self.create_timer(
            1.0 / self.monitoring_freq, self.monitoring_loop)
        
        # Data storage
        self.current_pose = None
        self.current_velocity = None
        self.current_goal = None
        self.latest_laser = None
        self.obstacle_detected = False
        self.waypoint_status = ""
        self.rover_status = ""
        
        # Navigation tracking
        self.navigation_status = NavigationStatus.IDLE
        self.navigation_start_time = None
        self.goal_start_time = None
        self.last_goal_distance = None
        
        # Position history for stuck detection
        self.position_history = deque(maxlen=100)  # 10 seconds at 10Hz
        self.velocity_history = deque(maxlen=50)   # 5 seconds at 10Hz
        
        # Performance metrics
        self.total_distance_traveled = 0.0
        self.successful_navigations = 0
        self.failed_navigations = 0
        self.obstacle_encounters = 0
        self.stuck_incidents = 0
        self.last_position = None
        
        # Alert tracking
        self.last_stuck_alert = 0
        self.last_obstacle_alert = 0
        self.last_performance_alert = 0
        
        self.get_logger().info('Navigation Monitor initialized')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Update position history
        current_time = time.time()
        position_data = {
            'timestamp': current_time,
            'x': self.current_pose.position.x,
            'y': self.current_pose.position.y,
            'z': self.current_pose.position.z
        }
        self.position_history.append(position_data)
        
        # Update velocity history
        velocity_magnitude = math.sqrt(
            self.current_velocity.linear.x**2 + 
            self.current_velocity.linear.y**2
        )
        self.velocity_history.append({
            'timestamp': current_time,
            'linear': velocity_magnitude,
            'angular': abs(self.current_velocity.angular.z)
        })
        
        # Calculate distance traveled
        if self.last_position is not None:
            dx = self.current_pose.position.x - self.last_position.x
            dy = self.current_pose.position.y - self.last_position.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance_traveled += distance
        
        self.last_position = self.current_pose.position

    def cmd_vel_callback(self, msg):
        """Monitor commanded velocities"""
        # This helps detect when the rover is trying to move vs actually moving
        pass

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.latest_laser = msg

    def obstacle_callback(self, msg):
        """Process obstacle detection alerts"""
        if msg.data and not self.obstacle_detected:
            self.obstacle_encounters += 1
            current_time = time.time()
            if current_time - self.last_obstacle_alert > 5.0:  # Throttle alerts
                self.publish_alert("Obstacle detected by vision system")
                self.last_obstacle_alert = current_time
        
        self.obstacle_detected = msg.data

    def waypoint_status_callback(self, msg):
        """Process waypoint navigation status"""
        self.waypoint_status = msg.data

    def rover_status_callback(self, msg):
        """Process rover status updates"""
        self.rover_status = msg.data

    def monitoring_loop(self):
        """Main monitoring loop"""
        if self.current_pose is None:
            return
        
        # Update navigation status
        self.update_navigation_status()
        
        # Check for various conditions
        self.check_stuck_condition()
        self.check_navigation_timeout()
        self.check_goal_progress()
        
        # Publish monitoring data
        self.publish_diagnostics()
        self.publish_navigation_status()
        self.publish_performance_metrics()

    def update_navigation_status(self):
        """Update current navigation status based on various indicators"""
        previous_status = self.navigation_status
        
        # Determine current status
        if self.obstacle_detected:
            self.navigation_status = NavigationStatus.OBSTACLE_DETECTED
        elif self.is_stuck():
            self.navigation_status = NavigationStatus.STUCK
        elif "emergency" in self.rover_status.lower():
            self.navigation_status = NavigationStatus.EMERGENCY_STOP
        elif "navigating" in self.waypoint_status.lower():
            self.navigation_status = NavigationStatus.NAVIGATING
        elif "completed" in self.waypoint_status.lower():
            self.navigation_status = NavigationStatus.GOAL_REACHED
        elif "failed" in self.waypoint_status.lower():
            self.navigation_status = NavigationStatus.NAVIGATION_FAILED
        else:
            self.navigation_status = NavigationStatus.IDLE
        
        # Log status changes
        if previous_status != self.navigation_status:
            self.get_logger().info(f'Navigation status changed: {previous_status.value} -> {self.navigation_status.value}')
            
            # Update counters
            if self.navigation_status == NavigationStatus.GOAL_REACHED:
                self.successful_navigations += 1
            elif self.navigation_status == NavigationStatus.NAVIGATION_FAILED:
                self.failed_navigations += 1

    def is_stuck(self):
        """Detect if rover is stuck"""
        if len(self.position_history) < 20:  # Need enough history
            return False
        
        # Check movement in recent history
        recent_positions = list(self.position_history)[-20:]  # Last 2 seconds
        
        if len(recent_positions) < 2:
            return False
        
        # Calculate total movement
        total_movement = 0.0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i]['x'] - recent_positions[i-1]['x']
            dy = recent_positions[i]['y'] - recent_positions[i-1]['y']
            total_movement += math.sqrt(dx*dx + dy*dy)
        
        # Check if velocities are being commanded but no movement
        commanded_movement = any(
            v['linear'] > 0.1 or v['angular'] > 0.1 
            for v in list(self.velocity_history)[-10:]
        ) if self.velocity_history else False
        
        # Stuck if very little movement despite commands
        if commanded_movement and total_movement < self.stuck_threshold:
            return True
        
        return False

    def check_stuck_condition(self):
        """Check and alert for stuck condition"""
        if self.is_stuck():
            current_time = time.time()
            if current_time - self.last_stuck_alert > self.stuck_time_threshold:
                self.stuck_incidents += 1
                self.publish_alert(f"Rover appears to be stuck. Incident #{self.stuck_incidents}")
                self.last_stuck_alert = current_time

    def check_navigation_timeout(self):
        """Check for navigation timeout"""
        if (self.navigation_status == NavigationStatus.NAVIGATING and 
            self.goal_start_time is not None):
            
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time > self.max_nav_time:
                self.publish_alert(f"Navigation timeout: {elapsed_time:.1f}s exceeded maximum {self.max_nav_time}s")

    def check_goal_progress(self):
        """Monitor progress towards current goal"""
        # This would require access to the current goal
        # For now, we'll monitor general progress indicators
        pass

    def calculate_navigation_efficiency(self):
        """Calculate navigation efficiency metrics"""
        if self.total_distance_traveled == 0:
            return 0.0
        
        total_attempts = self.successful_navigations + self.failed_navigations
        if total_attempts == 0:
            return 0.0
        
        success_rate = self.successful_navigations / total_attempts
        return success_rate

    def calculate_average_speed(self):
        """Calculate average movement speed"""
        if len(self.velocity_history) == 0:
            return 0.0
        
        recent_velocities = list(self.velocity_history)[-20:]  # Last 2 seconds
        speeds = [v['linear'] for v in recent_velocities]
        return np.mean(speeds) if speeds else 0.0

    def get_obstacle_frequency(self):
        """Get obstacle encounter frequency"""
        if self.total_distance_traveled == 0:
            return 0.0
        
        return self.obstacle_encounters / max(self.total_distance_traveled, 1.0)

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Navigation system status
        nav_status = DiagnosticStatus()
        nav_status.name = "mars_rover_navigation"
        nav_status.message = f"Status: {self.navigation_status.value}"
        
        if self.navigation_status in [NavigationStatus.NAVIGATING, NavigationStatus.GOAL_REACHED]:
            nav_status.level = DiagnosticStatus.OK
        elif self.navigation_status in [NavigationStatus.OBSTACLE_DETECTED]:
            nav_status.level = DiagnosticStatus.WARN
        else:
            nav_status.level = DiagnosticStatus.ERROR
        
        nav_status.values = [
            KeyValue(key="navigation_status", value=self.navigation_status.value),
            KeyValue(key="total_distance", value=f"{self.total_distance_traveled:.2f}"),
            KeyValue(key="successful_navigations", value=str(self.successful_navigations)),
            KeyValue(key="failed_navigations", value=str(self.failed_navigations)),
            KeyValue(key="obstacle_encounters", value=str(self.obstacle_encounters)),
            KeyValue(key="stuck_incidents", value=str(self.stuck_incidents)),
        ]
        
        # Movement diagnostics
        movement_status = DiagnosticStatus()
        movement_status.name = "mars_rover_movement"
        movement_status.message = f"Average speed: {self.calculate_average_speed():.2f} m/s"
        
        if self.is_stuck():
            movement_status.level = DiagnosticStatus.ERROR
            movement_status.message = "Rover appears to be stuck"
        elif self.calculate_average_speed() < 0.1:
            movement_status.level = DiagnosticStatus.WARN
            movement_status.message = "Low movement speed"
        else:
            movement_status.level = DiagnosticStatus.OK
        
        movement_status.values = [
            KeyValue(key="average_speed", value=f"{self.calculate_average_speed():.3f}"),
            KeyValue(key="is_stuck", value=str(self.is_stuck())),
            KeyValue(key="position_x", value=f"{self.current_pose.position.x:.2f}"),
            KeyValue(key="position_y", value=f"{self.current_pose.position.y:.2f}"),
        ]
        
        diag_array.status = [nav_status, movement_status]
        self.diagnostics_publisher.publish(diag_array)

    def publish_navigation_status(self):
        """Publish navigation status summary"""
        status_data = {
            'timestamp': time.time(),
            'navigation_status': self.navigation_status.value,
            'waypoint_status': self.waypoint_status,
            'rover_status': self.rover_status,
            'obstacle_detected': self.obstacle_detected,
            'is_stuck': self.is_stuck(),
            'current_position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            } if self.current_pose else None,
            'current_velocity': {
                'linear': math.sqrt(
                    self.current_velocity.linear.x**2 + 
                    self.current_velocity.linear.y**2
                ) if self.current_velocity else 0.0,
                'angular': abs(self.current_velocity.angular.z) if self.current_velocity else 0.0
            }
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.nav_status_publisher.publish(status_msg)

    def publish_performance_metrics(self):
        """Publish navigation performance metrics"""
        performance_data = {
            'timestamp': time.time(),
            'total_distance_traveled': self.total_distance_traveled,
            'successful_navigations': self.successful_navigations,
            'failed_navigations': self.failed_navigations,
            'navigation_efficiency': self.calculate_navigation_efficiency(),
            'obstacle_encounters': self.obstacle_encounters,
            'obstacle_frequency': self.get_obstacle_frequency(),
            'stuck_incidents': self.stuck_incidents,
            'average_speed': self.calculate_average_speed(),
            'uptime_hours': (time.time() - self.get_clock().now().seconds_nanoseconds()[0]) / 3600.0
        }
        
        # Check for performance issues
        if performance_data['navigation_efficiency'] < 0.5 and self.successful_navigations > 5:
            current_time = time.time()
            if current_time - self.last_performance_alert > 60.0:  # Alert once per minute
                self.publish_alert(
                    f"Low navigation efficiency: {performance_data['navigation_efficiency']:.2f}")
                self.last_performance_alert = current_time
        
        performance_msg = String()
        performance_msg.data = json.dumps(performance_data, indent=2)
        self.performance_publisher.publish(performance_msg)

    def publish_alert(self, alert_message):
        """Publish navigation alert"""
        alert_data = {
            'timestamp': time.time(),
            'level': 'warning',
            'message': alert_message,
            'context': {
                'navigation_status': self.navigation_status.value,
                'obstacle_detected': self.obstacle_detected,
                'is_stuck': self.is_stuck(),
                'position': {
                    'x': self.current_pose.position.x,
                    'y': self.current_pose.position.y
                } if self.current_pose else None
            }
        }
        
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data, indent=2)
        self.alerts_publisher.publish(alert_msg)
        
        self.get_logger().warn(f"Navigation Alert: {alert_message}")

def main(args=None):
    rclpy.init(args=args)
    
    navigation_monitor = NavigationMonitor()
    
    try:
        rclpy.spin(navigation_monitor)
    except KeyboardInterrupt:
        navigation_monitor.get_logger().info('Shutting down navigation monitor')
    finally:
        navigation_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
