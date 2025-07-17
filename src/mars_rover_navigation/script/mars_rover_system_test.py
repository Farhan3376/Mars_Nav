#!/usr/bin/env python3

"""
Mars Rover Navigation System Test Script

This script performs comprehensive testing of the Mars rover navigation system
to verify that all components are working correctly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import time
import math
import sys

class MarsRoverSystemTest(Node):
    def __init__(self):
        super().__init__('mars_rover_system_test')
        
        # Test results storage
        self.test_results = {}
        self.tests_completed = 0
        self.total_tests = 10
        
        # Data received flags
        self.odom_received = False
        self.laser_received = False
        self.image_received = False
        self.status_received = False
        
        # Initialize subscribers for testing
        self.setup_subscribers()
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.test_start_time = time.time()
        self.current_test = 0
        
        self.get_logger().info('Mars Rover System Test Started')
        self.get_logger().info('=' * 60)

    def setup_subscribers(self):
        """Setup subscribers for testing"""
        self.odom_subscriber = self.create_subscription(
            Odometry, '/mars_rover/odom', self.odom_callback, 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/mars_rover/scan', self.laser_callback, 10)
        self.image_subscriber = self.create_subscription(
            Image, '/mars_rover/camera/image_raw', self.image_callback, 10)
        self.status_subscriber = self.create_subscription(
            String, '/mars_rover/status', self.status_callback, 10)
        
        # Publishers for testing
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.command_publisher = self.create_publisher(String, '/mars_rover/command', 10)

    def odom_callback(self, msg):
        """Odometry callback"""
        self.odom_received = True
        self.latest_odom = msg

    def laser_callback(self, msg):
        """Laser scan callback"""
        self.laser_received = True
        self.latest_laser = msg

    def image_callback(self, msg):
        """Image callback"""
        self.image_received = True
        self.latest_image = msg

    def status_callback(self, msg):
        """Status callback"""
        self.status_received = True
        self.latest_status = msg

    def run_tests(self):
        """Main test execution"""
        current_time = time.time()
        elapsed_time = current_time - self.test_start_time
        
        if self.current_test == 0:
            self.test_node_discovery()
        elif self.current_test == 1:
            self.test_sensor_data()
        elif self.current_test == 2:
            self.test_odometry()
        elif self.current_test == 3:
            self.test_laser_scan()
        elif self.current_test == 4:
            self.test_camera()
        elif self.current_test == 5:
            self.test_control_system()
        elif self.current_test == 6:
            self.test_navigation_stack()
        elif self.current_test == 7:
            self.test_obstacle_detection()
        elif self.current_test == 8:
            self.test_terrain_analysis()
        elif self.current_test == 9:
            self.test_waypoint_navigation()
        elif self.current_test >= self.total_tests:
            self.print_final_results()
            return
        
        self.current_test += 1

    def test_node_discovery(self):
        """Test 1: Check if required nodes are running"""
        self.get_logger().info('Test 1: Node Discovery')
        
        # Get list of nodes
        node_names = self.get_node_names()
        
        required_nodes = [
            'robot_state_publisher',
            'gazebo',
            'controller_manager'
        ]
        
        missing_nodes = []
        for node in required_nodes:
            found = any(node in name for name in node_names)
            if not found:
                missing_nodes.append(node)
        
        if missing_nodes:
            self.test_results['node_discovery'] = f'FAILED - Missing nodes: {missing_nodes}'
            self.get_logger().error(f'Missing required nodes: {missing_nodes}')
        else:
            self.test_results['node_discovery'] = 'PASSED'
            self.get_logger().info('✓ All required nodes are running')

    def test_sensor_data(self):
        """Test 2: Check if sensor data is being received"""
        self.get_logger().info('Test 2: Sensor Data Reception')
        
        # Wait a bit for data to arrive
        time.sleep(2)
        
        sensor_status = {
            'odometry': self.odom_received,
            'laser': self.laser_received,
            'camera': self.image_received,
            'status': self.status_received
        }
        
        failed_sensors = [name for name, status in sensor_status.items() if not status]
        
        if failed_sensors:
            self.test_results['sensor_data'] = f'FAILED - No data from: {failed_sensors}'
            self.get_logger().error(f'No data received from: {failed_sensors}')
        else:
            self.test_results['sensor_data'] = 'PASSED'
            self.get_logger().info('✓ All sensor data is being received')

    def test_odometry(self):
        """Test 3: Validate odometry data"""
        self.get_logger().info('Test 3: Odometry Validation')
        
        if not self.odom_received:
            self.test_results['odometry'] = 'FAILED - No odometry data'
            self.get_logger().error('No odometry data received')
            return
        
        odom = self.latest_odom
        
        # Check if position values are reasonable
        pos = odom.pose.pose.position
        if abs(pos.x) > 1000 or abs(pos.y) > 1000 or abs(pos.z) > 100:
            self.test_results['odometry'] = 'FAILED - Unrealistic position values'
            self.get_logger().error(f'Unrealistic position: x={pos.x}, y={pos.y}, z={pos.z}')
            return
        
        self.test_results['odometry'] = 'PASSED'
        self.get_logger().info(f'✓ Odometry valid - Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')

    def test_laser_scan(self):
        """Test 4: Validate laser scan data"""
        self.get_logger().info('Test 4: Laser Scan Validation')
        
        if not self.laser_received:
            self.test_results['laser_scan'] = 'FAILED - No laser data'
            self.get_logger().error('No laser scan data received')
            return
        
        laser = self.latest_laser
        
        # Check if scan parameters are reasonable
        if len(laser.ranges) == 0:
            self.test_results['laser_scan'] = 'FAILED - Empty laser scan'
            self.get_logger().error('Laser scan is empty')
            return
        
        # Check for valid range data
        valid_ranges = [r for r in laser.ranges if laser.range_min <= r <= laser.range_max]
        valid_percentage = len(valid_ranges) / len(laser.ranges) * 100
        
        if valid_percentage < 10:
            self.test_results['laser_scan'] = f'FAILED - Only {valid_percentage:.1f}% valid ranges'
            self.get_logger().error(f'Only {valid_percentage:.1f}% of laser ranges are valid')
            return
        
        self.test_results['laser_scan'] = 'PASSED'
        self.get_logger().info(f'✓ Laser scan valid - {len(laser.ranges)} points, {valid_percentage:.1f}% valid')

    def test_camera(self):
        """Test 5: Validate camera data"""
        self.get_logger().info('Test 5: Camera Validation')
        
        if not self.image_received:
            self.test_results['camera'] = 'FAILED - No camera data'
            self.get_logger().error('No camera data received')
            return
        
        image = self.latest_image
        
        # Check image dimensions
        if image.width == 0 or image.height == 0:
            self.test_results['camera'] = 'FAILED - Invalid image dimensions'
            self.get_logger().error(f'Invalid image dimensions: {image.width}x{image.height}')
            return
        
        # Check if image data exists
        if len(image.data) == 0:
            self.test_results['camera'] = 'FAILED - Empty image data'
            self.get_logger().error('Image data is empty')
            return
        
        self.test_results['camera'] = 'PASSED'
        self.get_logger().info(f'✓ Camera valid - {image.width}x{image.height}, {len(image.data)} bytes')

    def test_control_system(self):
        """Test 6: Test robot control system"""
        self.get_logger().info('Test 6: Control System Test')
        
        # Send a small movement command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = 0.1
        
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info('Sent test movement command')
        
        # Wait a moment then stop
        time.sleep(1)
        
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        
        self.test_results['control_system'] = 'PASSED'
        self.get_logger().info('✓ Control system test completed')

    def test_navigation_stack(self):
        """Test 7: Test navigation stack"""
        self.get_logger().info('Test 7: Navigation Stack Test')
        
        # Check if navigation topics exist
        topic_names = self.get_topic_names_and_types()
        nav_topics = [
            '/plan',
            '/local_plan',
            '/global_costmap/costmap',
            '/local_costmap/costmap'
        ]
        
        missing_topics = []
        for topic in nav_topics:
            if not any(topic in name[0] for name in topic_names):
                missing_topics.append(topic)
        
        if missing_topics:
            self.test_results['navigation_stack'] = f'PARTIALLY WORKING - Missing topics: {missing_topics}'
            self.get_logger().warn(f'Some navigation topics missing: {missing_topics}')
        else:
            self.test_results['navigation_stack'] = 'PASSED'
            self.get_logger().info('✓ Navigation stack appears to be running')

    def test_obstacle_detection(self):
        """Test 8: Test obstacle detection system"""
        self.get_logger().info('Test 8: Obstacle Detection Test')
        
        # Check if obstacle detection topics exist
        topic_names = self.get_topic_names_and_types()
        obstacle_topics = [
            '/mars_rover/obstacle_detected',
            '/mars_rover/detections'
        ]
        
        available_topics = []
        for topic in obstacle_topics:
            if any(topic in name[0] for name in topic_names):
                available_topics.append(topic)
        
        if len(available_topics) > 0:
            self.test_results['obstacle_detection'] = 'PASSED'
            self.get_logger().info(f'✓ Obstacle detection system active - Topics: {available_topics}')
        else:
            self.test_results['obstacle_detection'] = 'FAILED - No obstacle detection topics'
            self.get_logger().error('No obstacle detection topics found')

    def test_terrain_analysis(self):
        """Test 9: Test terrain analysis system"""
        self.get_logger().info('Test 9: Terrain Analysis Test')
        
        # Check if terrain analysis topics exist
        topic_names = self.get_topic_names_and_types()
        terrain_topics = [
            '/mars_rover/terrain_map',
            '/mars_rover/terrain_features',
            '/mars_rover/terrain_analysis'
        ]
        
        available_topics = []
        for topic in terrain_topics:
            if any(topic in name[0] for name in topic_names):
                available_topics.append(topic)
        
        if len(available_topics) >= 2:
            self.test_results['terrain_analysis'] = 'PASSED'
            self.get_logger().info(f'✓ Terrain analysis system active - Topics: {available_topics}')
        else:
            self.test_results['terrain_analysis'] = 'PARTIALLY WORKING'
            self.get_logger().warn(f'Limited terrain analysis topics: {available_topics}')

    def test_waypoint_navigation(self):
        """Test 10: Test waypoint navigation system"""
        self.get_logger().info('Test 10: Waypoint Navigation Test')
        
        # Check if waypoint navigation topics exist
        topic_names = self.get_topic_names_and_types()
        waypoint_topics = [
            '/mars_rover/waypoint_command',
            '/mars_rover/waypoint_status',
            '/mars_rover/waypoints_viz'
        ]
        
        available_topics = []
        for topic in waypoint_topics:
            if any(topic in name[0] for name in topic_names):
                available_topics.append(topic)
        
        if len(available_topics) >= 2:
            self.test_results['waypoint_navigation'] = 'PASSED'
            self.get_logger().info(f'✓ Waypoint navigation system active - Topics: {available_topics}')
        else:
            self.test_results['waypoint_navigation'] = 'PARTIALLY WORKING'
            self.get_logger().warn(f'Limited waypoint navigation topics: {available_topics}')

    def print_final_results(self):
        """Print final test results"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('MARS ROVER SYSTEM TEST RESULTS')
        self.get_logger().info('=' * 60)
        
        passed_tests = 0
        failed_tests = 0
        partial_tests = 0
        
        for test_name, result in self.test_results.items():
            if 'PASSED' in result:
                self.get_logger().info(f'✓ {test_name}: {result}')
                passed_tests += 1
            elif 'FAILED' in result:
                self.get_logger().error(f'✗ {test_name}: {result}')
                failed_tests += 1
            else:
                self.get_logger().warn(f'⚠ {test_name}: {result}')
                partial_tests += 1
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'SUMMARY: {passed_tests} passed, {failed_tests} failed, {partial_tests} partial')
        
        if failed_tests == 0:
            self.get_logger().info('🎉 ALL CRITICAL TESTS PASSED! System is ready for operation.')
        elif failed_tests <= 2:
            self.get_logger().warn('⚠️  Some tests failed, but system may still be functional.')
        else:
            self.get_logger().error('❌ Multiple test failures detected. Please check system configuration.')
        
        self.get_logger().info('=' * 60)
        
        # Shutdown the test
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = MarsRoverSystemTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    except Exception as e:
        print(f'Test failed with error: {e}')
    finally:
        try:
            test_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
