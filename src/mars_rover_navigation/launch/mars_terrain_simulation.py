#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_mars_rover = FindPackageShare(package='mars_rover_navigation').find('mars_rover_navigation')
    
    # Paths
    urdf_file = os.path.join(pkg_mars_rover, 'urdf', 'mars_rover.urdf.xacro')
    world_file = os.path.join(pkg_mars_rover, 'worlds', 'realistic_mars_terrain.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file,
        description='World file to load')
    
    # Robot description
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Start Gazebo with Mars terrain
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_name],
        output='screen'
    )
    
    # ROS-Gazebo bridge with additional topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'
        ],
        output='screen'
    )
    
    # Spawn robot in Mars terrain
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mars_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'  # Spawn higher to account for terrain
        ],
        output='screen'
    )

    # Test navigator node
    test_navigator_node = Node(
        package='mars_rover_navigation',
        executable='autonomous_navigator',
        name='autonomous_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(spawn_robot)
    ld.add_action(test_navigator_node)

    return ld
