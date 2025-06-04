#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package Directories
    pkg_share = get_package_share_directory('my_robot_py')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }])
    
    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world_file]}.items())
    
    # Spawn Robot
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'my_robot'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    # Bridge ROS topics and Gazebo messages
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                  '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add any actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    
    return ld