#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Basic system launch file that starts the core foundation components.
    """
    
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='foundation_robot',
        description='Unique identifier for this robot instance'
    )
    
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Whether to start in simulation mode'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Get launch configurations
    robot_id = LaunchConfiguration('robot_id')
    use_simulation = LaunchConfiguration('use_simulation')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')
    
    # System monitor node
    system_monitor_node = Node(
        package='foundation_common',
        executable='system_monitor_node.py',
        name='system_monitor',
        parameters=[{
            'robot_id': robot_id,
            'publish_rate': 2.0,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )
    
    # Robot state publisher
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('foundation_description'),
        'urdf',
        'foundation_robot.urdf.xacro'
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_file,
            'use_sim_time': use_simulation,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )
    
    # Joint state publisher (for manual testing)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_simulation),
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )
    
    # TF static transforms
    tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('foundation_bringup'),
        'config',
        'foundation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_id_arg,
        use_simulation_arg,
        use_rviz_arg,
        log_level_arg,
        
        # Nodes
        system_monitor_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        tf_base_link,
        rviz_node,
    ])
