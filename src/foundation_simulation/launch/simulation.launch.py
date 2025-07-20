#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Simulation launch file that starts Gazebo and spawns the robot.
    """
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World file to load in Gazebo'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='foundation_robot',
        description='Name of the robot in simulation'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot in simulation'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot in simulation'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Z position of the robot in simulation'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Whether to start Gazebo GUI'
    )
    
    # Get launch configurations
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_gui = LaunchConfiguration('use_gui')
    
    # Paths
    world_file = PathJoinSubstitution([
        FindPackageShare('foundation_simulation'),
        'worlds',
        world
    ])
    
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('foundation_description'),
        'urdf',
        'foundation_robot.urdf.xacro'
    ])
    
    # Start Gazebo
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gui)
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_file,
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )
    
    # Include basic system launch
    basic_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('foundation_bringup'),
                'launch',
                'basic_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_simulation': 'true',
            'robot_id': robot_name,
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        robot_name_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_gui_arg,
        
        # Gazebo
        gazebo_server,
        gazebo_client,
        
        # Robot
        robot_state_publisher,
        spawn_robot,
        
        # Basic system
        basic_system_launch,
    ])
