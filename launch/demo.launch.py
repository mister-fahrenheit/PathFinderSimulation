#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='simple_warehouse.world',
        description='Gazebo world file to load'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='husky',
        description='Robot name (husky or turtlebot3)'
    )
    
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start x position'
    )
    
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start y position'
    )
    
    start_yaw_arg = DeclareLaunchArgument(
        'start_yaw',
        default_value='0.0',
        description='Start yaw angle'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal x position'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='3.0',
        description='Goal y position'
    )
    
    goal_yaw_arg = DeclareLaunchArgument(
        'goal_yaw',
        default_value='0.0',
        description='Goal yaw angle'
    )
    
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_ros2'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': world_file,
            'robot_name': robot_name,
            'x': start_x,
            'y': start_y,
            'z': '0.0',
            'yaw': start_yaw
        }.items()
    )
    
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('path_ros2'),
                        'launch',
                        'navigation.launch.py'
                    ])
                ])
            )
        ]
    )
    
    goal_publisher = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='path_ros2',
                executable='goal_publisher.py',
                name='goal_publisher',
                output='screen',
                arguments=[goal_x, goal_y, goal_yaw]
            )
        ]
    )
    
    rviz_launch = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('path_ros2'),
                    'rviz',
                    'navigation.rviz'
                ])],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        world_file_arg,
        robot_name_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        goal_x_arg,
        goal_y_arg,
        goal_yaw_arg,
        simulation_launch,
        navigation_launch,
        goal_publisher,
        rviz_launch,
    ])
