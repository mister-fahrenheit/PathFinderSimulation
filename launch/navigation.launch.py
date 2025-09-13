#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    perception_node = Node(
        package='perception_node',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    planner_node = Node(
        package='planner_node',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    control_node = Node(
        package='control_node',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    robot_pose_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_pose_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    odom_to_pose = Node(
        package='path_ros2',
        executable='odom_to_pose.py',
        name='odom_to_pose',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        perception_node,
        planner_node,
        control_node,
        robot_pose_publisher,
        odom_to_pose,
    ])
