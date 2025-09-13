#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


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
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Initial z position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle'
    )
    
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('path_ros2'),
                'worlds',
                world_file
            ])
        }.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': get_robot_description(robot_name)
        }]
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        output='screen'
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])


def get_robot_description(robot_name):
    if robot_name == 'husky':
        return get_husky_description()
    elif robot_name == 'turtlebot3':
        return get_turtlebot3_description()
    else:
        return get_husky_description()  # Default


def get_husky_description():
    return """
    <?xml version="1.0"?>
    <robot name="husky">
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="1.0 0.6 0.2"/>
                </geometry>
                <material name="blue">
                    <color rgba="0.0 0.0 1.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <box size="1.0 0.6 0.2"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="50.0"/>
                <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            </inertial>
        </link>
        
        <link name="laser_link">
            <visual>
                <geometry>
                    <cylinder radius="0.05" length="0.1"/>
                </geometry>
                <material name="red">
                    <color rgba="1.0 0.0 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="0.05" length="0.1"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.1"/>
                <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
            </inertial>
        </link>
        
        <joint name="laser_joint" type="fixed">
            <parent link="base_link"/>
            <child link="laser_link"/>
            <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
        </joint>
        
        <link name="left_wheel">
            <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
                <material name="black">
                    <color rgba="0.0 0.0 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
            </inertial>
        </link>
        
        <link name="right_wheel">
            <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
                <material name="black">
                    <color rgba="0.0 0.0 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="0.1" length="0.05"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
            </inertial>
        </link>
        
        <joint name="left_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="left_wheel"/>
            <origin xyz="0.0 0.3 0.0" rpy="-1.57 0 0"/>
            <axis xyz="0 0 1"/>
        </joint>
        
        <joint name="right_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="right_wheel"/>
            <origin xyz="0.0 -0.3 0.0" rpy="-1.57 0 0"/>
            <axis xyz="0 0 1"/>
        </joint>
        
        <gazebo reference="laser_link">
            <sensor type="ray" name="laser_scanner">
                <pose>0 0 0 0 0 0</pose>
                <visualize>false</visualize>
                <update_rate>40</update_rate>
                <ray>
                    <scan>
                        <horizontal>
                            <samples>720</samples>
                            <resolution>1</resolution>
                            <min_angle>-3.14159</min_angle>
                            <max_angle>3.14159</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>0.10</min>
                        <max>30.0</max>
                        <resolution>0.01</resolution>
                    </range>
                </ray>
                <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
                    <ros>
                        <remapping>~/out:=scan</remapping>
                    </ros>
                    <output_type>sensor_msgs/LaserScan</output_type>
                    <frame_name>laser_link</frame_name>
                </plugin>
            </sensor>
        </gazebo>
        
        <gazebo reference="left_wheel">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <kp>1000000.0</kp>
            <kd>100.0</kd>
            <minDepth>0.001</minDepth>
            <maxVel>1.0</maxVel>
            <fdir1>1 0 0</fdir1>
        </gazebo>
        
        <gazebo reference="right_wheel">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <kp>1000000.0</kp>
            <kd>100.0</kd>
            <minDepth>0.001</minDepth>
            <maxVel>1.0</maxVel>
            <fdir1>1 0 0</fdir1>
        </gazebo>
        
        <gazebo>
            <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
                <ros>
                    <namespace>/</namespace>
                </ros>
                <left_joint>left_wheel_joint</left_joint>
                <right_joint>right_wheel_joint</right_joint>
                <wheel_separation>0.6</wheel_separation>
                <wheel_diameter>0.2</wheel_diameter>
                <wheel_acceleration>1.0</wheel_acceleration>
                <wheel_torque>20</wheel_torque>
                <command_topic>cmd_vel</command_topic>
                <odometry_topic>odom</odometry_topic>
                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>
                <publish_wheel_tf>true</publish_wheel_tf>
                <odometry_source>world</odometry_source>
                <legacy_mode>false</legacy_mode>
            </plugin>
        </gazebo>
    </robot>
    """


def get_turtlebot3_description():
    return """
    <?xml version="1.0"?>
    <robot name="turtlebot3">
        <link name="base_link">
            <visual>
                <geometry>
                    <cylinder radius="0.1" length="0.2"/>
                </geometry>
                <material name="blue">
                    <color rgba="0.0 0.0 1.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="0.1" length="0.2"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="10.0"/>
                <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
            </inertial>
        </link>
        
        <link name="laser_link">
            <visual>
                <geometry>
                    <cylinder radius="0.03" length="0.05"/>
                </geometry>
                <material name="red">
                    <color rgba="1.0 0.0 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="0.03" length="0.05"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.05"/>
                <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
            </inertial>
        </link>
        
        <joint name="laser_joint" type="fixed">
            <parent link="base_link"/>
            <child link="laser_link"/>
            <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
        </joint>
        
        <gazebo reference="laser_link">
            <sensor type="ray" name="laser_scanner">
                <pose>0 0 0 0 0 0</pose>
                <visualize>false</visualize>
                <update_rate>40</update_rate>
                <ray>
                    <scan>
                        <horizontal>
                            <samples>360</samples>
                            <resolution>1</resolution>
                            <min_angle>-3.14159</min_angle>
                            <max_angle>3.14159</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>0.10</min>
                        <max>30.0</max>
                        <resolution>0.01</resolution>
                    </range>
                </ray>
                <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
                    <ros>
                        <remapping>~/out:=scan</remapping>
                    </ros>
                    <output_type>sensor_msgs/LaserScan</output_type>
                    <frame_name>laser_link</frame_name>
                </plugin>
            </sensor>
        </gazebo>
        
        <gazebo>
            <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
                <ros>
                    <namespace>/</namespace>
                </ros>
                <left_joint>left_wheel_joint</left_joint>
                <right_joint>right_wheel_joint</right_joint>
                <wheel_separation>0.16</wheel_separation>
                <wheel_diameter>0.066</wheel_diameter>
                <wheel_acceleration>1.0</wheel_acceleration>
                <wheel_torque>20</wheel_torque>
                <command_topic>cmd_vel</command_topic>
                <odometry_topic>odom</odometry_topic>
                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>
                <publish_wheel_tf>true</publish_wheel_tf>
                <odometry_source>world</odometry_source>
                <legacy_mode>false</legacy_mode>
            </plugin>
        </gazebo>
    </robot>
    """
