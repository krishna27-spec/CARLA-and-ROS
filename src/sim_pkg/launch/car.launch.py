"""Gazebo Classic + 4-wheel Ackermann car (models/ackermann_car).

Spawns the car on the cone track by default (world, x_pose, y_pose and yaw are
launch arguments; the start pose below matches cone_track.world). The plugin publishes /odom and listens on /cmd_vel
(Twist: linear.x = speed, angular.z = steering angle in rad). The relay
node is not started; send Twist to /cmd_vel directly.
Sensors: /scan (lidar), /imu, /camera/front/image_raw, /gps/fix and /gps/vel,
/joint_states (wheel encoders).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    sim_share = get_package_share_directory('sim_pkg')
    sdf = os.path.join(sim_share, 'models', 'ackermann_car', 'model.sdf')

    return LaunchDescription([
        # worlds reference model://traffic_cone from this package
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [os.path.join(sim_share, 'models'), ':',
             EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_share, 'worlds', 'cone_track.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-4.6'),
        DeclareLaunchArgument('y_pose', default_value='-3.5'),
        DeclareLaunchArgument('yaw', default_value='-0.172'),
        DeclareLaunchArgument('spawn_timeout', default_value='120'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # sensor frames (the sensors live in base_link at these offsets)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.20', '--z', '0.12', '--frame-id', 'base_link',
                        '--child-frame-id', 'lidar_link'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.15', '--z', '0.03', '--frame-id', 'base_link',
                        '--child-frame-id', 'imu_link'],
             parameters=[{'use_sim_time': True}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.31', '--z', '0.075', '--frame-id', 'base_link',
                        '--child-frame-id', 'camera_link'],
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.10', '--z', '0.12', '--frame-id', 'base_link',
                        '--child-frame-id', 'gps_link'],
             parameters=[{'use_sim_time': True}]),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'ackermann_car',
                '-file', sdf,
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', '0.01',
                '-Y', LaunchConfiguration('yaw'),
                '-timeout', LaunchConfiguration('spawn_timeout'),
            ],
            output='screen'),
    ])
