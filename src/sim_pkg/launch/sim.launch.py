"""Gazebo Classic + TurtleBot3 bringup.

Differences from turtlebot3_gazebo/turtlebot3_world.launch.py:
  * the robot spawner waits up to `spawn_timeout` s (Gazebo here takes ~30 s
    to start, longer than the stock spawner's 30 s)
  * start pose, world and GUI are launch arguments; defaults are the
    obstacle course (worlds/obstacle_course.world) and its start marker
  * starts cmd_vel_relay: controllers publish TwistStamped on
    /cmd_vel_stamped, the relay forwards Twist to /cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node


def generate_launch_description():
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    sim_share = get_package_share_directory('sim_pkg')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')

    sdf = PathJoinSubstitution(
        [tb3_share, 'models', ['turtlebot3_', model], 'model.sdf'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_share, 'worlds', 'obstacle_course.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('model', default_value='burger'),
        DeclareLaunchArgument('x_pose', default_value='-3.5'),
        DeclareLaunchArgument('y_pose', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_timeout', default_value='120'),

        # robot_state_publisher.launch.py reads this from the environment
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_share, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', model,
                '-file', sdf,
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', '0.01',
                '-Y', LaunchConfiguration('yaw'),
                '-timeout', LaunchConfiguration('spawn_timeout'),
            ],
            output='screen'),

        Node(
            package='controls_pkg',
            executable='cmd_vel_relay',
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
