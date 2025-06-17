#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    TURTLEBOT3_MODEL = 'burger'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 你的包路径和自定义world文件路径
    my_pkg = 'turtlebot3_multi_robot'
    world_path = os.path.join(get_package_share_directory(my_pkg), 'worlds', 'multi_empty_world.world')

    # Gazebo ROS 包路径
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Turtlebot3 机器人模型URDF和模型路径
    urdf_path = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')
    model_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', f'turtlebot3_{TURTLEBOT3_MODEL}', 'model.sdf')

    # 声明参数
    declared_arguments = [
        DeclareLaunchArgument('world', default_value=world_path, description='Full path to world file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
    ]

    # 启动 gzserver，传入 world 参数
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 启动 gzclient
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
    )

    # 启动 robot_state_publisher 发布TF，使用URDF文件
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_path]
    )

    # Spawn turtlebot3模型到gazebo中
    spawn_turtlebot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', model_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn_turtlebot3,
        ]
    )

