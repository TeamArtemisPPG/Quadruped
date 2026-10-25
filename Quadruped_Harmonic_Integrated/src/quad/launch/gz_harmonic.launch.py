#!/usr/bin/env python3
"""Launch the quadruped in Gazebo Harmonic (gz sim) with ros2_control (ROS 2 Jazzy)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('quad')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Insert the real installed path of controllers yaml into the URDF
    controllers_yaml = os.path.join(pkg, 'config', 'controllers_harmonic.yaml')
    with open(os.path.join(pkg, 'urdf', 'urdf_harmonic.urdf'), 'r') as f:
        robot_description = f.read().replace('__CONTROLLERS_YAML__', controllers_yaml)

    world = os.path.join(pkg, 'worlds', 'ground.sdf')

    args = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='6.0',
                              description='Spawn height (m); drop onto ground'),
        DeclareLaunchArgument('gz_args', default_value=f'-r -v 2 {world}',
                              description='Args for gz sim ("-s" added for headless)'),
    ]

    # Lets gz resolve package://quad/meshes/... (needs the dir that contains "quad/")
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'quadruped', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z')],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
    )
    jgc = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_group_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
    )

    # Controllers start only after the robot is spawned (gz plugin hosts controller_manager)
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[jgc]))

    # Move legs to the stance pose once the controller is active
    # (same pose walk_gait.py ramps from; right legs 1,4 / left legs 2,3)
    r = [0.0, -1.40020, -0.46137]
    l = [0.0, 1.40020, -0.46137]
    stance = r + l + l + r          # legs 1,2,3,4
    stance_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '--qos-reliability', 'reliable',
             '/joint_group_controller/commands',
             'std_msgs/msg/Float64MultiArray', '{data: %s}' % stance],
        output='screen',
    )
    after_jgc = RegisterEventHandler(
        OnProcessExit(target_action=jgc, on_exit=[stance_cmd]))

    return LaunchDescription(args + [
        set_resource_path, gz_sim, rsp, clock_bridge,
        TimerAction(period=8.0, actions=[spawn]), after_spawn, after_jsb,
        after_jgc])