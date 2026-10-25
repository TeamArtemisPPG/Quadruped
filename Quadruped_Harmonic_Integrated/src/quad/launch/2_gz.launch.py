# #!/usr/bin/env python3

# import os

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     pkg_name = "quad"  # 🔴 CHANGE THIS
#     pkg_path = get_package_share_directory(pkg_name)

#     urdf_file = os.path.join(pkg_path, "urdf", "urdf.urdf")  # 🔴 CHANGE
#     controller_file = os.path.join(pkg_path, "config", "controllers.yaml")

#     with open(urdf_file, 'r') as infp:
#         robot_description = infp.read()

#     return LaunchDescription([

#         # -------- Start Gazebo --------
#         ExecuteProcess(
#             cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
#             output='screen'
#         ),

#         # -------- Robot State Publisher --------
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             parameters=[{'robot_description': robot_description}],
#             output='screen'
#         ),

#         # -------- ros2_control Node --------
#         # Node(
#         #     package='controller_manager',
#         #     executable='ros2_control_node',
#         #     parameters=[
#         #         {'robot_description': robot_description},
#         #         controller_file
#         #     ],
#         #     output='screen'
#         # ),

#         # -------- Spawn Robot --------
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=[
#                 '-topic', 'robot_description',
#                 '-entity', 'quadruped'
#             ],
#             output='screen'
#         ),

#         # -------- Load Controllers --------
#         ExecuteProcess(
#             cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#                  'joint_state_broadcaster'],
#             output='screen'
#         ),

#         ExecuteProcess(
#             cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#                  'joint_group_controller'],
#             output='screen'
#         ),

#         # -------- Your Gazebo IK Node --------
#         # Node(
#         #     package=pkg_name,
#         #     executable='gz_ik',  # 🔴 match your file name
#         #     output='screen'
#         # ),
#     ])
#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "quad"
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_path, "urdf", "urdf.urdf")

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        # -------- Start Gazebo --------
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # -------- Robot State Publisher --------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # -------- DELAYED SPAWN (5 sec) --------
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'quadruped',

                        # 🔥 Spawn position
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1'   # lift above ground
                    ],
                    output='screen'
                )
            ]
        ),

        # -------- Controllers (optional delay also recommended) --------
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'control', 'load_controller',
                        '--set-state', 'active',
                        'joint_state_broadcaster'
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'control', 'load_controller',
                        '--set-state', 'active',
                        'joint_group_controller'
                    ],
                    output='screen'
                )
            ]
        ),

    ])