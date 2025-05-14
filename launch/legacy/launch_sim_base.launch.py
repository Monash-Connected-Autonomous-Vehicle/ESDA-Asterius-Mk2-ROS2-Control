import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'esda_vehicle_control'

    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # Launch joystick if needed
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Twist mux node
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Start Ignition Gazebo with your custom world
    ignition = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-v', '4',
            os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.world')
        ],
        output='screen'
    )

    # Spawn robot in Ignition from robot_description topic
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-entity', 'my_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Controller manager spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )

    # Delay controller spawners until after spawn_entity finishes
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner, joint_broad_spawner]
        )
    )

    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        ignition,
        spawn_entity,
        delayed_diff_drive_spawner
    ])
