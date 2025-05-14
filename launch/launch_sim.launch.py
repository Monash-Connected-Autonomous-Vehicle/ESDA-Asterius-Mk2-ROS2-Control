import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    

    # Define the input parameters
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    pkg = 'esda_vehicle_control'

    # ROS Controller Files:
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(pkg),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # --- 2) Launch Ignition sim server + client ---
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py'
          )
        ),
        launch_arguments={
          'ign_args': '-r empty.sdf',  # or your .sdf world
          'verbose': 'true'
        }.items()
    )
    
    
    # --- 4) Spawn the robot_description into Ignition ---
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",  "my_robot_1",
            "-topic", "robot_description",
            "-allow_renaming", "true",
            "--ros-args",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- 5) Load and start your controllers ---
    # Controller manager spawners
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file', robot_controllers,
            #'--ros-args', '-r', '/diff_drive_base_controller/cmd_vel:=/cmd_vel'
        ],
        output='screen'
    )

    # --- 6) Set up the Ros Gazebo Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    return LaunchDescription([
      DeclareLaunchArgument('use_sim_time',    default_value='true'),
      DeclareLaunchArgument('use_ros2_control',default_value='true'),
      rsp,
      bridge,
      ign_launch,
      spawn_entity,
      joint_broad_spawner,
      diff_drive_spawner,
    ])
