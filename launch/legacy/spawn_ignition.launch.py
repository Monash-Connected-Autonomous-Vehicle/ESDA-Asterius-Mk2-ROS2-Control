import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = 'esda_vehicle_control'
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    xacro_file = os.path.join(
        get_package_share_directory(pkg),
        'description','robot.urdf.xacro'
    )
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    robot_description = ParameterValue(robot_description_config, value_type=str)
    # 1) Launch Ignition Gazebo server+client
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch','gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'ign_args': '-r empty.sdf',   # replace with your world if needed
            'verbose':'true'
        }.items()
    )

    # 2) Bring up the ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(
                get_package_share_directory(pkg),
                'config','diff_controllers.yaml'
            )
        ],
        output='screen'
    )

    # 3) Spawn your robot from /robot_description
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'diff_bot'
        ],
        output='screen'
    )

    # 4) Load and start controllers
    js_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster','--controller-manager','/controller_manager']
    )
    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller','--controller-manager','/controller_manager']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',    default_value='false'),
        DeclareLaunchArgument('use_ros2_control',default_value='true'),
        ign_launch,
        control_node,
        spawn_node,
        js_spawner,
        diff_spawner
    ])
