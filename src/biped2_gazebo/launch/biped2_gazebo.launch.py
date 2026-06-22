import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('biped2_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_desc, 'urdf', 'biped2.urdf')
    if not os.path.isfile(urdf_file):
        raise RuntimeError(
            f'Missing {urdf_file}. Run: colcon build --symlink-install '
            '--packages-select biped2_description'
        )

    default_world = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str,
    )

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    stop_script = os.path.abspath(
        os.path.join(pkg_desc, '..', '..', '..', '..', 'scripts', 'stop_gazebo.sh')
    )

    cleanup_gazebo = ExecuteProcess(
        cmd=['bash', stop_script],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description,
            }
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'biped2',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-timeout',
            '120',
        ],
        output='screen',
    )

    foot_fk_publisher = Node(
        package='biped2_kinematics',
        executable='foot_fk_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    foot_lift_demo = Node(
        package='biped2_kinematics',
        executable='foot_lift_demo',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_frame_id': 'world',
                'lift_height': 0.10,
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            cleanup_gazebo,
            TimerAction(period=2.0, actions=[gazebo, robot_state_publisher]),
            TimerAction(period=7.0, actions=[spawn_entity]),
            TimerAction(period=12.0, actions=[foot_fk_publisher, foot_lift_demo]),
        ]
    )
