import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_g1 = get_package_share_directory('g1_description')
    urdf_file = os.path.join(pkg_g1, 'urdf', 'g1_29dof_mode_16.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 1. Kill any stale Gazebo processes first
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 gzserver; pkill -9 gzclient; sleep 1'],
        output='screen'
    )

    # 2. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'paused': 'false'
            }.items()
    )

    # 3. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 4. Spawn robot at 3 seconds
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file,
                    '-entity', 'g1',
                    '-x', '0', '-y', '0', '-z', '0.79',
                ],
                output='screen'
            )
        ]
    )

    # 5. Load joint state broadcaster at 5 seconds
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # 6. Load effort controller at 6 seconds
    effort_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['effort_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # 7. Start standing controller at 7 seconds — before robot falls too far
    standing_controller = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='g1_control',
                executable='standing_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # kill_gazebo,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        effort_controller_spawner,
        standing_controller,
    ])