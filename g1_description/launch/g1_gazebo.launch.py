import os
import atexit
import tempfile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_g1 = get_package_share_directory('g1_description')
    urdf_file = os.path.join(pkg_g1, 'urdf', 'g1_29dof_mode_16.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    controllers_yaml = os.path.join(pkg_g1, 'config', 'controllers.yaml')
    robot_description = robot_description.replace('CONTROLLERS_YAML', controllers_yaml)
    robot_description = robot_description.replace('package://g1_description', pkg_g1)

    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_description)
    tmp_urdf.close()
    atexit.register(os.unlink, tmp_urdf.name)

    pkg_g1_worlds = os.path.join(pkg_g1, 'worlds', 'g1_stable.world')

    # ── Gazebo (starts PAUSED, custom 2 ms world) ───────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'paused': 'true',
            'world': pkg_g1_worlds,
        }.items()
    )

    # ── Robot state publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # t=3s  Spawn robot
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', tmp_urdf.name,
                    '-entity', 'g1',
                    '-x', '0', '-y', '0', '-z', '0.8',
                ],
                output='screen'
            )
        ]
    )

    repause = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/pause_physics',
                    'std_srvs/srv/Empty', '{}'
                ],
                output='screen'
            )
        ]
    )

    # t=5s  Joint state broadcaster
    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager', '/controller_manager',
                           '--controller-manager-timeout', '30',],
                output='screen'
            )
        ]
    )

    # t=6s  Effort controller
    effort_controller_spawner = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['effort_controller',
                           '--controller-manager', '/controller_manager',
                           '--controller-manager-timeout', '30',],
                output='screen'
            )
        ]
    )

    # t=6.5s  Standing controller — starts while Gazebo is still paused.
    #         Joint states are published even while paused, so the controller
    #         will receive all joint positions and be fully ready before
    #         physics starts.
    standing_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='g1_control',
                executable='standing_controller',
                output='screen'
            )
        ]
    )

    # t=9s  Unpause — controller has had enough time to initialise and
    #        will be publishing torques before physics starts moving.
    unpause = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/unpause_physics',
                    'std_srvs/srv/Empty', '{}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        repause,
        joint_state_broadcaster_spawner,
        effort_controller_spawner,
        standing_controller,
        unpause,
    ])