"""
walking_controller.launch.py
─────────────────────────────
Integrates with the existing g1_description Gazebo Classic setup.

Sequence:
  t=0s   Gazebo Classic starts (paused)
  t=0s   robot_state_publisher starts
  t=3s   G1 spawned in Gazebo
  t=3.5s Gazebo re-paused for controller setup
  t=4s   joint_state_broadcaster spawned
  t=4.5s joint_trajectory_controller spawned  ← replaces effort_controller
  t=5s   standing_controller starts (moves robot to nominal stance)
  t=6s   Gazebo unpaused
  t=8s   footstep_planner + walking_controller start
"""

import os
import atexit
import tempfile

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_g1         = get_package_share_directory('g1_description')
    pkg_zmp        = get_package_share_directory('g1_zmp_walking')

    # ── URDF ──────────────────────────────────────────────────────────────────
    # Use the same URDF your existing launch file uses
    urdf_file = os.path.join(pkg_g1, 'urdf', 'g1_29dof_mode_16.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    controllers_yaml = os.path.join(pkg_g1, 'config', 'controllers.yaml')
    robot_description = robot_description.replace('CONTROLLERS_YAML', controllers_yaml)
    robot_description = robot_description.replace('package://g1_description', pkg_g1)

    # Write patched URDF to a temp file (same as your existing launch)
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_description)
    tmp_urdf.close()
    atexit.register(os.unlink, tmp_urdf.name)

    # ── 1. Gazebo Classic (starts paused) ─────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'paused':  'true',
        }.items()
    )

    # ── 2. Robot state publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}],
    )

    # ── t=3s: Spawn robot ──────────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package    = 'gazebo_ros',
                executable = 'spawn_entity.py',
                arguments  = [
                    '-file',   tmp_urdf.name,
                    '-entity', 'g1',
                    '-x', '0', '-y', '0', '-z', '0.8',
                ],
                output='screen',
            )
        ],
    )

    force_pause = TimerAction(
        period=3.2,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                    '/pause_physics', 'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )
    # ── t=3.5s: Re-pause while controllers load ────────────────────────────────
    repause = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/pause_physics', 'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )

    # ── t=4s: joint_state_broadcaster ─────────────────────────────────────────
    spawn_jsb = TimerAction(
        period=4.0,
        actions=[
            Node(
                package    = 'controller_manager',
                executable = 'spawner',
                arguments  = [
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            )
        ],
    )

    # ── t=4.5s: joint_trajectory_controller ───────────────────────────────────
    # This is what walking_controller_node publishes to.
    # Make sure your controllers.yaml includes a joint_trajectory_controller
    # entry (see config/g1_controllers.yaml for the definition).
    spawn_jtc = TimerAction(
        period=4.5,
        actions=[
            Node(
                package    = 'controller_manager',
                executable = 'spawner',
                arguments  = [
                    # 'joint_trajectory_controller',
                    'effort_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            )
        ],
    )

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

    # ── t=5s: standing_controller (moves robot to nominal stance) ─────────────
    standing_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package    = 'g1_control',
                executable = 'standing_controller',
                output     = 'screen',
            )
        ],
    )

    # ── t=5.5s: Send hold trajectory BEFORE unpausing ─────────────────────────
    send_hold = TimerAction(period=7.5, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'action', 'send_goal',
                '/joint_trajectory_controller/follow_joint_trajectory',
                'control_msgs/action/FollowJointTrajectory',
                '''{
                  "trajectory": {
                    "joint_names": [
                      "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
                      "left_knee_joint",        "left_ankle_pitch_joint","left_ankle_roll_joint",
                      "right_hip_pitch_joint",  "right_hip_roll_joint", "right_hip_yaw_joint",
                      "right_knee_joint",       "right_ankle_pitch_joint","right_ankle_roll_joint",
                      "waist_yaw_joint"
                    ],
                    "points": [{
                      "positions":  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                      "velocities": [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                      "time_from_start": {"sec": 1, "nanosec": 0}
                    }]
                  }
                }'''
            ],
            output='screen',
        )
    ])

    # ── t=6s: Unpause Gazebo ───────────────────────────────────────────────────
    unpause = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )

    # ── t=8s: Footstep planner ────────────────────────────────────────────────
    # Starts after robot has reached standing pose (~2s after unpause)
    footstep_planner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package    = 'g1_zmp_walking',
                executable = 'footstep_planner_node',
                name       = 'footstep_planner',
                output     = 'screen',
                parameters = [
                    os.path.join(pkg_zmp, 'config', 'g1_params.yaml'),
                ],
                remappings = [
                    ('~/footsteps',        '/footstep_planner/footsteps'),
                    ('~/footstep_markers', '/footstep_planner/footstep_markers'),
                    ('~/plan_footsteps',   '/footstep_planner/plan_footsteps'),
                ],
            )
        ],
    )

    # ── t=8s: Walking controller ──────────────────────────────────────────────
    walking_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package    = 'g1_zmp_walking',
                executable = 'walking_controller_node',
                name       = 'g1_walking_controller',
                output     = 'screen',
                parameters = [
                    os.path.join(pkg_zmp, 'config', 'g1_params.yaml'),
                ],
                remappings = [
                    ('~/joint_states',  '/g1_controller/joint_states'),
                    ('~/status',        '/g1_controller/status'),
                    ('~/debug_markers', '/g1_controller/debug_markers'),
                ],
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        # force_pause,
        repause,
        spawn_jsb,
        # spawn_jtc,
        effort_controller_spawner,
        standing_controller,
        # send_hold,
        unpause,
        footstep_planner,
        # walking_controller,
    ])