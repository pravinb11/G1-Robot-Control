from launch import LaunchDescription
from launch.actions          import DeclareLaunchArgument
from launch.substitutions    import LaunchConfiguration, PathJoinSubstitution
from launch.conditions       import IfCondition
from launch_ros.actions      import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('g1_zmp_walking')

    return LaunchDescription([
        DeclareLaunchArgument('rviz',        default_value='true'),
        DeclareLaunchArgument('standalone',  default_value='false',
            description='Run individual pipeline nodes (true) or integrated controller (false)'),

        # ── Footstep Planner (always needed) ─────────────────────────────────
        Node(
            package    = 'g1_zmp_walking',
            executable = 'footstep_planner_node',
            name       = 'footstep_planner',
            output     = 'screen',
            parameters = [PathJoinSubstitution([pkg, 'config', 'g1_params.yaml'])],
            remappings = [
                ('~/footsteps',        '/footstep_planner/footsteps'),
                ('~/footstep_markers', '/footstep_planner/footstep_markers'),
                ('~/plan_footsteps',   '/footstep_planner/plan_footsteps'),
            ],
        ),

        # ── Main integrated controller ────────────────────────────────────────
        Node(
            package    = 'g1_zmp_walking',
            executable = 'walking_controller_node',
            name       = 'g1_walking_controller',
            output     = 'screen',
            parameters = [PathJoinSubstitution([pkg, 'config', 'g1_params.yaml'])],
            remappings = [
                ('~/joint_states',   '/g1_controller/joint_states'),
                ('~/status',         '/g1_controller/status'),
                ('~/debug_markers',  '/g1_controller/debug_markers'),
            ],
        ),

        # ── RViz2 ─────────────────────────────────────────────────────────────
        Node(
            package    = 'rviz2',
            executable = 'rviz2',
            name       = 'rviz2',
            output     = 'screen',
            arguments  = ['-d', PathJoinSubstitution(
                               [pkg, 'config', 'footstep_rviz.rviz'])],
            condition  = IfCondition(LaunchConfiguration('rviz')),
        ),
    ])