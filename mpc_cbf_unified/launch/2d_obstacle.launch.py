"""
2-D static obstacle avoidance (ACC 2021, Fig. 2/3).

Brings up mpc_cbf_node with the double-integrator model against one static
obstacle at (0.5, 0.5) between start (0, 0) and goal (1, 1) — the exact
scenario reproduced in reproduction/zeng_acc2021/reproduce_acc2021.ipynb.
Runs with no arguments; every knob below is an optional override.

The double-integrator convention (documented in the node source):
cmd_vel.linear.x/y carry ACCELERATIONS [m/s^2] and sim_double_integrator.py
integrates them into velocity. Obstacle markers are SPHEREs; a marker's
scale.x/2 is its radius and the node adds ego_radius + safety_margin on top.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = PathJoinSubstitution([FindPackageShare('mpc_cbf_unified')])

    declared_args = []
    for name, default, description in [
        ('gamma', '0.3', 'class-K gain for the DCBF constraint (0, 1]'),
        ('horizon', '8', 'prediction horizon N'),
        ('cbf_variant', 'fixed_decay', 'fixed_decay | relaxed_decay | distance_only'),
        ('obstacle_x', '0.5', 'static obstacle centre x [m]'),
        ('obstacle_y', '0.5', 'static obstacle centre y [m]'),
        ('obstacle_radius', '0.2', 'static obstacle radius [m]'),
        ('goal_x', '1.0', 'goal position x [m]'),
        ('goal_y', '1.0', 'goal position y [m]'),
        ('rviz', 'true', 'launch RViz2 alongside the simulation'),
    ]:
        declared_args.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz', '2d_obstacle.rviz'])

    node = Node(
        package='mpc_cbf_unified',
        executable='mpc_cbf_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'mpc_cbf_params.yaml']),
            {
                'gamma': LaunchConfiguration('gamma'),
                'horizon': LaunchConfiguration('horizon'),
                'cbf_variant': LaunchConfiguration('cbf_variant'),
            },
        ],
        remappings=[
            ('~/odom', '/odom'),
            ('~/goal', '/goal'),
            ('~/obstacles', '/obstacles'),
            ('~/cmd_vel', '/cmd_vel'),
            ('~/predicted_path', '/predicted_path'),
            ('~/cbf_values', '/cbf_values'),
        ],
    )

    simulator = Node(
        package='mpc_cbf_unified',
        executable='sim_double_integrator.py',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '--goal-x', LaunchConfiguration('goal_x'),
            '--goal-y', LaunchConfiguration('goal_y'),
            '--obstacle-x', LaunchConfiguration('obstacle_x'),
            '--obstacle-y', LaunchConfiguration('obstacle_y'),
            '--obstacle-radius', LaunchConfiguration('obstacle_radius'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(declared_args + [node, simulator, rviz])
