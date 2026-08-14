"""
Car racing / overtaking scenario (ACC 2021, Fig. 6-8).

Kinematic bicycle on a straight two-lane track, overtaking a slower lead
vehicle modelled as a moving elliptical obstacle. This is the scenario where
MPC-DC with a short horizon fails and MPC-CBF succeeds — the headline result of
the ACC paper, and the source of media/cbfqp_vs_mpccbf_side_by_side.gif.
Runs with no arguments; `controller:=mpc_dc` produces the failing baseline.

The bicycle convention (documented in the node source, §9.5): cmd_vel.linear.x
is the acceleration [m/s^2] and cmd_vel.angular.z is the steering angle [rad];
sim_bicycle.py integrates the kinematic bicycle (wheelbase 0.35 m).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = PathJoinSubstitution([FindPackageShare('mpc_cbf_unified')])

    declared_args = []
    for name, default, description in [
        ('controller', 'mpc_cbf', 'mpc_cbf | mpc_dc (selects cbf_variant)'),
        ('gamma', '0.4', 'class-K gain for the DCBF constraint (0, 1]'),
        ('horizon', '11', 'prediction horizon N'),
        ('lead_speed', '0.8', 'speed of the overtaken vehicle [m/s]'),
        ('ego_speed_ref', '1.5', 'reference speed along the centre line [m/s]'),
        ('track_width', '3.0', 'lane-boundary state bounds [m]'),
        ('record_bag', 'false', 'record /odom, /cmd_vel and /cbf_values to a bag'),
    ]:
        declared_args.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    controller = LaunchConfiguration('controller')
    # mpc_cbf runs the fixed-decay DCBF; mpc_dc is the baseline from the paper
    # (no decay constraint, distance-only).
    cbf_variant = PythonExpression(
        ["'fixed_decay' if '", controller, "' == 'mpc_cbf' else 'distance_only'"]
    )
    track_width = LaunchConfiguration('track_width')

    node = Node(
        package='mpc_cbf_unified',
        executable='mpc_cbf_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'mpc_cbf_params.yaml']),
            {
                'model': 'bicycle_kinematic',
                'gamma': LaunchConfiguration('gamma'),
                'horizon': LaunchConfiguration('horizon'),
                'cbf_variant': cbf_variant,
                # Lane boundaries from track_width: y in [-w/2, w/2]. The
                # bicycle state is [px, py, yaw, v].
                'x_min': [-1.0e9, -0.5 * track_width, -3.14159, -1.0],
                'x_max': [1.0e9, 0.5 * track_width, 3.14159, 3.0],
                'u_min': [-2.0, -0.5],
                'u_max': [2.0, 0.5],
                'goal_topic': '/goal',
            },
        ],
        remappings=[
            ('~/odom', '/odom'),
            ('~/obstacles', '/obstacles'),
            ('~/cmd_vel', '/cmd_vel'),
            ('~/predicted_path', '/predicted_path'),
            ('~/cbf_values', '/cbf_values'),
        ],
    )

    simulator = Node(
        package='mpc_cbf_unified',
        executable='sim_bicycle.py',
        parameters=[{'use_sim_time': False}],
        arguments=['--lead-speed', LaunchConfiguration('lead_speed')],
    )

    reference = Node(
        package='mpc_cbf_unified',
        executable='track_reference.py',
        parameters=[{'use_sim_time': False}],
        arguments=['--speed', LaunchConfiguration('ego_speed_ref')],
    )

    recorder = Node(
        package='ros2bag',
        executable='ros2bag',
        name='ros2bag',
        arguments=['record', '-o', 'car_racing', '/odom', '/cmd_vel', '/cbf_values'],
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    return LaunchDescription(declared_args + [node, simulator, reference, recorder])
