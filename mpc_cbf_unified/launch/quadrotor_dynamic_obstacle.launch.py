"""
Planar quadrotor vs. a fast moving obstacle.

The scenario behind the side-by-side demo GIF: a one-step CBF-QP safety filter
cannot avoid an obstacle approaching faster than its reaction horizon, while
MPC-CBF with N = 15 does. Runs the planar-quadrotor model so the result is not
trivially a double-integrator artefact. Runs with no arguments;
`controller:=cbf_qp` produces the failing baseline.

The planar-quad convention (documented in the node source, §9.5):
cmd_vel.linear.x is thrust [N] and cmd_vel.angular.y is the pitch torque [N m];
sim_quadrotor_planar.py integrates the planar quadrotor (m = 1 kg, I = 0.1
kg m^2). The obstacle marker moves at constant velocity encoded in its
`points` field with frame_locked := false (the node reads that as is_dynamic).
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
        ('controller', 'mpc_cbf', 'mpc_cbf | cbf_qp (cbf_qp == horizon 1)'),
        ('horizon', '15', 'prediction horizon N (ignored when controller == cbf_qp)'),
        ('gamma', '0.2', 'class-K gain for the DCBF constraint (0, 1]'),
        ('obstacle_speed', '2.0', 'obstacle speed toward the vehicle [m/s]'),
        ('obstacle_radius', '0.4', 'obstacle radius [m]'),
        ('headless', 'false', 'skip RViz2 (for headless CI / GIF generation)'),
    ]:
        declared_args.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    controller = LaunchConfiguration('controller')
    # CBF-QP is exactly MPC-CBF with N = 1: no lookahead, so the safety filter
    # reacts only to the current h — the failure mode the demo shows.
    horizon = PythonExpression(["'15' if '", controller, "' == 'mpc_cbf' else '1'"])

    node = Node(
        package='mpc_cbf_unified',
        executable='mpc_cbf_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'mpc_cbf_params.yaml']),
            {
                'model': 'quadrotor_planar',
                'gamma': LaunchConfiguration('gamma'),
                'horizon': horizon,
                # Planar-quad state [px, pz, vx, vz, pitch, pitch_rate]:
                # hover thrust is m g = 9.81 N, so T must stay positive.
                'Q': [10.0, 10.0, 1.0, 1.0, 0.1, 0.1],
                'Qf': [100.0, 100.0, 10.0, 10.0, 1.0, 1.0],
                'R': [0.1, 0.1],
                'x_min': [-1.0e9, 0.0, -5.0, -5.0, -0.5, -3.0],
                'x_max': [1.0e9, 1.0e9, 5.0, 5.0, 0.5, 3.0],
                'u_min': [0.0, -1.0],
                'u_max': [15.0, 1.0],
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
        executable='sim_quadrotor_planar.py',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '--obstacle-speed', LaunchConfiguration('obstacle_speed'),
            '--obstacle-radius', LaunchConfiguration('obstacle_radius'),
        ],
    )

    rviz_config = PathJoinSubstitution(
        [pkg_share, 'config', 'rviz', 'quadrotor_dynamic_obstacle.rviz']
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(PythonExpression(["'true' if '", LaunchConfiguration('headless'),
                                                "' == 'false' else 'false'"])),
    )

    return LaunchDescription(declared_args + [node, simulator, rviz])
