"""
Tube-MPC-CBF under wind disturbance.

Same 2-D obstacle scenario as 2d_obstacle.launch.py, but the simulator injects
a bounded disturbance drawn from W. Running it twice (use_tube_mpc:=false then
true) produces the two traces in media/tube_robustness.gif: nominal MPC-CBF
clips the obstacle, tube-MPC-CBF does not. Runs with no arguments.

`wind_profile:=worst_case` pushes along -grad h — the adversarial vertex of W
the RPI set must cover. `trials > 1` runs a headless batch (no RViz, no
interactive goal) and writes one CSV row per trial via log_min_barrier.py for
analysis/disturbance_robustness_sweep.ipynb.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = PathJoinSubstitution([FindPackageShare('mpc_cbf_unified')])

    declared_args = []
    for name, default, description in [
        ('use_tube_mpc', 'true', 'run the tube-MPC-CBF controller instead of MPC-CBF'),
        ('tighten_mode', 'support_function', 'support_function | lipschitz | none'),
        ('wind_magnitude', '1.0', 'disturbance scale, fraction of the W box [0, 1]'),
        ('wind_profile', 'worst_case', 'worst_case | uniform | gust'),
        ('seed', '0', 'RNG seed (deterministic runs)'),
        ('trials', '1', '>1 runs a headless batch and writes CSV rows per trial'),
    ]:
        declared_args.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )

    node = Node(
        package='mpc_cbf_unified',
        executable='mpc_cbf_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'mpc_cbf_params.yaml']),
            PathJoinSubstitution([pkg_share, 'config', 'tube_mpc_params.yaml']),
            {
                'use_tube_mpc': LaunchConfiguration('use_tube_mpc'),
                'tube.tighten_mode': LaunchConfiguration('tighten_mode'),
                'goal_topic': '/goal',
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
            '--disturbance', LaunchConfiguration('wind_profile'),
            '--w-max', LaunchConfiguration('wind_magnitude'),
            '--seed', LaunchConfiguration('seed'),
            '--trials', LaunchConfiguration('trials'),
        ],
    )

    # Always runs: with trials == 1 it logs a single min-h value at the end;
    # with trials > 1 it accumulates one row per episode and writes the CSV
    # (default output: tube_min_barrier_sweep.csv in the working directory).
    log_min_barrier = Node(
        package='mpc_cbf_unified',
        executable='log_min_barrier.py',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '--trials', LaunchConfiguration('trials'),
            '--seed', LaunchConfiguration('seed'),
        ],
    )

    return LaunchDescription(declared_args + [node, simulator, log_min_barrier])
