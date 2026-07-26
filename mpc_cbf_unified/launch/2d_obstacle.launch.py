"""SKELETON — 2-D static obstacle avoidance (ACC 2021, Fig. 2/3).

Brings up mpc_cbf_node with the double-integrator model against one static
obstacle at (0.5, 0.5) between start (0, 0) and goal (1, 1) — the exact
scenario reproduced in reproduction/zeng_acc2021/reproduce_acc2021.ipynb.

Implement per IMPLEMENTATION_GUIDE.md §7.
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    # TODO(deepseek): assemble and return the description.
    #
    # Launch arguments (all with defaults, all documented):
    #   gamma            float  0.3     forwarded to the node
    #   horizon          int    8
    #   cbf_variant      str    fixed_decay
    #   obstacle_x/y     float  0.5/0.5
    #   obstacle_radius  float  0.2
    #   goal_x/y         float  1.0/1.0
    #   rviz             bool   true    launch RViz with config/rviz/2d_obstacle.rviz
    #
    # Nodes:
    #   1. mpc_cbf_unified/mpc_cbf_node
    #      - parameters: [config/mpc_cbf_params.yaml, {overrides from args}]
    #      - remappings: ~/odom -> /odom, ~/cmd_vel -> /cmd_vel
    #   2. A tiny Python simulator node integrating the double integrator at
    #      1/dt and republishing Odometry, plus a static MarkerArray for the
    #      obstacle. Put it in mpc_cbf_unified/scripts/sim_double_integrator.py
    #      and install it via CMakeLists (ament_python_install_package /
    #      install(PROGRAMS ...)). Without it this launch file has no plant.
    #   3. RViz2, conditioned on the `rviz` argument.
    raise NotImplementedError("2d_obstacle.launch.py not implemented")
