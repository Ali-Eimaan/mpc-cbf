"""SKELETON — tube-MPC-CBF under wind disturbance.

Same 2-D obstacle scenario as 2d_obstacle.launch.py, but the simulator injects
a bounded disturbance drawn from W. Running it twice (use_tube_mpc:=false then
true) produces the two traces in media/tube_robustness.gif: nominal MPC-CBF
clips the obstacle, tube-MPC-CBF does not.

Implement per .deepseek/09_NODE.md §9.7.
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    # TODO(deepseek §9.7): assemble and return the description.
    #
    # Launch arguments:
    #   use_tube_mpc      bool  true
    #   tighten_mode      str   support_function
    #   wind_magnitude    float 1.0   [m/s^2] peak, must stay inside W
    #   wind_profile      str   worst_case | uniform | gust
    #                           worst_case pushes along -grad h (the adversarial
    #                           case the RPI set must cover)
    #   seed              int   0
    #   trials            int   1     >1 runs a batch headless and writes CSV
    #
    # Nodes:
    #   1. mpc_cbf_node with parameters from both YAML files, use_tube_mpc
    #      overridden by the argument.
    #   2. scripts/sim_double_integrator.py with --disturbance flags.
    #   3. scripts/log_min_barrier.py — records min_k h(x_k) per trial to CSV
    #      for analysis/disturbance_robustness_sweep.ipynb.
    raise NotImplementedError("tube_mpc_disturbance.launch.py not implemented")
