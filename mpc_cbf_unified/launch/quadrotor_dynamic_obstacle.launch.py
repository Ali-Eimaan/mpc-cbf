"""SKELETON — planar quadrotor vs. a fast moving obstacle.

The scenario behind the side-by-side demo GIF: a one-step CBF-QP safety filter
cannot avoid an obstacle approaching faster than its reaction horizon, while
MPC-CBF with N = 15 does. Runs the planar-quadrotor model so the result is not
trivially a double-integrator artefact.

Implement per IMPLEMENTATION_GUIDE.md §7.
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    # TODO(deepseek): assemble and return the description.
    #
    # Launch arguments:
    #   controller        str   mpc_cbf | cbf_qp   (cbf_qp == horizon 1,
    #                           cbf_variant fixed_decay, tracking cost only)
    #   horizon           int   15
    #   gamma             float 0.2
    #   obstacle_speed    float 2.0  [m/s] toward the vehicle
    #   obstacle_radius   float 0.4
    #   headless          bool  false
    #
    # Nodes:
    #   1. mpc_cbf_node with model:=quadrotor_planar.
    #   2. scripts/sim_quadrotor_planar.py — plant + dynamic obstacle publisher
    #      (constant velocity; the solver's constant-velocity prediction is
    #      therefore exact here, which is the point: the failure of CBF-QP is
    #      not a prediction-error artefact).
    #   3. RViz2 unless headless.
    raise NotImplementedError("quadrotor_dynamic_obstacle.launch.py not implemented")
