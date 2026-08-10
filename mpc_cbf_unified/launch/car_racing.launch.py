"""SKELETON — car racing / overtaking scenario (ACC 2021, Fig. 6-8).

Kinematic bicycle on a straight two-lane track, overtaking a slower lead
vehicle modelled as a moving elliptical obstacle. This is the scenario where
MPC-DC with a short horizon fails and MPC-CBF succeeds — the headline result of
the ACC paper, and the source of media/cbfqp_vs_mpccbf_side_by_side.gif.

Implement per .deepseek/09_NODE.md §9.7.
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    # TODO(deepseek §9.7): assemble and return the description.
    #
    # Launch arguments:
    #   controller       str   mpc_cbf | mpc_dc   (selects cbf_variant)
    #   gamma            float 0.4
    #   horizon          int   11
    #   lead_speed       float 0.8   [m/s] speed of the overtaken vehicle
    #   ego_speed_ref    float 1.5   [m/s]
    #   track_width      float 3.0   [m]  -> lane-boundary state bounds
    #   record_bag       bool  false -> ros2 bag record of /odom, /cbf_values
    #
    # Nodes:
    #   1. mpc_cbf_node with model:=bicycle_kinematic and the reference
    #      trajectory publisher below feeding ~/goal at every step (centre-line
    #      at ego_speed_ref).
    #   2. scripts/sim_bicycle.py — plant + lead-vehicle marker publisher.
    #   3. scripts/track_reference.py — publishes the centre-line reference.
    #   4. Optional rosbag2 recorder, conditioned on record_bag.
    raise NotImplementedError("car_racing.launch.py not implemented")
