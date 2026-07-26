# Media

Placeholder. These three assets are referenced by the root `README.md` and must exist before v1.0.
Each is produced by a notebook cell, never by hand-recording a screen — a regenerable figure is a
figure a reviewer can trust.

| File | Produced by | Shows |
|---|---|---|
| `obstacle_avoidance.gif` | `reproduction/zeng_acc2021/reproduce_acc2021.ipynb` §2 | 2-D obstacle avoidance at three values of gamma, with `h(x)` plotted alongside |
| `cbfqp_vs_mpccbf_side_by_side.gif` | `analysis/cbfqp_vs_mpccbf_comparison.ipynb` (final cell) | CBF-QP colliding with a fast obstacle next to MPC-CBF avoiding it, same scenario, same seed |
| `tube_robustness.gif` | `analysis/disturbance_robustness_sweep.ipynb` (final cell) | Nominal MPC-CBF clipping the obstacle under wind vs tube-MPC-CBF holding the margin |

Constraints: ≤ 8 MB each (GitHub renders them inline; larger files stall the README), ≤ 8 s, ≥ 15 fps,
and every frame must carry the current `h(x)` value so the safety claim is legible from the GIF alone.
