# Media

Three animated assets are referenced by the root `README.md`. Each is produced by a notebook cell, never
by hand-recording a screen — a regenerable figure is a figure a reviewer can trust.

## Inventory

| File | Status | Produced by | Shows |
|---|---|---|---|
| `cbfqp_vs_mpccbf_side_by_side.gif` | present (238 KB) | `analysis/cbfqp_vs_mpccbf_comparison.ipynb` (final cell, `FuncAnimation` + `PillowWriter`) | CBF-QP colliding with a fast obstacle next to MPC-CBF avoiding it, same scenario, same seed |
| `obstacle_avoidance.gif` | to generate | `reproduction/zeng_acc2021/reproduce_acc2021.ipynb` §2 (static source `figures/fig4d_trajectories.png`) | 2-D obstacle avoidance at γ ∈ {0.1, 0.2, 0.3, 1.0}, with `h(x)` plotted alongside |
| `tube_robustness.gif` | to generate | `analysis/disturbance_robustness_sweep.ipynb` §2 (static source `figures/robustness_sweep.png`) | Nominal MPC-CBF clipping the obstacle under wind vs tube-MPC-CBF holding the margin |

## Regeneration

`cbfqp_vs_mpccbf_side_by_side.gif` is the template: the notebook's final cell uses
`matplotlib.animation.FuncAnimation` with a `PillowWriter` at 15 fps, capped at ~90 frames (≤ 8 s),
redrawing the trajectory up to frame `i` and printing the current `h(x)` in the title each frame. The two
remaining gifs are produced by the same pattern from the trajectory arrays their notebooks already
compute (§2 of the ACC notebook for `obstacle_avoidance.gif`; §2 of the robustness sweep for
`tube_robustness.gif`).

```bash
jupyter nbconvert --to notebook --execute analysis/cbfqp_vs_mpccbf_comparison.ipynb
jupyter nbconvert --to notebook --execute reproduction/zeng_acc2021/reproduce_acc2021.ipynb
jupyter nbconvert --to notebook --execute analysis/disturbance_robustness_sweep.ipynb
```

## Constraints

≤ 8 MB each (GitHub renders them inline; larger files stall the README), ≤ 8 s, ≥ 15 fps, and every
frame must carry the current `h(x)` value so the safety claim is legible from the GIF alone.
