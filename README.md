# mpc-cbf

Discrete-time MPC with CBF constraints (Zeng-Zhang-Sreenath ACC 2021 + CDC 2021 reproduced in C++/acados, plus tube-MPC-CBF extension).

> **Status: skeleton.** The repository structure, interfaces, tests and documentation outlines are in
> place; the implementation is not. The full implementation specification lives in
> [`.deepseek/`](.deepseek/) — start at [`.deepseek/README.md`](.deepseek/README.md).

<!-- TODO: badges once CI is green — colcon build, notebook reproduction, license -->

## What this is

A single-agent safety-critical controller that integrates a predictive horizon with a control barrier
function safety condition, in three layers:

1. **MPC-CBF** — discrete-time CBF constraints imposed along the MPC prediction horizon (ACC 2021).
2. **Relaxed decay rate** — the decay rate becomes a decision variable, enlarging the feasible set
   without giving up the safety guarantee (CDC 2021).
3. **Tube MPC-CBF** — robust safety under bounded disturbances via an offline RPI set and constraint
   tightening (extension beyond the two papers).

C++ / acados for the controller, Python for code generation, reproduction and analysis.

## Layout

| Path | Contents |
|---|---|
| `mpc_cbf_unified/` | ROS 2 (Lyrical Luth) package: solvers, node, launch files, configs, tests |
| `codegen/` | acados/CasADi code generation and the shared model definitions |
| `reproduction/` | Notebooks reproducing the two source papers + `REPRODUCTION_REPORT.md` |
| `analysis/` | CBF-QP vs MPC-CBF, feasibility recovery, disturbance robustness studies |
| `docs/` | Derivations (LaTeX), math summary, annotated prior art |
| `media/` | Demo GIFs (regenerated from notebooks) |
| `.deepseek/` | Implementation specification: rules, milestones, per-subsystem contracts, review protocol |

## Quick start

<!-- TODO: fill in once Milestone 1 lands -->

```bash
# TODO: acados install, codegen, colcon build, and the 2-D obstacle demo
```

## Demos

<!-- TODO: embed media/obstacle_avoidance.gif, media/cbfqp_vs_mpccbf_side_by_side.gif,
     media/tube_robustness.gif — see media/README.md -->

## Reproductions

<!-- TODO: link REPRODUCTION_REPORT.md and summarise the matched figures in one table -->

## Citing

<!-- TODO: CITATION.cff, and the pointer to the thesis work this feeds -->

## License

BSD-3-Clause — see [LICENSE](LICENSE).
