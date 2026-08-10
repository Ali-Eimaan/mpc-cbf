# §12 · Reproductions, analysis and media

**Governs:** `reproduction/`, `analysis/`, `media/`
**Milestone:** M10
**Done when:** A9 is green and all three GIFs exist and are regenerable.

---

## Notebook ground rules — all five

1. **Runs top-to-bottom headless**, no manual steps, fixed seed printed in the first cell.
2. **Every claim is an `assert`.** A plot nobody checks is decoration. The two reproduction
   notebooks are executed by CI ([14_CI.md §14.5](14_CI.md)), so an assertion is what turns a
   numerical regression into a red build.
3. **Figures to `<notebook_dir>/figures/`, tables to CSV** next to the notebook. Both are
   `.gitignore`d; the notebook is the source, the outputs are not.
4. **Any deviation from the source paper gets its own markdown cell.** An unstated deviation
   makes the reproduction worthless.
5. **Scenario constants are imported, not retyped.** The 2-D scenario appears in the gtest
   fixture, the pytest fixture and two notebooks; if they drift, none of the numbers can be
   compared.

---

## §12.1 `reproduction/zeng_acc2021/reproduce_acc2021.ipynb`

Zeng, Zhang & Sreenath, *Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function*, ACC 2021.

| Section | Content | Assertions |
| --- | --- | --- |
| 1 | scenario setup, `rollout()` helper | — |
| 2 | trajectories for `γ ∈ {0.1, 0.2, 0.3, 1.0}` | minimum clearance monotonically decreasing in `γ` |
| 3 | `h(x_k)` over time, with the `(1−γ)^k` envelope overlaid | `min h ≥ −1e-9` for every `γ`; DCBF inequality holds at every step to 1e-6 |
| 4 | MPC-CBF vs MPC-DC at `N ∈ {3,5,8}` | at `N=3`, MPC-DC has a failure and MPC-CBF does not |
| 5 | car racing, bicycle model | overtake completes; `min h ≥ 0` |
| 6 | summary table → `results_acc2021.csv`, printed as markdown | — |

**Figure numbers.** The section headings quote figure numbers from memory. **Verify each against
the actual paper before it appears in `REPRODUCTION_REPORT.md`** (rule 4, §0). A wrong figure
number in a reproduction report is the kind of error that makes a reviewer stop reading.

The `(1−γ)^k` envelope in §3 is the visual form of the invariance argument
([04_MODELS.md §4.5](04_MODELS.md)) — it is the single most explanatory plot in the repository.

## §12.2 `reproduction/zeng_cdc2021/reproduce_cdc2021.ipynb`

Zeng, Li & Sreenath, *Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with
Discrete-Time Control Barrier Functions*, CDC 2021.

| Section | Content | Assertions |
| --- | --- | --- |
| 1 | the relaxed-decay formulation, with the `ωγ ≤ 1` safety derivation written out | — |
| 2 | feasible-region comparison over a `γ` sweep, both variants | relaxed-decay feasible fraction ≥ fixed-decay at every `γ` |
| 3 | realised `ω_k` trajectories | `max(ω·γ) ≤ 1 + 1e-9` at every step |
| 4 | `N_CBF` sweep at `N = 8`: solve time vs feasible fraction | — |
| 5 | summary → `results_cdc2021.csv` | — |

§3 has a diagnostic purpose beyond the assertion: if `ω` sits at its upper bound everywhere,
`omega_weight` is too small relative to the tracking cost and the plot means nothing. Say so in
the cell output rather than shipping a flat line.

§4 is what justifies the `cbf_horizon` default in `config/mpc_cbf_params.yaml`. Right now that
default is a guess; after this notebook it must be a measurement or explicitly labelled as a
guess.

## §12.3 `analysis/cbfqp_vs_mpccbf_comparison.ipynb`

100 seeded scenarios; identical plant, cost and barrier; two controllers — a one-step CBF-QP
filter on a nominal tracking controller, and MPC-CBF at `N = 15`.

Metrics per run: collision (`min h < 0`), infeasible steps, time-to-goal, path length, control
effort, mean and p95 solve time.

**The claim is not "MPC-CBF is better". It is "here is exactly where each one fails, and why."**
Two things this notebook must contain or it has not done its job:

- **The myopic filter winning on solve time**, by a factor you state. It will be 10–50×. Report it
  plainly; a reader who has written a CBF-QP knows this and will notice its absence.
- **At least one scenario where MPC-CBF loses.** It exists — conservatism at large `γ`, solve-time
  budget, or a case where the longer horizon commits early to a worse side of an obstacle. Find
  it, show it, explain the mechanism. If you cannot find one, widen the scenario distribution
  until you do. A one-sided result reads as a result that was not honestly sought, and that
  costs more credibility than the finding would have gained.

Persist the scenarios to `analysis/scenarios.json` so the numbers are re-checkable and the GIF
uses the same ones.

Keep the CBF-QP baseline in the notebook, not in the C++ library. It is a baseline, not a product.

## §12.4 `analysis/feasibility_recovery_study.ipynb`

Consumes the failure list from [11_PYTHON_REFERENCE.md §11.3](11_PYTHON_REFERENCE.md).

1. **Feasible-set maps.** Grid over the state space, three colours: feasible at `k=0` / feasible
   for 100 steps / infeasible. Sweep `γ ∈ {0.1, 0.3, 0.7, 1.0}`.
2. **Recovery strategies**, scored on two axes:

   | Strategy | Mechanism |
   | --- | --- |
   | `slack` | soften the DCBF row with an L1-penalised slack |
   | `relaxed_decay` | the CDC 2021 `ω` variable |
   | `horizon_backoff` | re-solve with a shorter horizon |
   | `previous_horizon` | the node's default ([09_NODE.md §9.4](09_NODE.md)) |

   Report recovery rate **and** the worst `min h` incurred, as a scatter. **A strategy that
   recovers by going unsafe has not recovered** — the single-axis version of this plot would
   rank the most dangerous strategy first.

This notebook is where [01_OVERVIEW.md §1.6](01_OVERVIEW.md)'s honesty about recursive
feasibility becomes a measurement instead of a disclaimer. Frame it that way in the opening cell.

## §12.5 `analysis/disturbance_robustness_sweep.ipynb`

1. **The RPI set.** `Ω` projected onto `(px,py)` and `(vx,vy)`, with 10 000 sampled steady-state
   closed-loop errors overlaid. **Assert 100 % containment.** If the cloud escapes `Ω`, the RPI
   computation is wrong and everything downstream is decoration. This is the most convincing
   single figure in the repository.
2. **Magnitude sweep.** `w_max` geometric sweep, 50 seeds, nominal vs tube: `min_k h`, violation
   rate, cost.

   Expected shape: nominal degrades then violates; tube holds `min h ≥ 0` up to the design
   `w_max` **and then also fails**. A tube that never fails means `W` was specified larger than
   the disturbance actually applied — fix the experiment, not the plot.
3. **Conservatism cost.** Path length and time-to-goal versus nominal at *zero* disturbance.
   Report the number in the README; it is the honest counterweight to the safety claim.

Cross-check `Ω` against `compute_offline_sets()` and assert agreement to 1e-6
([05_CODEGEN.md §5.7](05_CODEGEN.md)) before plotting anything.

## §12.6 `media/`

| File | Produced by | Must show |
| --- | --- | --- |
| `obstacle_avoidance.gif` | §12.1 | 2-D avoidance at three `γ`, with `h(t)` inset |
| `cbfqp_vs_mpccbf_side_by_side.gif` | §12.3 | same seed, same scenario, one controller colliding |
| `tube_robustness.gif` | §12.5 | nominal clipping the obstacle under wind, tube holding, tube cross-section drawn |

Constraints: ≤ 8 MB each (GitHub renders them inline; larger files stall the README), ≤ 8 s,
≥ 15 fps, and **every frame carries the current `h(x)` value** so the safety claim is legible from
the GIF alone.

Generated by a notebook cell, never by screen-recording a demo. A regenerable figure is one a
reviewer can trust; a screen capture is a claim about something that happened once on your
machine.

`media/README.md` records the provenance of each file. Keep it accurate — it is what makes the
GIFs evidence rather than marketing.
