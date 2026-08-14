# Reproduction Report

This document is the credential the repository exists to earn: it states precisely which figures from the
two source papers were reproduced, which numbers matched, to what tolerance, and — importantly — what did
**not** match and why.

## Environment

The figures below were produced on the development machine and **re-executed
unchanged in the release container** before tagging `v1.0.0`; both notebooks
assert their own numbers, so a regression would have failed the run.

| Item | Development run (numbers below) | Release re-run |
|---|---|---|
| Date | 2026-08-14 | 2026-08-15 |
| Release tag | — | `v1.0.0` |
| OS / compiler | Linux 7.0.0-29-generic / gcc 15.2.0 | `ros:lyrical-ros-base` (Ubuntu 26.04) / gcc 15.2.0 |
| acados | v0.6.0, commit `503364817c872d474ab5bed219c26760ac267769` | v0.6.0 |
| CasADi / numpy / scipy | 3.7.2 / 2.5.2 / 1.18.0 | 3.7.2 / 2.5.2 / 1.18.0 |
| QP solver | HPIPM (partial condensing) | HPIPM (partial condensing) |
| CPU | Intel Core i7-7600U @ 2.80 GHz | not quoted — timings come from the development run only |

Solve times are quoted from the development run on the CPU named above. The
container re-run confirms the safety and feasibility numbers, not the timings:
a containerised measurement on shared hardware is not a number worth defending.

## Paper 1 — Zeng, Zhang & Sreenath, ACC 2021

*Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function.*

### Figure-by-figure

> Figure map verified against the arXiv HTML (an earlier draft's "Fig. 2/3/4/6–8" labels were wrong).

| Paper figure | What it shows | Reproduced? | Our value | Paper value | Tolerance | Notebook cell |
|---|---|---|---|---|---|---|
| Fig. 1 | Car-racing overtake snapshots | Yes | overtake at t = 4.8 s, min h = 2.189e-01 | safe overtake | min h ≥ -1e-6 | §5 |
| Fig. 2 | MPC-CBF feasibility level sets (schematic) | No — schematic | — | — | — | — |
| Fig. 3 | Feasible-set comparison MPC-CBF vs MPC-DC (schematic) | No — schematic | — | — | — | — |
| Fig. 4(d) | Trajectories for γ ∈ {0.1, 0.2, 0.3, 1.0} | Yes | min clearance 0.6265 / 0.3627 / 0.1976 / 0.0000 m | monotone decreasing in γ | strict > | §2 |
| Fig. 4(e) | MPC-CBF N=5 vs MPC-DC (Table I analog) | Yes | MPC-CBF feasible, min h = 1.17e-01; MPC-DC infeasible, min h = 5.01e+00 | MPC-DC fails at short horizon | MPC-DC infeasible | §4 |
| Fig. 5 | Curvilinear coordinates (schematic) | No — schematic | — | — | — | — |
| Fig. 6 | Speed profile | Yes | `figures/fig6_speed_profile.png` | — | qualitative | §5 |

### Quantitative claims checked

| Claim | Status | Evidence |
|---|---|---|
| `h(x_k) >= 0` for the whole closed loop, all gamma | PASS | min h = 3.52e-08 (γ = 1.0) ≥ -1e-6 |
| Smaller gamma ⇒ larger minimum clearance | PASS | clearance(0.1) = 0.6265 > clearance(1.0) = 0.0000 |
| MPC-DC at short horizon fails where MPC-CBF succeeds | PASS | N = 5: MPC-CBF feasible, MPC-DC infeasible (min h = 5.01) |
| DCBF inequality holds at every step to 1e-6 | PASS | worst violation ≤ 1e-6 |
| `h(x_k) >= (1-gamma)^k h(x_0)` envelope (eq. (3)) | PASS | γ=0.1: 2.27; γ=0.2: 1.22; γ=0.3: 6.32e-01; γ=1.0: 3.52e-08 |

### Deviations from the paper

> List every one. An unlisted deviation is a silent error.

- **Solver.** Paper: IPOPT (0.028 s mean solve). Repo: acados SQP (Gauss–Newton, HPIPM, tolerances 1e-3, mean 1.3 ms). Safety/feasibility claims transfer; timings do not and are not compared.
- **Terminal weight.** Paper does not state Qf; repo default Qf = 10 Q, overridden to the paper's Q = 10·I₄ / R = I₂ / P = 100·I₄ for the reproduction.
- **Cost scale.** Paper's Σ uᵀu Δt differs from the repo LS cost by a uniform Δt factor (same argmin).
- **State bounds.** Paper |x| ≤ 5 added via `idxbx`; `build_ocp` sets none by default.
- **Ego radius.** Paper point mass (r_eff = r_obs = 1.5, no inflation); repo fixture inflates by ego_radius + safety_margin (deliberate model difference).
- **Horizon grid.** Spec's N = 3 is too short for both variants (ACADOS_MINSTEP); the grid uses N ∈ {5, 8}.
- **Car racing (§5.1).** Kinematic bicycle (RK4) replaces the paper's data-driven lateral-vehicle model; squared-distance CBF replaces the quartic curvilinear CBF; one lead car instead of two; discrete RK4 map instead of 1000 Hz integration; N = 15 (not 11/12); lead at e_y = −0.1; passing-lane reference y = 0.7. The reproduced claim is qualitative (a safe overtake completes).
- **Obstacle slots.** Parameter vector padded to 8 slots; the paper scenario uses `N_OBSTACLES_SOLVER = 1` (dummy DCBF rows stall acados SQP at small γ).
- **Warm start.** Cold start from the u = 0 coasting trajectory, then shifted-previous-solution warm start (paper does not specify); `nlp_solver_max_iter = 100`.

## Paper 2 — Zeng, Li & Sreenath, CDC 2021

*Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with Discrete-Time Control Barrier Functions.*

### Figure-by-figure

| Paper figure | What it shows | Reproduced? | Our value | Paper value | Tolerance | Notebook cell |
|---|---|---|---|---|---|---|
| Fig. 2 | Feasible region, fixed vs relaxed decay | Yes | fixed 0.983/0.989/0.994/0.994 vs relaxed 0.983/0.994/0.994/0.994 over γ ∈ {0.1, 0.3, 0.7, 1.0} | relaxed ⊇ fixed | relaxed ≥ fixed at every γ | §2 |
| Fig. 3 | omega_k trajectories | Yes | ω ∈ [1.0000, 1.0463], max ω·γ = 0.3139 | ω = 1 unless decay binds | ω·γ ≤ 1 | §3 |
| Fig. 4 | Closed-loop trajectories | Yes | 25 steps to goal, min h = 1.4962e-02 | goal reached safely | min h ≥ 0 | §3 |
| Fig. 5 | CBF-horizon sweep | Yes | N_CBF {1,2,4,8} → feasible 1.000/1.000/1.000/0.989, solve 1.71/2.27/2.15/2.64 ms | solve time scales with N_CBF | qualitative | §4 |

### Quantitative claims checked

| Claim | Status | Evidence |
|---|---|---|
| Relaxed decay enlarges the feasible set at every gamma | PASS | fixed {0.983, 0.989, 0.994, 0.994} vs relaxed {0.983, 0.994, 0.994, 0.994}; relaxed ≥ fixed at every γ, strictly greater at γ = 0.3 |
| `omega_k * gamma <= 1` holds at every step | PASS | max ω·γ = 0.3139 (γ = 0.3) ≤ 1 |
| Safety preserved under relaxation | PASS | min h = 1.4962e-02 ≥ 0 over the 25-step closed loop |
| Recovery rate of relaxed decay on fixed-decay failures | PASS | γ = 0.3: 2 of 4 fixed-decay failures recovered (50 %) |

### Deviations

- **Start (0, 0.2), not (0, 0).** At the symmetric fixture start (0, 0) the relaxed SQP 2-cycles between "pass left"/"pass right" (status 2, KKT residual ~1.9e-2) at the first step — a solver convergence artifact, not infeasibility. The off-diagonal start (0, 0.2) breaks the symmetry so the tight-passage ω relaxation survives. Reported per the ground rule that contradictions are not papered over.
- **ω bounds vs safety condition.** ω ∈ [0, 3] (YAML `omega_max`); the safety condition ω·γ ≤ 1 couples ω with the runtime parameter γ and cannot be a static bound, so it is checked empirically on the returned trajectory (§3) and in the pytest suite's A6.
- **N_CBF implementation.** `build_ocp` validates `cbf_horizon` but the generated OCP always carries full-horizon DCBF rows; the runtime only sizes diagnostics. The sweep therefore deactivates the DCBF half of `lh` at stages k ≥ N_CBF via `solver.constraints_set(k, 'lh', ...)` — the same thing a runtime with a real N_CBF would do (the measurement, not a workaround).
- **8-slot dummy obstacles** retained for parity with the pytest fixture; the fixture scenario solves cleanly with them (unlike ACC §1.1 deviation 9).

## Baselines established for downstream citation

These numbers are cited by `transition-viable-swarm` and the planned CDC 2027 submission. Changing them
requires re-running both notebooks and updating the dependents.

| Quantity | Value | Source |
|---|---|---|
| MPC-CBF mean solve time, N = 8, 1 obstacle | 1.32 ms | ACC notebook §4 (`results_acc2021_table1.csv`) |
| MPC-CBF 95th-percentile solve time (RTI) | 0.555 ms | gtest `SolveMeetsRealTimeBudget` |
| Feasible fraction, gamma = 0.3, N = 8 | 0.989 | CDC notebook §2 (fixed decay) |
| Tube conservatism (path-length penalty at zero disturbance) | 12.27 % | robustness sweep §3 |

## How to re-run

```bash
pip install -r codegen/requirements.txt
python codegen/generate_mpc_cbf_solver.py --all
jupyter nbconvert --to notebook --execute reproduction/zeng_acc2021/reproduce_acc2021.ipynb
jupyter nbconvert --to notebook --execute reproduction/zeng_cdc2021/reproduce_cdc2021.ipynb
```
