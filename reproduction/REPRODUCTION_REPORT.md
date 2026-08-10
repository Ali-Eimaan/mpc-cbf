# Reproduction Report

> **SKELETON.** Every `TBD` is filled by executing the two reproduction notebooks and pasting their
> emitted tables. Do not hand-write a number into this file — every entry must be traceable to a cell
> output. See `.deepseek/13_DOCS.md` §13.5.

This document is the credential the repository exists to earn: it states precisely which figures from the
two source papers were reproduced, which numbers matched, to what tolerance, and — importantly — what did
**not** match and why.

## Environment

| Item | Value |
|---|---|
| Date of run | TBD |
| Commit | TBD |
| OS / compiler | TBD |
| acados version (tag + commit) | TBD |
| CasADi / numpy / scipy | TBD |
| QP solver | TBD (HPIPM partial condensing) |
| CPU | TBD |

## Paper 1 — Zeng, Zhang & Sreenath, ACC 2021

*Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function.*

### Figure-by-figure

| Paper figure | What it shows | Reproduced? | Our value | Paper value | Tolerance | Notebook cell |
|---|---|---|---|---|---|---|
| Fig. 2 | Trajectories vs gamma | TBD | TBD | TBD | TBD | §2 |
| Fig. 3 | h(x) over time | TBD | TBD | TBD | TBD | §3 |
| Fig. 4 | Control inputs | TBD | TBD | TBD | TBD | §4 |
| Fig. 5 | MPC-CBF vs MPC-DC | TBD | TBD | TBD | TBD | §4 |
| Fig. 6 | Car racing trajectory | TBD | TBD | TBD | TBD | §5 |
| Fig. 7 | Racing inputs | TBD | TBD | TBD | TBD | §5 |
| Fig. 8 | Racing h(x) | TBD | TBD | TBD | TBD | §5 |

### Quantitative claims checked

| Claim | Status | Evidence |
|---|---|---|
| `h(x_k) >= 0` for the whole closed loop, all gamma | TBD | min h = TBD |
| Smaller gamma ⇒ larger minimum clearance | TBD | clearance(0.1) = TBD > clearance(1.0) = TBD |
| MPC-DC at short horizon fails where MPC-CBF succeeds | TBD | N = 3: MPC-DC TBD, MPC-CBF TBD |
| DCBF inequality holds at every step to 1e-6 | TBD | max violation = TBD |

### Deviations from the paper

> List every one. An unlisted deviation is a silent error.

- TBD (e.g. "paper does not state the terminal weight; we used Qf = 10 Q, which changes ... by ...")

## Paper 2 — Zeng, Li & Sreenath, CDC 2021

*Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with Discrete-Time Control Barrier Functions.*

| Paper figure | What it shows | Reproduced? | Our value | Paper value | Tolerance | Notebook cell |
|---|---|---|---|---|---|---|
| Fig. 2 | Feasible region, fixed vs relaxed decay | TBD | TBD | TBD | TBD | §2 |
| Fig. 3 | omega_k trajectories | TBD | TBD | TBD | TBD | §3 |
| Fig. 4 | Closed-loop trajectories | TBD | TBD | TBD | TBD | §2 |
| Fig. 5 | CBF-horizon sweep | TBD | TBD | TBD | TBD | §4 |

### Quantitative claims checked

| Claim | Status | Evidence |
|---|---|---|
| Relaxed decay enlarges the feasible set at every gamma | TBD | fractions: TBD |
| `omega_k * gamma <= 1` holds at every step | TBD | max = TBD |
| Safety preserved under relaxation | TBD | min h = TBD |
| Recovery rate of relaxed decay on fixed-decay failures | TBD | TBD % |

### Deviations

- TBD

## Baselines established for downstream citation

These numbers are cited by `transition-viable-swarm` and the planned CDC 2027 submission. Changing them
requires re-running both notebooks and updating the dependents.

| Quantity | Value | Source |
|---|---|---|
| MPC-CBF mean solve time, N = 8, 1 obstacle | TBD ms | ACC notebook §4 |
| MPC-CBF 95th-percentile solve time (RTI) | TBD ms | gtest `SolveMeetsRealTimeBudget` |
| Feasible fraction, gamma = 0.3, N = 8 | TBD | CDC notebook §2 |
| Tube conservatism (path-length penalty at zero disturbance) | TBD % | robustness sweep §3 |

## How to re-run

```bash
pip install -r codegen/requirements.txt
python codegen/generate_mpc_cbf_solver.py --all
jupyter nbconvert --to notebook --execute reproduction/zeng_acc2021/reproduce_acc2021.ipynb
jupyter nbconvert --to notebook --execute reproduction/zeng_cdc2021/reproduce_cdc2021.ipynb
```
