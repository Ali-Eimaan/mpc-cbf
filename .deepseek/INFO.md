# GitHub Portfolio — Detailed Specification & PhD-Readiness Analysis

> **Read-only.** This is the author's own portfolio specification, moved here from the repository
> root. It fixes the target structure and the demo deliverables for this repository. Do not edit
> it to match the implementation — if the implementation must deviate, record the deviation in
> [01_OVERVIEW.md §1.5](01_OVERVIEW.md) instead.

**Owner:** Pakistani robotics PhD candidate (MS Mechatronics, 5 yrs embedded/robotics, ROS 2 Jazzy + PX4 + Gazebo)
**Thesis target:** *Transition-Viable Distributed MPC-CBF for Aerial-Ground Swarms: A Hybrid-Systems Theory of Asynchronous Split, Merge, and Morph Events with Set-Valued Resets*
**Application target:** Fall 2027 PhD intake at hardware-equipped labs (Tier A: KTH, ETH, TU Delft, TU/e; Tier B: Waterloo, UTIAS, Polytechnique Montréal, NTNU; Tier C: HKUST, HKU, NTU, UNIST)
**Document purpose:** Define the full repository structure, file-level contents, and identify any gaps that need closing before December 2026 application submissions.

---

## How to read this document

For each repo I specify:
1. **Identity** — name, one-line tagline, primary language, lines-of-code estimate at v1.0 release.
2. **Purpose** — what it proves to a hardware-equipped advisor reading your GitHub.
3. **File-level structure** — every directory and the files in it, with one-sentence descriptions of what each non-boilerplate file contains.
4. **Demo deliverables** — videos, GIFs, plots that go in the README.
5. **CI / quality signals** — what GitHub Actions runs and what badges go on the README.
6. **Citation hook** — how this repo links back to the TVF/RRC-CBF/AHTD thesis.

At the end I do a hard portfolio gap analysis against what hardware-equipped PhD admissions committees actually look for.

# Repo 5 — `mpc-cbf`

**Tagline:** Discrete-time MPC with CBF constraints (Zeng-Zhang-Sreenath ACC 2021 + CDC 2021 reproduced in C++/acados, plus tube-MPC-CBF extension).
**Language:** C++ (65%) + Python (35%).
**Estimated size at v1.0:** ~3,500 LOC.
**Why this exists:** Demonstrates you can integrate two control paradigms (predictive horizon + safety filter) and reason about feasibility/safety jointly. This is the closest single-agent precursor to your thesis topic.

## Directory structure

```
mpc-cbf/
├── .github/workflows/colcon_build.yml
├── mpc_cbf_unified/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/mpc_cbf_unified/
│   │   ├── mpc_cbf_node.hpp
│   │   ├── mpc_cbf_solver.hpp
│   │   ├── tube_mpc_cbf_solver.hpp
│   │   └── disturbance_sets.hpp
│   ├── src/
│   │   ├── mpc_cbf_node.cpp
│   │   ├── mpc_cbf_solver.cpp
│   │   └── tube_mpc_cbf_solver.cpp
│   ├── launch/
│   │   ├── 2d_obstacle.launch.py
│   │   ├── car_racing.launch.py
│   │   ├── quadrotor_dynamic_obstacle.launch.py
│   │   └── tube_mpc_disturbance.launch.py
│   ├── config/
│   │   ├── mpc_cbf_params.yaml
│   │   └── tube_mpc_params.yaml
│   ├── test/
│   │   ├── test_mpc_cbf_feasibility.cpp
│   │   ├── test_tube_mpc_robustness.cpp
│   │   └── test_recursive_feasibility.py
├── codegen/
│   ├── generate_mpc_cbf_solver.py
│   └── generate_tube_solver.py
├── reproduction/
│   ├── zeng_acc2021/
│   │   └── reproduce_acc2021.ipynb
│   ├── zeng_cdc2021/
│   │   └── reproduce_cdc2021.ipynb
│   └── REPRODUCTION_REPORT.md
├── analysis/
│   ├── cbfqp_vs_mpccbf_comparison.ipynb
│   ├── feasibility_recovery_study.ipynb
│   └── disturbance_robustness_sweep.ipynb
├── docs/
│   ├── derivations/
│   │   ├── discrete_time_cbf.tex
│   │   ├── mpc_cbf_unified_formulation.tex
│   │   └── tube_mpc_robustness.tex
│   ├── README_math.md
│   └── PRIOR_ART.md
├── media/
│   ├── obstacle_avoidance.gif
│   ├── cbfqp_vs_mpccbf_side_by_side.gif
│   └── tube_robustness.gif
├── LICENSE
└── README.md
```

## File-level descriptions

- **`include/mpc_cbf_unified/mpc_cbf_solver.hpp`** — wraps acados to solve the joint MPC + DT-CBF QP, with diagnostics for which constraint is active when infeasibility is detected.
- **`include/mpc_cbf_unified/tube_mpc_cbf_solver.hpp`** — tube-MPC-CBF for systems with bounded disturbances; uses polytopic RPI sets computed offline.
- **`include/mpc_cbf_unified/disturbance_sets.hpp`** — utilities for constructing and Minkowski-summing zonotope/polytope disturbance sets.
- **`reproduction/zeng_acc2021/reproduce_acc2021.ipynb`** — fully reproduces the Zeng-Zhang-Sreenath ACC 2021 results (2D obstacle avoidance with discrete-time CBFs). **Acts as your "I have read and can implement the foundational papers" credential.**
- **`reproduction/zeng_cdc2021/reproduce_cdc2021.ipynb`** — same for the CDC 2021 enhanced-feasibility paper.
- **`reproduction/REPRODUCTION_REPORT.md`** — explicit table of which figures from each paper you reproduced and what numerical values you matched.
- **`analysis/cbfqp_vs_mpccbf_comparison.ipynb`** — head-to-head: CBF-QP only vs MPC-CBF, on 100 random scenarios. Shows where each fails. **The "I understand the tradeoffs" notebook.**
- **`docs/PRIOR_ART.md`** — annotated bibliography of MPC-CBF integration approaches (Zeng et al., Wabersich-Zeilinger predictive safety filter, Grandia et al. legged-robot multi-layered safety). Ends with "the gap this thesis fills."

## Demo deliverables

- Side-by-side GIF: CBF-QP only fails to avoid a fast-moving obstacle; MPC-CBF succeeds.
- Tube-MPC plot showing constraint satisfaction under wind disturbance.

## CI

- colcon build + tests; reproduction notebooks executed in CI to confirm they still produce expected numbers.

## Citation hook

The MPC-CBF solver here is the per-agent solver inside `transition-viable-swarm` and `transition-viable-swarm`. The reproductions establish baselines you'll cite in your CDC 2027 paper.

---

## Notes added when this file moved into `.deepseek/`

Three points where the implementation documents make a choice this file leaves open or states
differently. Each is recorded here so the divergence is visible from the author's own document:

1. **ROS distro.** This file says Jazzy. The implementation targets **Lyrical Luth**, to share one
   validated toolchain with the author's `cbf-safety-filter`. Reversible in three lines —
   see [02_ENVIRONMENT.md §2.5](02_ENVIRONMENT.md).
2. **Files added beyond this tree.** Six, each because a specified file cannot be written without
   it. Listed in [01_OVERVIEW.md §1.5](01_OVERVIEW.md). Nothing specified here was dropped or
   renamed.
3. **"Fully reproduces"** (the ACC notebook description above) is the target, not a promise the
   implementation may make on its behalf. What was actually matched, and what was not, goes in
   `REPRODUCTION_REPORT.md` with tolerances and a Deviations section
   ([13_DOCS.md §13.5](13_DOCS.md)).
