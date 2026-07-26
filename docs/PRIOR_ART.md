# Prior art — MPC + CBF integration

> **SKELETON.** Structure and the entries to cover are fixed; annotations are `TODO`. Each entry needs:
> full citation, the one-sentence idea, what it assumes, what it guarantees, and how it relates to this
> repository. Aim for 3-6 sentences per entry — an annotated bibliography, not a list.
> See `IMPLEMENTATION_GUIDE.md` §12.3.

## 1. Foundations — continuous-time CBFs

- **Ames, Xu, Grizzle, Tabuada (2017), "Control Barrier Function Based Quadratic Programs for Safety
  Critical Systems", IEEE TAC.** TODO.
- **Ames et al. (2019), "Control Barrier Functions: Theory and Applications", ECC.** TODO — the survey to
  cite for the CBF-QP baseline used in `analysis/cbfqp_vs_mpccbf_comparison.ipynb`.
- **Xu, Tabuada, Grizzle, Ames (2015), "Robustness of Control Barrier Functions for Safety Critical
  Control".** TODO — the ISSf notion this repo's tube approach is an alternative to.

## 2. Discrete-time CBFs

- **Agrawal & Sreenath (2017), "Discrete Control Barrier Functions for Safety-Critical Control of Discrete
  Systems with Application to Bipedal Robot Navigation", RSS.** TODO — origin of the discrete-time
  formulation this repo implements.
- **Zeng, Zhang, Sreenath (2021), "Safety-Critical Model Predictive Control with Discrete-Time Control
  Barrier Function", ACC.** TODO — reproduced in `reproduction/zeng_acc2021/`.
- **Zeng, Li, Sreenath (2021), "Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with
  Discrete-Time Control Barrier Functions", CDC.** TODO — reproduced in `reproduction/zeng_cdc2021/`.

## 3. Predictive safety filters

- **Wabersich & Zeilinger (2018/2021), "Predictive Safety Filter"/"Linear Model Predictive Safety
  Certification".** TODO — the main alternative philosophy: keep the performance controller arbitrary and
  certify its output with a backup MPC. Contrast the guarantee it gives with the one here.
- **Wabersich, Taylor, Choi, Sreenath, Tomlin, Ames, Zeilinger (2023), "Data-Driven Safety Filters",
  IEEE CSM.** TODO — situates both families.

## 4. Robust and tube MPC

- **Mayne, Seron, Raković (2005), "Robust model predictive control of constrained linear systems with
  bounded disturbances", Automatica.** TODO — the tube construction adapted in
  `tube_mpc_cbf_solver.cpp`.
- **Raković, Kerrigan, Kouramas, Mayne (2005), "Invariant approximations of the minimal robust positively
  invariant set", IEEE TAC.** TODO — the algorithm in `computeRpiSet()`.
- **Köhler, Soloperto, Müller, Allgöwer (2021), "A computationally efficient robust model predictive
  control framework for uncertain nonlinear systems".** TODO — what to cite when extending the tube beyond
  the linearised model.

## 5. Robust / stochastic CBFs

- **Jankovic (2018), "Robust control barrier functions for constrained stabilization of nonlinear
  systems".** TODO.
- **Cosner, Singletary, Taylor, Molnar, Bouman, Ames (2021+), measurement-robust and stochastic CBF
  variants.** TODO — the alternative to constraint tightening when the disturbance is stochastic rather
  than bounded, and why this repo chose bounded.

## 6. Multi-layered safety on real robots

- **Grandia, Taylor, Ames, Hutter (2021), "Multi-Layered Safety for Legged Robots via Control Barrier
  Functions and Model Predictive Control", ICRA.** TODO — the closest hardware-validated relative of this
  work; note explicitly what they do at each layer and at what rates.
- **Grandia et al. (2023), perceptive locomotion follow-ups.** TODO.

## 7. Distributed / multi-agent MPC-CBF

- **Distributed CBF-QP formulations (Wang, Ames, Egerstedt; Lindemann & Dimarogonas).** TODO — where
  pairwise safety constraints get split between agents, and what breaks when the split is asynchronous.
- **Decentralised MPC with coupled constraints (Richards & How; Trodden & Richards).** TODO.

## The gap this thesis fills

> TODO — the most important paragraph in this file. Roughly three paragraphs, in this shape:
>
> 1. **What is solved.** Single-agent MPC-CBF is well understood: this repository reproduces it, and the
>    tube extension handles bounded disturbance for one agent.
> 2. **What is not.** Every formulation above assumes a *fixed* agent set and *fixed* constraint topology.
>    When agents split, merge, or morph a formation, the constraint set itself changes at an event time,
>    the state jumps to a set-valued reset, and neither the CBF invariance argument nor the tube's RPI
>    certificate survives the discontinuity. Robustness results assume disturbances inside a fixed `W`,
>    not a change in problem dimension.
> 3. **The claim.** *Transition-Viable Distributed MPC-CBF* — a hybrid-systems account of asynchronous
>    split/merge/morph with set-valued resets, with per-agent solvers of exactly the kind implemented here.
>    State the concrete deliverable: viability conditions at transitions, plus a distributed solver whose
>    per-agent inner problem is this repo's `MpcCbfSolver`.
>
> Write it so a reviewer can locate the delta in one read. No hedging, no "future work will explore".
