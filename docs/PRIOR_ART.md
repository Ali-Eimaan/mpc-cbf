# Prior art — MPC + CBF integration

## 1. Foundations — continuous-time CBFs

- **Ames, Xu, Grizzle, Tabuada (2017), "Control Barrier Function Based Quadratic Programs for Safety
  Critical Systems", IEEE TAC.** Introduces the control barrier function and the CBF-QP: a pointwise
  quadratic program that takes a nominal control law and returns the nearest safe input, guaranteeing
  forward invariance of the superlevel set C = {x : h(x) ≥ 0} through the condition ḣ ≥ −α(h). Assumes a
  control-affine model, full state feedback, and a known h with Lipschitz gradient. The guarantee is
  continuous-time and one-step (no lookahead), so the filter can be myopic or infeasible near input
  limits. This repo's MPC-CBF is the discrete-time, lookahead generalisation of exactly this filter.
- **Ames et al. (2019), "Control Barrier Functions: Theory and Applications", ECC.** The survey to cite
  for the CBF-QP baseline used in `analysis/cbfqp_vs_mpccbf_comparison.ipynb`. Unifies CLF and CBF
  theory, distinguishes reciprocal from zeroing CBFs, and standardises the safety-critical QP as a
  minimally invasive filter on a nominal controller. It fixes the notation (class-K α, superlevel set C)
  this repo adopts verbatim in `docs/README_math.md`. The notebook's baseline shows the one-step filter
  colliding with a fast obstacle where the MPC-CBF lookahead clears it.
- **Xu, Tabuada, Grizzle, Ames (2015), "Robustness of Control Barrier Functions for Safety Critical
  Control".** Introduces input-to-state safety (ISSf), the first robustness notion for CBFs: under
  bounded, matched disturbance the superlevel set is not exactly invariant but degrades to an inflated
  set bounded by an ISSf gain. It is the continuous-time alternative to this repo's tube approach —
  robustness via a single analytical margin rather than an RPI set and constraint tightening. This repo
  chose the tube because the disturbance is additive on the discrete map, where the RPI tightening is
  exact rather than a sufficient bound; the contrast is stated in `tube_mpc_robustness.tex`.

## 2. Discrete-time CBFs

- **Agrawal & Sreenath (2017), "Discrete Control Barrier Functions for Safety-Critical Control of Discrete
  Systems with Application to Bipedal Robot Navigation", RSS.** Origin of the discrete-time CBF:
  h(x_{k+1}) − h(x_k) ≥ −γ h(x_k) with γ ∈ (0, 1], which forward-invariates the superlevel set on the
  discrete map and admits the envelope h(x_k) ≥ (1−γ)^k h(x_0). Assumes a discrete-time model and a
  barrier affine in the (known) control for QP solvability. This repo implements exactly that row —
  `dcbf_constraint()` in the codegen, derived in `docs/derivations/discrete_time_cbf.tex`. The bipedal
  navigation demonstration is the same application class as this repo's racing demo.
- **Zeng, Zhang, Sreenath (2021), "Safety-Critical Model Predictive Control with Discrete-Time Control
  Barrier Function", ACC.** Embeds the Agrawal–Sreenath DCBF as a hard constraint at every stage of a
  nonlinear MPC, so safety is enforced over the whole prediction horizon rather than one step at a time.
  Demonstrates on a double integrator and a car-racing example that MPC-CBF stays feasible where a
  short-horizon MPC with the same barrier fails. Reproduced in `reproduction/zeng_acc2021/` (see
  REPRODUCTION_REPORT.md for the figure-by-figure account). This repo's `MpcCbfSolver` is this
  formulation.
- **Zeng, Li, Sreenath (2021), "Enhancing Feasibility and Safety of Nonlinear Model Predictive Control with
  Discrete-Time Control Barrier Functions", CDC.** Relaxes the fixed decay rate: the multiplier becomes a
  per-step decision variable ω_k with ω_k γ ≤ 1, and the DCBF is applied over a shorter horizon
  N_CBF ≤ N, enlarging the feasible set without losing the h ≥ 0 guarantee. The safety argument —
  (1 − ω_k γ) ≥ 0 preserves the forward-invariance induction — is reproduced in
  `docs/derivations/mpc_cbf_unified_formulation.tex`. Reproduced in `reproduction/zeng_cdc2021/`; this is
  the repo's "relaxed decay" mode.

## 3. Predictive safety filters

- **Wabersich & Zeilinger (2018/2021), "Predictive Safety Filter"/"Linear Model Predictive Safety
  Certification".** The main alternative philosophy: keep the performance controller arbitrary and wrap it
  in a safety filter that, at each step, solves a backup MPC to certify (or override) the candidate
  input. Its guarantee is online feasibility of the backup problem, not a hard constraint inside the
  performance MPC. The contrast: it isolates safety from performance at the cost of a second online
  optimisation, whereas this repo folds safety into the performance MPC's constraints. The repo's tube
  solver is closest to their linear predictive safety certificate.
- **Wabersich, Taylor, Choi, Sreenath, Tomlin, Ames, Zeilinger (2023), "Data-Driven Safety Filters",
  IEEE CSM.** Survey situating both safety-filter families (CBF-QP and predictive/MPC filters) and adding
  data-driven variants that learn the safety constraint from samples. Useful as the field map when
  positioning this repo: model-based MPC-CBF, with the tube MPC extension as the robustness layer. Cited
  to anchor the taxonomy in the introduction and this bibliography.

## 4. Robust and tube MPC

- **Mayne, Seron, Raković (2005), "Robust model predictive control of constrained linear systems with
  bounded disturbances", Automatica.** Defines tube MPC: a nominal trajectory generated by an MPC that
  ignores disturbance, an ancillary feedback u = K(x − z) that keeps the true state in a tube around the
  nominal, and an RPI set Ω that certifies the tube. Assumes a linear time-invariant plant with additive
  disturbance in a compact set W. Adapted directly in `tube_mpc_cbf_solver.cpp` (nominal z, ancillary K,
  error e = (A+BK)e + w); the tightening in `docs/derivations/tube_mpc_robustness.tex` is this
  machinery.
- **Raković, Kerrigan, Kouramas, Mayne (2005), "Invariant approximations of the minimal robust positively
  invariant set", IEEE TAC.** The algorithm for the minimal robust positively invariant (mRPI) set:
  Ω = ⊕_{i=0}^{s−1} A^i W ⊕ F_α^s, the invariant approximation `computeRpiSet()` implements. Gives the
  F_α^s construction and the diagonal closed form w/(1−λ) used for the error-box analysis. Assumes A is
  Schur. This is the exact certificate the tube's safety theorem leans on.
- **Köhler, Soloperto, Müller, Allgöwer (2021), "A computationally efficient robust model predictive
  control framework for uncertain nonlinear systems".** A tube/robust MPC framework for uncertain
  nonlinear systems, extending the Mayne–Seron–Raković machinery past LTI plants via incremental
  stability estimates instead of an exact RPI set. This is what to cite when the repo's tube is extended
  beyond the linearised model — the current implementation linearises (A, B) per solve, so Ω is local and
  the nonlinear mismatch is absorbed as a modelling gap (§6 of `docs/README_math.md`).

## 5. Robust / stochastic CBFs

- **Jankovic (2018), "Robust control barrier functions for constrained stabilization of nonlinear
  systems".** Robust CBFs for constrained stabilisation: guarantees forward invariance of the safe set
  when the plant carries a matched disturbance, generalising the nominal CBF condition with a robustness
  margin. It is the continuous-time, single-agent precursor to the robust-DCBF row in
  `tube_mpc_robustness.tex`. The repo's discrete-time, set-based tightening is the discrete counterpart
  of Jankovic's margin.
- **Cosner, Singletary, Taylor, Molnar, Bouman, Ames (2021+), measurement-robust and stochastic CBF
  variants.** Measurement-robust and stochastic CBFs that trade the worst-case bound for a probabilistic
  or expected-value guarantee when the disturbance is stochastic rather than set-bounded. This repo
  deliberately chose bounded disturbance: the tube's RPI certificate is exact for a fixed W, and the
  path-overhead cost is quantified (12.27 %) rather than left to a probabilistic risk budget. Cited as
  the rejected alternative and the reason it was rejected.

## 6. Multi-layered safety on real robots

- **Grandia, Taylor, Ames, Hutter (2021), "Multi-Layered Safety for Legged Robots via Control Barrier
  Functions and Model Predictive Control", ICRA.** The closest hardware-validated relative: a layered
  stack with a 10 Hz whole-body MPC planning dynamically feasible motion and a 1 kHz CBF-QP safety filter
  enforcing collision/contact constraints on top of it. Explicitly separates rates and roles — planner
  for optimality, filter for safety. This repo mirrors the layering (MPC for performance, CBF rows for
  safety) but folds them into one discrete-time OCP rather than two control loops, removing the
  filter/planner mismatch at the sampling boundary.
- **Grandia et al. (2023), perceptive locomotion follow-ups.** Follow-ups that feed onboard perception
  into the CBF layer, showing the MPC-CBF stack survives noisy, real-time state estimates. Cited to close
  the loop from theory to deployed perceptive systems — the repo's constant-velocity obstacle prediction
  (§6 caveat in `docs/README_math.md`) is the unmodelled gap these works fill with learned or measured
  perception.

## 7. Distributed / multi-agent MPC-CBF

- **Distributed CBF-QP formulations (Wang, Ames, Egerstedt; Lindemann & Dimarogonas).** Split pairwise
  safety constraints between the two agents sharing each pair, each agent solving a local QP from local
  (or neighbour) state. The safety argument assumes the split is synchronous and the topology fixed: each
  pairwise QP sees the other agent's true state at the same instant. What breaks under asynchrony is
  exactly what this thesis targets — a message delay or a split leaves an agent's local view stale, and
  the shared constraint is violated before the next exchange. Listed as the immediate predecessor of the
  distributed formulation.
- **Decentralised MPC with coupled constraints (Richards & How; Trodden & Richards).** Agents optimise
  locally while a consensus or constraint-duplication scheme enforces the shared (collision) constraints,
  with feasibility certificates under a fixed topology. The coupling is handled by duplicating shared
  variables and adding consistency terms. Same fixed-topology assumption: when agents split or merge, the
  duplicated constraint set changes dimension mid-horizon and the certificate lapses. This repo's per-agent
  `MpcCbfSolver` is the building block such a distributed scheme would call.

## The gap this thesis fills

**What is solved.** Single-agent MPC-CBF is well understood and reproduced here: `reproduction/zeng_acc2021/`
and `reproduction/zeng_cdc2021/` match the ACC and CDC results to tolerance, and the tube extension
(`tube_mpc_cbf_solver.cpp`) certifies safety for one agent under bounded disturbance in a fixed `W`, at a
measured 12.27 % path-overhead cost.

**What is not.** Every formulation above assumes a *fixed* agent set and a *fixed* constraint topology.
When agents split, merge, or morph a formation, the constraint set changes at an event instant, the state
undergoes a set-valued reset, and neither the CBF forward-invariance induction nor the tube's RPI
certificate survives the discontinuity: the induction's `h(x_k)` refers to a state whose dimension is
about to change, and the RPI set `Ω` is certified for a fixed `(A, B, W)`. Robustness results assume
disturbances inside a fixed `W`, not a change in problem dimension or constraint graph.

**The claim.** *Transition-Viable Distributed MPC-CBF*: a hybrid-systems account of asynchronous
split/merge/morph with set-valued resets. The deliverable is a set of viability conditions at transitions
— precisely when a set-valued post-reset state remains in the safe set so the next per-agent MPC-CBF solve
is feasible — plus a distributed solver whose per-agent inner problem is this repo's `MpcCbfSolver`, the
fixed-topology building block validated here.
