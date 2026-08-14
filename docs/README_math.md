# Mathematical background

The readable summary. The full derivations live in `docs/derivations/*.tex`; this file states the
definitions and the results, and says where each one is proven. Symbols are kept identical to the
code and to the `.tex` files: `gamma`, `omega`, `N`, `N_CBF`, `Omega` (the RPI set), `W` (the
disturbance set).

## Notation

- State space `X ⊆ R^n`, input set `U ⊆ R^m`, discrete dynamics `x_{k+1} = f(x_k, u_k)`.
- Safe set `C = {x ∈ X : h(x) ≥ 0}` for a barrier `h : X → R`.
- A class-K function `α : R_≥0 → R_≥0` is continuous, strictly increasing, with `α(0) = 0`.
- `P` is the position selector: `P x` picks the position coordinates out of `x` (for the double
  integrator, `x = [px, py, vx, vy]` and `P x = [px, py]`).

## 1. Discrete-time control barrier functions

> `h` is a discrete-time control barrier function (DCBF) for `x_{k+1} = f(x_k, u_k)` on `C` if there
> exists a class-K function `α` with `α(r) ≤ r` such that
>
>     sup_{u ∈ U} [ h(f(x, u)) − h(x) ] ≥ −α(h(x))     for all x ∈ C.

**Forward invariance.** If `h` is a DCBF and `u_k` satisfies the DCBF inequality at every `k`, then
`x_0 ∈ C` implies `x_k ∈ C` for all `k ≥ 0`. The proof is two lines and is the entire safety argument
(see `derivations/discrete_time_cbf.tex`, §3):

    h(x_{k+1}) ≥ h(x_k) − α(h(x_k)) ≥ h(x_k) − h(x_k) = 0,

using `α(r) ≤ r` and `h(x_k) ≥ 0` in the second inequality. The condition `α(r) ≤ r` is not a
notational convenience: without it the induction step is vacuous.

**Linear specialisation.** Setting `α(h) = γ h` with `γ ∈ (0, 1]` gives

    h(x_{k+1}) ≥ (1 − γ) h(x_k),   γ ∈ (0, 1],

hence the exponential envelope `h(x_k) ≥ (1−γ)^k h(x_0)`. Operationally, `γ` is the fraction of
the barrier value the system is permitted to give up per step: `γ → 0` is maximally conservative,
`γ = 1` permits reaching the boundary in one step. This envelope is overlaid on the `h(x_k)` traces
in `reproduction/zeng_acc2021` (§3) — it is the visual form of the invariance argument.

**Why `γ ≤ 1` is not optional.** If `γ > 1`, then `1 − γ < 0`, and a state with `h(x_k) > 0` admits
`h(x_{k+1}) ≥ (1−γ) h(x_k) < 0`; the constraint would *require* a positive state to jump to a
negative one. Concretely, the implementation rejects `γ > 1` at configuration load
(`MpcCbfSolver::initialize()`), not at solve time.

## 2. The MPC-CBF problem

The formulation implemented in `codegen/generate_mpc_cbf_solver.py`:

    min_{u_{0|t..N-1|t}}   Σ_{k=0}^{N-1} [ ‖x_{k|t} − x^ref_k‖²_Q + ‖u_{k|t}‖²_R ] + ‖x_{N|t} − x^ref_N‖²_{Q_f}
    s.t.  x_{k+1|t} = f(x_{k|t}, u_{k|t}),           k = 0..N-1
          x_{0|t}   = x_t
          x_{k|t} ∈ X,   u_{k|t} ∈ U
          h_j(x_{k+1|t}) − h_j(x_{k|t}) ≥ −γ h_j(x_{k|t}),   k = 0..N_CBF−1,  for each obstacle j.

The DCBF condition is imposed **along the prediction** (`k = 0..N_CBF−1`), not only at `k = 0`.
Imposing it only at `k = 0` is exactly the one-step CBF-QP filter: the controller must pay the
entire barrier decay immediately and cannot see that a manoeuvre costs barrier now but recovers it
later. A horizon lets the optimiser trade decay across stages — this is the mechanism measured in
`analysis/feasibility_recovery_study.ipynb` and in the `N_CBF` sweep of `reproduction/zeng_cdc2021`
(§4). When `N_CBF < N`, the constraint is dropped for `k ≥ N_CBF`; the relationship to MPC-DC (which
keeps only `h(x_{k|t}) ≥ 0` and no decay row) is one special case of the same table — see
`derivations/mpc_cbf_unified_formulation.tex`, §1.

## 3. Relaxed decay rate (CDC 2021)

The decay rate becomes a decision variable `ω_{k|t} ≥ 0`:

    h(x_{k+1|t}) − h(x_{k|t}) ≥ −ω_{k|t} γ h(x_{k|t}),   ω_{k|t} ≥ 0,

penalised by `p_ω (ω − 1)²` so that relaxation is used only where it is needed.

**Safety condition.** If `h(x_{k|t}) ≥ 0` and `ω_{k|t} γ ≤ 1`, then `h(x_{k+1|t}) ≥ 0`:

    h(x_{k+1|t}) ≥ h(x_{k|t}) − ω γ h(x_{k|t}) = (1 − ωγ) h(x_{k|t}) ≥ 0.

This is why `omega_max * gamma ≤ 1` is a hard validation check in `MpcCbfSolver::initialize()`
rather than a soft warning: a relaxation that can push a non-negative state negative would
silently void the safety claim that the relaxed formulation is supposed to *retain*, not trade away.

## 4. Feasibility

**The DCBF constraints do not by themselves confer recursive feasibility.** Feasibility of the
problem at time `t` does not imply feasibility at time `t+1`; the feasible set is not forward
invariant. There are two standard sufficient conditions that would close this gap:

1. a **terminal ingredient** — a control-invariant terminal set contained in `C`, with a terminal
   cost that is a control Lyapunov function on it; or
2. a horizon long enough that the terminal state can be driven into such a set.

This repository implements **neither**. What it does is measure the gap: the closed-loop study in
`analysis/feasibility_recovery_study.ipynb` records where fixed-decay MPC-CBF loses feasibility and
scores recovery strategies (slack, relaxed decay, horizon backoff, the node's default
`previous_horizon`) on **both** recovery rate and worst barrier value. That study is *measurement*,
not proof — it documents exactly how far the implementation is from a recursive-feasibility
certificate, and it shows that the node's default recovery strategy is the one strategy that can
recover by going unsafe.

## 5. Tube MPC-CBF

For `x_{k+1} = f(x_k, u_k) + w_k` with bounded disturbance `w_k ∈ W`:

1. **Nominal trajectory** `z` with `z_{k+1} = A z_k + B v_k`; the **ancillary law**
   `u = v + K(x − z)` steers the true state toward the nominal.
2. **Error dynamics** `e_{k+1} = (A + BK) e_k + w_k`. A robust positively invariant (RPI) set `Ω`
   satisfies `(A+BK) Ω ⊕ W ⊆ Ω`; then `e_0 ∈ Ω` implies `e_k ∈ Ω` for all `k`.
3. **Constraint tightening** `X ⊖ Ω` and `U ⊖ KΩ`, computed in H-representation via support
   functions (Pontryagin difference).
4. **Barrier tightening** — the part specific to this repository. Find a margin `c_j(z)` such that

       h_j(z) − c_j(z) ≥ 0  ⟹  h_j(z + e) ≥ 0   ∀ e ∈ Ω.

   For the squared-distance barrier `h_j(x) = ‖P x − p_j‖² − r_j²` the exact requirement is
   `c_j(z) ≥ sup_{e ∈ Ω} [h_j(z) − h_j(z+e)]`. Expanding,

       h_j(z) − h_j(z+e) = −2 (Pz − p_j)ᵀ P e − ‖P e‖².

   Dropping the negative quadratic term `−‖Pe‖² ≤ 0` over-estimates the margin, so the
   support-function form

       c_j(z) = h_Ω( −2 Pᵀ (Pz − p_j) )

   is **sound** (never too small). The Lipschitz form `c_j = L_h · max{‖e‖ : e ∈ Ω}` is a valid
   but never-less-conservative alternative: `c_j^support ≤ c_j^Lipschitz` (proven in
   `derivations/tube_mpc_robustness.tex`, §5). The support form is the default tightening mode.
5. **Robust DCBF condition.** The nominal problem carries

       h_j(z_{k+1}) − h_j(z_k) ≥ −γ ( h_j(z_k) − c_{j,k} ),    h_j(z_k) − c_{j,k} ≥ 0.

   Given feasibility at `t = 0`, `e_0 ∈ Ω`, and no input saturation, the **true** state satisfies
   `h_j(x_t) ≥ 0` for all `t` and all admissible disturbance sequences. Non-saturation is a real
   hypothesis — the implementation monitors it with a clip counter and the tube's guarantee is
   void where it is violated.

## 6. What this repository does *not* prove

Be explicit; this section is worth more than the rest to a careful reader.

- **No recursive-feasibility certificate** (see §4). Feasibility today does not imply feasibility
  tomorrow, for any of the three layers.
- **`Ω` is exact only for the linear model.** For `double_integrator_2d` the discretisation is the
  exact ZOH map, so the RPI certificate is global. For the nonlinear plants (`bicycle_kinematic`,
  `quadrotor_planar`) `Ω` is computed at a linearisation point and is therefore a *local*
  certificate — valid near that operating point, not everywhere.
- **Obstacle motion is predicted as constant-velocity.** Prediction error is *not* in `W` unless
  the caller deliberately adds it.
- **State-estimation error is not modelled.** The tube accounts for the process disturbance `w`
  only; measurement noise is outside its scope.
- **Nothing here addresses multi-agent coupling.** Distributed split/merge/morph, set-valued
  resets, and changing constraint topology are the thesis this repository feeds, not something it
  implements. See `docs/PRIOR_ART.md` for the delta.

## References

Full bibliographic entries are in the `.tex` files and, annotated, in `docs/PRIOR_ART.md`:

- Agrawal & Sreenath (2017), *Discrete Control Barrier Functions*, RSS.
- Ames, Xu, Grizzle, Tabuada (2017), *CBF Based Quadratic Programs*, IEEE TAC.
- Ames et al. (2019), *Control Barrier Functions: Theory and Applications*, ECC.
- Zeng, Zhang & Sreenath (2021), *Safety-Critical MPC with DT-CBF*, ACC.
- Zeng, Li & Sreenath (2021), *Enhancing Feasibility and Safety of NMPC with DT-CBF*, CDC.
- Mayne, Seron & Raković (2005), *Robust MPC of constrained linear systems*, Automatica.
- Raković, Kerrigan, Kouramas & Mayne (2005), *Invariant approximations of the minimal RPI set*,
  IEEE TAC.
