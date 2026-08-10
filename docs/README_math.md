# Mathematical background

> **SKELETON.** Section headings and the statements to be proved are fixed; the prose and derivations are
> `TODO`. This file is the readable summary; the full derivations live in `docs/derivations/*.tex`.
> See `.deepseek/13_DOCS.md` §13.1.

## Notation

TODO: state space `X ⊆ R^n`, input set `U ⊆ R^m`, discrete dynamics `x_{k+1} = f(x_k, u_k)`,
safe set `C = {x : h(x) ≥ 0}`, class-K function conventions. Keep symbols identical to the `.tex` files
and to the code — `gamma`, `omega`, `N`, `N_CBF`, `Omega` (RPI set), `W` (disturbance set).

## 1. Discrete-time control barrier functions

TODO. State the definition:

> `h` is a discrete-time CBF for `x_{k+1} = f(x_k,u_k)` on `C` if there exists a class-K function `α`
> with `α(r) ≤ r` such that `sup_{u ∈ U} [ h(f(x,u)) − h(x) ] ≥ −α(h(x))` for all `x ∈ C`.

Then the forward-invariance result, its proof sketch, and the linear specialisation `α(h) = γ h` giving

    h(x_{k+1}) ≥ (1 − γ) h(x_k),   γ ∈ (0, 1].

Explain what `γ` means operationally: the fraction of the barrier value the system is permitted to give up
per step; `γ → 0` is maximally conservative, `γ = 1` permits reaching the boundary in one step.

**Why `γ ≤ 1` is not optional.** TODO: show that `γ > 1` permits `h(x_{k+1}) < 0` from `h(x_k) > 0`.

## 2. The MPC-CBF problem

TODO. Write the full OCP (this is the formulation implemented in `codegen/generate_mpc_cbf_solver.py`;
keep them character-for-character consistent):

    min_{u_{0|t..N-1|t}}   Σ_{k=0}^{N-1} q(x_{k|t}, u_{k|t}) + p(x_{N|t})
    s.t.  x_{k+1|t} = f(x_{k|t}, u_{k|t}),          k = 0..N-1
          x_{0|t}   = x_t
          x_{k|t} ∈ X,  u_{k|t} ∈ U
          h(x_{k+1|t}) − h(x_{k|t}) ≥ −γ h(x_{k|t}), k = 0..N_CBF−1

Discuss: why the CBF constraint is imposed along the *prediction*, not only at `k = 0`; what changes when
`N_CBF < N`; and the relationship to MPC-DC (`h(x_{k|t}) ≥ 0` only).

## 3. Relaxed decay rate (CDC 2021)

TODO. The constraint becomes

    h(x_{k+1|t}) − h(x_{k|t}) ≥ −ω_{k|t} γ h(x_{k|t}),   ω_{k|t} ≥ 0

with `ω` a decision variable penalised by `p_ω (ω − 1)^2`. Prove the safety condition:

> If `h(x_{k|t}) ≥ 0` and `ω_{k|t} γ ≤ 1`, then `h(x_{k+1|t}) ≥ 0`.

and explain why this is what makes `omega_max * gamma <= 1` a hard validation check in
`MpcCbfSolver::initialize()` rather than a soft warning.

## 4. Feasibility

TODO — and be careful here, this is the section a reviewer will read closely.

State plainly: **the DCBF constraints do not by themselves confer recursive feasibility.** Explain the two
standard routes (terminal ingredients; sufficiently long horizon with a control-invariant terminal set),
state which one this repo implements, and present the empirical feasible-set study
(`analysis/feasibility_recovery_study.ipynb`) as *measurement*, not as proof.

## 5. Tube MPC-CBF

TODO. For `x_{k+1} = f(x_k,u_k) + w_k`, `w_k ∈ W`:

1. Nominal trajectory `z`, ancillary law `u = v + K(x − z)`.
2. Error dynamics `e_{k+1} = (A + BK) e_k + w_k`, RPI set `Ω` with `(A+BK) Ω ⊕ W ⊆ Ω`.
3. Constraint tightening: `X ⊖ Ω`, `U ⊖ KΩ`.
4. **Barrier tightening** — the part specific to this repo. Derive the margin `c` such that

       h(z) − c ≥ 0  ⟹  h(z + e) ≥ 0  ∀ e ∈ Ω

   both for the support-function form (exact for affine `h`, plus the quadratic remainder term for the
   squared-distance barrier actually used) and the Lipschitz form
   `c = L_h · max{‖e‖ : e ∈ Ω}`. Show the support form is never more conservative than the Lipschitz one.
5. The resulting robust DCBF condition, and the statement of what is guaranteed: safety of the **true**
   state for all admissible disturbance sequences, given feasibility at `t = 0`.

## 6. What this repository does *not* prove

TODO. Be explicit — this section is worth more than the rest to a careful reader:

- No recursive-feasibility certificate (see §4).
- `Ω` is exact only for the linear model; for the nonlinear plants it is computed at a linearisation point
  and is therefore a local certificate. Say where.
- Obstacle motion is predicted as constant-velocity; prediction error is not in `W` unless you put it there.
- Nothing here addresses multi-agent coupling — that is the thesis, not this repo.

## References

TODO: full bibliographic entries, plus a pointer to `docs/PRIOR_ART.md` for the annotated version.
