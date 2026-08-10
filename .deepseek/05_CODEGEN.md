# §5 · acados code generation

**Governs:** `codegen/generate_mpc_cbf_solver.py`, `codegen/generate_tube_solver.py`
**Milestone:** M2 (nominal), M7 (tube)
**Done when:** all six nominal configurations generate and compile; a generated solver solves the
2-D scenario from Python before any C++ exists.

Before you start: **verify every acados API name below against
`${ACADOS_SOURCE_DIR}/examples/acados_python/` for the version you installed** (risk V10,
[§2.1](02_ENVIRONMENT.md)). Names change between releases. Where one differs, use the installed
spelling and correct this document in the same commit.

---

## §5.1 What is baked in and what is a runtime parameter

| Baked into the generated code (regeneration required) | Runtime parameter (no regeneration) |
| --- | --- |
| model, `nx`, `nu` | obstacle positions, velocities, radii |
| horizon `N`, CBF horizon `N_CBF` | `γ` |
| `dt` | cost weights `Q`, `R`, `Qf` |
| obstacle count `n_obs` | reference trajectory |
| `CbfVariant` | state and input bounds |
| tube tightening structure | tightening values `c_{j,k}` |

Getting this table wrong in either direction is expensive: baking `γ` in means a regeneration per
tuning step, and trying to make `N` a parameter means fighting acados' static dimensions.

## §5.2 `build_acados_model()`

- `model.x`, `model.u` from the `ModelSpec` ([§4.2](04_MODELS.md)); `model.p` per §5.3.
- **`model.disc_dyn_expr = discretise(spec, dt)` and
  `ocp.solver_options.integrator_type = 'DISCRETE'`.**

  This is not a performance choice. The DCBF condition constrains `h(x_{k+1})` where `x_{k+1}` is
  the *discrete* successor. An `ERK`/`IRK` integrator makes acados enforce the condition on an
  internally integrated state, which changes what the constraint means and quietly invalidates
  the invariance argument in [§4.5](04_MODELS.md). If you find `ERK` anywhere in this repository,
  it is a bug.

- `model.name = f"mpc_cbf_{model_key}_N{horizon}_{variant}"`, e.g.
  `mpc_cbf_double_integrator_2d_N8_fixed_decay`. The C++ side selects a generated solver by this
  string, and distinct names are what keep six configurations' symbols from colliding in one
  library ([03_BUILD_SYSTEM.md §3.3](03_BUILD_SYSTEM.md)).

## §5.3 The parameter vector — fix this once, mirror it everywhere

Per stage:

```
index                    contents
0 … 7·n_obs−1            per obstacle j: [ox, oy, oz, vx, vy, vz, radius]      (7 doubles)
7·n_obs                  gamma
                         ── tube solver only ──
7·n_obs+1 … +n_obs       tightening c_j for this stage                          (n_obs doubles)
```

With `n_obs = 8`: `np = 57` nominal, `np = 65` tube. Put both numbers in a comment at the top of
the model builder **and** in `MpcCbfSolver::Impl` ([06_SOLVER.md §6.1](06_SOLVER.md)).

Two rules that follow:

1. **Obstacle positions are pushed already propagated to the stage.** For stage `k`,
   `o_j(k) = o_j(0) + k·dt·v_j` when the obstacle is dynamic, else `o_j(0)`. The velocity entries
   ride along for diagnostics and future use; the *expression* uses only the position and radius.
2. **The radius pushed is `r_eff`**, inflation already applied by the caller
   ([§4.4](04_MODELS.md)). The generated expression must not inflate again.

Unused obstacle slots are padded by the caller with a far-away dummy
(`position = (1e6, 1e6, 1e6)`, `radius = 0`) so the vector is always full and the generated code
never sees a NaN ([06_SOLVER.md §6.4](06_SOLVER.md)).

## §5.4 `build_ocp()` — assembly order

1. `ocp.model = build_acados_model(...)`; `ocp.dims.N = horizon`.
2. **Cost:** `NONLINEAR_LS`, `y = vertcat(x, u)`, `y_e = x`; `W = blkdiag(diag(Q), diag(R))`,
   `W_e = diag(Qf)`. Weights are runtime-settable, so the YAML defaults go in here.
3. **Bounds:** `lbu/ubu/idxbu` for all inputs; `lbx/ubx/idxbx` for the finite state bounds only —
   skip entries set to `±1e9` in the YAML rather than passing a huge number acados must carry
   through every QP.
4. **Initial state:** `ocp.constraints.x0 = zeros(nx)`, overwritten every solve.
5. **Barrier rows**, via `ocp.model.con_h_expr` (stages `1 … N−1`) and `con_h_expr_e` (stage `N`):

   | Row type | Expression | `lh` | `uh` |
   | --- | --- | --- | --- |
   | distance | `h_j(x_k)` | `0` | `+BIG` |
   | DCBF, `k < N_CBF` | `h_j(F(x_k,u_k)) − h_j(x_k) + γ h_j(x_k)` | `0` | `+BIG` |

   **Emit both.** The DCBF row alone preserves safety only *from* a safe state; the distance row
   is what makes an already-unsafe state report infeasible instead of tracking a negative barrier
   downward ([01_OVERVIEW.md §1.4](01_OVERVIEW.md)).

   **Row order is obstacle-major within each row type**, and the same order is assumed by
   `SolverDiagnostics::cbf_values` (index `k·n_obs + j`). Write the layout in a comment in both
   places; a mismatch here makes every diagnostic point at the wrong obstacle without failing
   anything.

   Use a large finite `uh` (`1e9`), not `inf` — see [16_CONVENTIONS.md §16.4](16_CONVENTIONS.md).

6. **Stage 0.** The initial state is fixed, so `h_j(x_0) ≥ 0` is a constant row. Emit it anyway:
   it is what makes an unsafe start surface as infeasible, which
   `InfeasibilityReasonIsPopulated` tests. Expect acados to report it immediately.
7. **Choice of `x_{k+1}`.** Two equivalent implementations exist: substitute `F(x_k, u_k)` into
   the expression, or write the row against the next shooting node's state. **Pick one, use it
   everywhere, and say which in a comment at the top of the generated model builder.** Mixing
   them across variants produces two subtly different problems that are very hard to tell apart
   from their outputs.
8. **Solver options:**

   ```python
   qp_solver            = 'PARTIAL_CONDENSING_HPIPM'
   hessian_approx       = 'GAUSS_NEWTON'
   integrator_type      = 'DISCRETE'
   nlp_solver_type      = 'SQP'          # 'SQP_RTI' with --rti
   nlp_solver_max_iter  = max_sqp_iterations
   levenberg_marquardt  = 1e-4
   qp_solver_warm_start = 1
   print_level          = 0
   ```

## §5.5 The relaxed-decay variant

`variant == "relaxed_decay"`:

- Extend the input vector by `n_obs` variables `ω_j`, so the generated `nu` becomes `nu + n_obs`.
  The physical `nu` is unchanged as far as every caller is concerned
  ([06_SOLVER.md §6.2](06_SOLVER.md)).
- Bound them: `ω_j ∈ [omega_min, omega_max]`.
- Cost: append `sqrt(omega_weight)·(ω_j − 1)` rows to `y`, so the least-squares cost contributes
  `omega_weight·(ω_j − 1)²`.

  **Penalise `(ω − 1)²`, not `ω`.** Penalising `ω` biases the solver toward `ω = 0`, which is
  *stricter* than the nominal problem — you would ship a "feasibility enhancement" that makes
  feasibility worse. See [16_CONVENTIONS.md §16.1](16_CONVENTIONS.md), trap 2.
- Use the `ω` form of `dcbf_constraint`.

`variant == "distance_only"`: emit only the distance rows. This is the MPC-DC baseline; it is
supposed to be worse, and A5 depends on it being implemented faithfully rather than
handicapped. Give it the same cost, the same bounds and the same horizon as the comparison
demands — the only difference is the missing decay row.

## §5.6 The generation matrix

`generate_all()` MUST emit exactly these. CI builds this list, and the launch files and tests
select from it by name.

| model | N | variant | consumed by |
| --- | --- | --- | --- |
| `double_integrator_2d` | 8 | `fixed_decay` | `2d_obstacle.launch.py`, most gtests, ACC notebook |
| `double_integrator_2d` | 3 | `distance_only` | the MPC-DC baseline (A5) |
| `double_integrator_2d` | 8 | `relaxed_decay` | CDC notebook, A6 |
| `bicycle_kinematic` | 11 | `fixed_decay` | `car_racing.launch.py` |
| `quadrotor_planar` | 15 | `fixed_decay` | dynamic-obstacle demo |
| `quadrotor_planar` | 1 | `fixed_decay` | the myopic CBF-QP baseline |

Generation is slow. Cache it in CI keyed on the acados tag *and* a hash of `codegen/`
([14_CI.md §14.2](14_CI.md)) — but never on the OS alone, or you will test last week's solver
against this week's formulation and the symptom will look like a parity bug.

## §5.7 `generate_tube_solver.py` — M7

Import from `generate_mpc_cbf_solver`; do not copy it. Exactly three differences:

1. Variables named `(z, v)` — nominal, not true.
2. Parameter vector extended by one tightening scalar per obstacle per stage (§5.3).
3. Barrier rows become

   ```
   distance:  h_j(z_k) − c_{j,k}                                    ≥ 0
   DCBF:      h_j(F(z_k,v_k)) − h_j(z_k) + γ (h_j(z_k) − c_{j,k})   ≥ 0
   ```

   The tightening enters **both** rows. Read [08_TUBE.md §8.4](08_TUBE.md) before touching the
   sign.

Bounds arrive **already tightened** from the caller. This script MUST NOT compute `X ⊖ Ω` or
`U ⊖ KΩ` — the C++ `initialize()` is the single source of truth for those, and computing them
twice from two implementations is how they end up disagreeing.

Solver names are prefixed `tube_mpc_cbf_`.

### `compute_offline_sets()` — the cross-check

A NumPy/SciPy reference for the LQR gain and the RPI set. Its only job is to check the C++:
`RpiMatchesPythonReference` ([10_TESTS.md §10.2](10_TESTS.md)) asserts the two agree on `Ω`'s
support function in 32 directions to 1e-6.

Use `scipy.linalg.solve_discrete_are` here, deliberately, so the two implementations are
genuinely independent — the C++ side iterates the Riccati recursion
([07_SETS.md §7.5](07_SETS.md)). Two implementations of the same algorithm agreeing proves much
less than two implementations of different algorithms agreeing.

Returns `(K, Omega_vertices, alpha, s)`.

## §5.8 CLI contract

Both scripts:

```
--model {double_integrator_2d,unicycle_2d,bicycle_kinematic,quadrotor_planar}
--horizon N          --dt DT            --n-obstacles K
--variant {fixed_decay,relaxed_decay,distance_only}      (nominal only)
--tighten-mode {support_function,lipschitz,none}         (tube only)
--rti                --output-dir DIR   --all
```

`--all` ignores the per-configuration flags and emits §5.6. Print each generated solver name to
stdout — CMake and the CI log are how anyone later reconstructs what was actually built.
