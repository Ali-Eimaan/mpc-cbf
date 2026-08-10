# §6 · `MpcCbfSolver`

**Governs:** `include/mpc_cbf_unified/mpc_cbf_solver.hpp`, `src/mpc_cbf_solver.cpp`
**Milestone:** M3 (fixed decay), M4 (the other two variants)
**Done when:** A2, A3, A4 (M3) and A5, A6 (M4) are green.

The header is the contract. Do not change a declaration in it without the procedure in rule 2 of
[00_RULES.md](00_RULES.md).

---

## §6.1 Structure

Everything acados-specific lives in `MpcCbfSolver::Impl` inside the `.cpp`. The header must stay
compilable without acados on the include path — that is why the PIMPL exists; keep it that way.

`Impl` holds:

```cpp
MpcConfig mpc;  CbfConfig cbf;  bool initialized;
int nx, nu, np;                       // np per §5.3: 7*n_obs + 1
Eigen::MatrixXd x_ref;                // nx x (N+1)
Eigen::MatrixXd x_guess, u_guess;     // warm start
std::vector<double> parameter_buffer; // sized once; reused every solve
// acados capsule + nlp_config / dims / in / out pointers
```

Guard all acados code with `#if MPC_CBF_WITH_ACADOS`. In the `#else` branch, `initialize()`
returns `false` and `solve()` returns `kNotInitialized`. **Never fake a solution**
([03_BUILD_SYSTEM.md §3.2](03_BUILD_SYSTEM.md)).

## §6.2 Dimensions

`stateDimOf`: 4 / 3 / 4 / 6. `inputDimOf`: 2 for all four ([§4.2](04_MODELS.md)).

Under `kRelaxedDecay` the generated input dimension is `nu + n_obs`, but `inputDim()` and
`MpcCbfSolution::u0` expose only the physical `nu`. The `ω` values go in
`SolverDiagnostics::omega`, never in `u0`. A caller must not need to know which variant is
running to interpret the control it is handed.

## §6.3 `initialize()` — validation list

Reject (return `false`, log the offending field **and its value** to stderr) when any of:

```
gamma ∉ (0, 1]
horizon < 1                                   dt <= 0
Q.size() != nx | R.size() != nu | Qf.size() != nx
x_min.size() != nx | x_max.size() != nx | u_min.size() != nu | u_max.size() != nu
any(u_min > u_max) | any(x_min > x_max)
any non-finite entry in any bound or weight
max_sqp_iterations < 1                        kkt_tolerance <= 0
variant == kRelaxedDecay and (omega_min < 0
                              or omega_min > omega_max
                              or omega_max * gamma > 1 + 1e-9)
```

**The `1e-9` in the last line is load-bearing.** At the YAML defaults (`gamma 0.3`,
`omega_max 3.0`) the product is exactly 1.0 — the boundary case the theory permits. A strict
`> 1` comparison rejects the shipped configuration and sends you looking for a bug that is not
there.

Then:

- resolve `cbf_horizon`: `≤ 0` means `horizon`; clamp to `[1, horizon]`;
- allocate the acados capsule for the configured model and variant, set solver options, push cost
  matrices and bounds;
- size `x_ref`, `x_guess`, `u_guess` and `parameter_buffer` **once**.

**After `initialize()` returns, `solve()` must not allocate.** `SolveDoesNotAllocate`
([10_TESTS.md §10.1](10_TESTS.md)) enforces it, and A2 is a tail measurement, which is exactly
what an occasional allocation shows up in.

## §6.4 Obstacle handling

`kMaxObstacles = 8` is compiled into the generated solver, so `solve()` must reduce whatever it
is given to exactly that many slots:

1. Sort by `‖p(x0) − o_j‖`, keep the nearest 8.
2. Pad the remaining slots with `{position = (1e6, 1e6, 1e6), radius = 0, is_dynamic = false}`.
3. Propagate to each stage: `o_j(k) = o_j(0) + k·dt·v_j` when dynamic.
4. Inflate: the radius pushed is `r_obs + ego_radius + safety_margin`, applied **here and only
   here** ([§5.3](05_CODEGEN.md)).

Distance sorting is adequate for the demos and **wrong in general** — a fast obstacle approaching
from outside the nearest eight will be dropped. Implement the sort, and record that limitation in
a one-line comment and in the README's Limitations section. Do not present it as optimal.

## §6.5 `solve()` — step by step

```
 1  guards      !initialized                        -> kNotInitialized
                x0.size() != nx || !x0.allFinite()  -> kNanDetected, reason set, no acados call
 2  obstacles   §6.4
 3  parameters  build each stage's vector into parameter_buffer (§5.3) and push it
                [verify the call for your version: ocp_nlp_in_set(..., k, "p", buf) or
                 <name>_acados_update_params(capsule, k, p, np) — risk V11]
 4  reference   "yref" per stage (state block from x_ref, input block zero), "yref_e" at N
 5  initial     lbx_0 = ubx_0 = x0
 6  warm start  push x_guess / u_guess when present
 7  solve       steady_clock around <name>_acados_solve(capsule)
 8  read back   x_pred (nx x N+1), u_pred (nu x N)
 9  diagnostics §6.7
10  status      §6.5.1
11  warm start  store x_pred/u_pred shifted one stage (last column duplicated) for next time
```

### §6.5.1 Status mapping — verify this, do not trust it

The mapping the skeleton assumes:

| acados return | `SolverStatus` |
| --- | --- |
| 0 | `kSuccess` |
| 1 | `kNanDetected` |
| 2 | `kMaxIterations` |
| 3 | `kQpFailure` |
| 4 | `kInfeasible` |
| other | `kQpFailure` |

**This is UNVERIFIED (risk V12).** Read `acados_solver_common.h` in your installed version, put
the verified mapping in a comment next to the switch, and update this table. A wrong mapping that
turns an infeasible solve into `kSuccess` is the one bug in this repository that reaches the
plant, and no test above this layer can catch it — the solver would be reporting success and the
constraint values would look fine, because the iterate that violated them is the one you are
reading back.

`MpcCbfSolution::usable()`: `true` for `kSuccess`; `true` for `kMaxIterations` **only** when
`u0` and `x_pred` are finite; `false` otherwise. That single exception is the whole of the
"never make an infeasible solve look feasible" rule's leeway — document it where it is
implemented.

## §6.6 `barrierValue()`

```cpp
// h(x) = ||p(x) - p_obs||^2 - (r_obs + inflation_radius)^2
```

Mirrors `barrier_expression()` ([§4.4](04_MODELS.md)) exactly, using the position indices from
§4.2 (`(px, pz)` for the planar quadrotor). **No square roots.** Mixing the squared and unsquared
forms makes every diagnostic wrong by a nonlinear factor while everything continues to run.

## §6.7 Diagnostics — fill on every solve, success or not

| Field | Content |
| --- | --- |
| `cbf_values[k·n_obs + j]` | `barrierValue(model, x_pred.col(k), obstacle_j_at_stage_k, inflation)` |
| `cbf_slack` | DCBF row residual `h_{k+1} − h_k + γ h_k`; `≥ 0` means satisfied |
| `omega` | tail of the generated input vector under `kRelaxedDecay`; empty otherwise |
| `first_active_cbf_step` / `first_active_obstacle` | the `(k, j)` minimising `cbf_slack`, reported only when that minimum is below `kActiveTolerance = 1e-3`; `-1` otherwise |
| `solve_time_ms` | `steady_clock` around the solve call only, not the parameter push |
| `sqp_iterations` | `ocp_nlp_get(..., "sqp_iter")` |
| `kkt_residual` | `ocp_nlp_get(..., "residuals")`, max entry |
| `cost` | `ocp_nlp_get_cost_value` or the version's equivalent |

`kActiveTolerance` is a **diagnostic** threshold. It must never influence a constraint, a status,
or a control.

### `infeasibility_reason`

Build a specific string. "solver failed" costs the next reader an hour. Classify in this order:

```
non-finite state            -> "non-finite state: index <i>"
h_j(x0) < 0                 -> "initial state violates barrier for obstacle <j> (h = <v>)"
u bounds inconsistent       -> "input bounds infeasible: u_min[<i>] > u_max[<i>]"
QP infeasible               -> "QP infeasible; tightest DCBF row at stage <k>, obstacle <j>, slack <s>"
max iterations              -> "max SQP iterations (<n>), KKT residual <r>"
otherwise                   -> "acados returned <code>"
```

Only fill it when `status != kSuccess`. A reason string on a successful solve is noise that
trains readers to ignore the field.

## §6.8 The remaining methods

| Method | Contract |
| --- | --- |
| `setReference(x_ref)` | replicate across all `N+1` columns |
| `setReferenceTrajectory(traj)` | copy column-wise; repeat the last column if short; **reject and log** if `rows != nx`, do not silently resize |
| `setGamma(g)` | validate `(0,1]`, store; no regeneration — it is a runtime parameter (§5.1). Returns `false` on a bad value without mutating anything |
| `warmStart(xg, ug)` | shape-check against `(nx, N+1)` and `(nu, N)`; **ignore a mismatch silently** — a bad warm start must never break a solve |
| `reset()` | drop the warm start and re-initialise the acados iterate; keep the configuration |
| `toString(...)` | static string literals, never a pointer into a temporary |
| `parseModelType` / `parseCbfVariant` | the YAML spellings from [09_NODE.md §9.2](09_NODE.md), case-insensitive; leave the output untouched on failure |

The asymmetry in the two shape checks is deliberate: a wrong *reference* is a configuration error
the operator must see, while a wrong *warm start* is a hint that can be discarded without
consequence.

## §6.9 What this class must never do

- allocate in `solve()` (§6.3)
- return a control when the solve did not succeed (§6.5.1)
- inflate an obstacle radius twice (§6.4)
- log unthrottled (it has no rate limiter — the node throttles; keep solver logging to
  `initialize()` and genuine errors)
- know anything about ROS
