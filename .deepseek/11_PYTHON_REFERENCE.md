# §11 · The Python path and the parity check

**Governs:** `mpc_cbf_unified/test/test_recursive_feasibility.py`,
`mpc_cbf_unified/test/cpp_solve_cli.cpp`, and the shared helpers in `codegen/`
**Milestone:** M5
**Done when:** A6 and A7 are green.

---

## §11.1 Why this exists

Two reasons, and the second is the important one.

1. **Sweeps belong in Python.** The feasibility study runs hundreds of solves over a grid of
   initial states and a sweep of `γ` and `N`. That is a job for NumPy and the acados Python
   interface, not for gtest.
2. **A single implementation agreeing with itself is not evidence.** A7 requires the C++ solver
   and the Python solver to produce the same `u0` to 1e-6. Without it, the notebooks and the
   deployed controller can diverge silently, and every plot in `analysis/` becomes a picture of
   something other than the shipped code.

The Python path is *not* an independent re-derivation of the OCP — both sides call acados. What it
independently exercises is the parameter packing, the reference handling, the obstacle
propagation and the barrier definition, which is where the bugs actually are.

The one genuinely independent implementation in the repository is
`compute_offline_sets()` ([05_CODEGEN.md §5.7](05_CODEGEN.md)), which uses SciPy's Riccati solver
against the C++ recursion. Keep it that way.

## §11.2 Module contract

`test_recursive_feasibility.py` imports the shared definitions rather than redefining them:

```python
from codegen.models import double_integrator_2d, unicycle_2d
from codegen.generate_mpc_cbf_solver import build_ocp
from acados_template import AcadosOcpSolver
```

**Skip the whole module cleanly when acados is unavailable** —
`pytest.skip(..., allow_module_level=True)` — so the suite stays runnable on a laptop without it.
A hard import error here turns "acados not installed" into "the test suite is broken", and the
two get treated very differently.

Fixtures are module-scoped: building an acados solver per test would make the sweep unusably slow.

| Fixture | Contents |
| --- | --- |
| `solver` | the 2-D scenario from [10_TESTS.md §10.1](10_TESTS.md): `N=8`, `dt=0.1`, `γ=0.3`, obstacle `r=0.2` at `(0.5,0.5)` |
| `feasible_start_grid` | 20×20 grid over `[−0.5, 1.5]²`, obstacle interior removed |

Keep the scenario numerically identical to the gtest fixture and the ACC notebook. Three copies
of "almost the same scenario" produce three sets of numbers that cannot be compared, which
defeats the purpose of having all three.

## §11.3 The feasibility study

| Test | Assertion |
| --- | --- |
| `test_feasible_set_is_nonempty` | ≥ 80 % of the grid admits a feasible first solve; on failure, report the **status histogram**, not just the fraction |
| `test_persistent_feasibility_along_closed_loop` | for every start feasible at `k=0`, 100 closed-loop steps; count and **print** the states that later went infeasible |
| `test_feasible_fraction_decreases_with_gamma` | parametrised over `γ ∈ {0.1,…,1.0}`; monotone shrinkage within noise; dump the curve to `results/feasibility_vs_gamma.csv` |
| `test_longer_horizon_enlarges_feasible_set` | parametrised over `N ∈ {1,3,5,8,15}`; feasible fraction non-decreasing in `N` |

**`test_persistent_feasibility_along_closed_loop` must print its failures, not merely assert.**
DCBF constraints do not confer recursive feasibility ([01_OVERVIEW.md §1.6](01_OVERVIEW.md)), so a
nonzero count is the *scientific result*, and [12_ANALYSIS.md §12.4](12_ANALYSIS.md) consumes the
list. Calibrate the assertion threshold from the measured value and record it in
`REPRODUCTION_REPORT.md` rather than assuming a number.

`test_longer_horizon_enlarges_feasible_set` at `N = 1` is the quantitative version of the
CBF-QP-versus-MPC-CBF argument. It is the only place that claim is made with numbers rather than
a GIF.

## §11.4 `test_infeasibility_is_recoverable_with_relaxed_decay` — A6

Collect the fixed-decay failures from §11.3, rebuild the solver with the CDC 2021 `ω`
formulation, and retry each.

| Assertion | Value |
| --- | --- |
| recovery rate | ≥ 90 % |
| `max(ω_k · γ)` over every recovered solve | ≤ 1 + 1e-9 |

The second is not optional. A recovery rate achieved by violating `ω γ ≤ 1` is not a recovery —
it is the safety condition being switched off, and it would look like a success in every other
metric ([04_MODELS.md §4.5](04_MODELS.md)).

If the recovery rate falls short, report the measured value and the states that resisted
recovery. Do not lower the threshold in place (rule 4, §0); change it in
[01_OVERVIEW.md §1.3](01_OVERVIEW.md) with a reason.

## §11.5 `test_python_and_cpp_agree_on_first_input` — A7

The parity check. 50 states drawn from the grid; assert `‖u0_py − u0_cpp‖_∞ < 1e-6`.

### The harness

`mpc_cbf_unified/test/cpp_solve_cli.cpp` — a test-only executable
([03_BUILD_SYSTEM.md §3.8](03_BUILD_SYSTEM.md)):

```
stdin :  {"x0": [...], "obstacles": [{"position": [...], "velocity": [...], "radius": r,
                                      "is_dynamic": false}, ...],
          "x_ref": [...]}
stdout:  {"status": "SUCCESS", "u0": [...], "solve_time_ms": 1.23,
          "cbf_values": [...], "first_active_cbf_step": -1}
```

One JSON object per line, one solve per line, so the 50 cases run in a single process. Locate the
binary through an environment variable set by `ament_add_pytest_test`, and skip with a clear
message when it is absent rather than failing.

### When parity fails

The disagreement is almost never in the OCP — both sides call the same generated solver. Check,
in this order:

1. **The barrier.** `barrier_expression` vs `barrierValue` ([04_MODELS.md §4.4](04_MODELS.md)).
2. **The parameter vector.** Layout, ordering, and whether the radius was inflated on one side
   only ([05_CODEGEN.md §5.3](05_CODEGEN.md)).
3. **Obstacle propagation.** Whether both sides advanced the obstacle by `k·dt·v`.
4. **The reference.** Whether the input block of `yref` is zero on both sides.
5. **Warm start.** A warm start must change the iteration count, never the answer beyond solver
   tolerance ([16_CONVENTIONS.md §16.4](16_CONVENTIONS.md)). If it changes the answer, the problem
   is degenerate — investigate before accepting.

A failure here is worth more than the time it costs. It is the check that makes every number in
`analysis/` and `reproduction/` describe the shipped controller rather than a Python sketch of it.

## §11.6 CI

A7 must be verified **by CI**, not merely locally ([15_ROADMAP.md §17](15_ROADMAP.md)). That means
the parity test runs in the job where `cpp_solve_cli` exists — the `colcon` job — not in the
standalone python job ([14_CI.md §14.5](14_CI.md)). Until that is wired up, say so; do not let a
green badge imply A7 holds.
