# §1 · What is being built

A discrete-time model predictive controller whose prediction horizon carries discrete-time
control barrier function constraints, plus a tube variant that keeps the guarantee under bounded
disturbance.

```
   x_ref ──►┌────────────────────────────────────┐  u0   ┌───────┐
            │  MPC-CBF                           │──────►│ plant │
 obstacles ►│  min Σ‖x−x_ref‖²_Q + ‖u‖²_R         │       └───┬───┘
            │  s.t. x⁺ = f(x,u), x∈X, u∈U        │           │ x
            │       h(x_k) ≥ 0                   │◄──────────┘
            │       Δh ≥ −γ h        (the CBF)   │
            └───────────────┬────────────────────┘
                            │ SolverDiagnostics
                            ▼  h per stage · active row · solve time · infeasibility reason
```

Three formulations share one solver class, selected by `CbfVariant`:

| Variant | Constraint | Use when |
| --- | --- | --- |
| `kFixedDecay` | `Δh ≥ −γ h` | the baseline — Zeng-Zhang-Sreenath ACC 2021 |
| `kRelaxedDecay` | `Δh ≥ −ω_k γ h`, `ω_k` free | feasibility matters more than a fixed decay rate — CDC 2021 |
| `kDistanceOnly` | `h ≥ 0` only | the MPC-DC baseline you are trying to beat |

and one robust extension, `TubeMpcCbfSolver`, which solves the nominal problem with every
constraint tightened by an offline RPI set.

## §1.1 Why this repository exists

It is a portfolio repository for a robotics PhD application, and the closest single-agent
precursor to the author's thesis topic. Its job is to let a reviewer conclude, in five minutes of
reading, that the author has implemented the foundational MPC-CBF papers from scratch,
understands the trade-off against a plain distance-constrained MPC, and knows precisely what the
formulation does and does not guarantee.

That drives most of the engineering decisions in these documents:

- **The correctness claim rests on reproduction, not on self-consistency.** Two published papers
  are reproduced figure by figure, and the report says which numbers matched and to what
  tolerance ([13_DOCS.md §13.5](13_DOCS.md)). "My controller passes my tests" persuades nobody
  who has written a controller.
- **A second implementation checks the first.** The Python path (§11) exists so that C++ and
  NumPy/acados must agree on `u0` to 1e-6 (A7). A single implementation agreeing with itself is
  not evidence.
- **Every number must be defensible.** A solve-time figure without the CPU named, a `γ` presented
  as tuned when it was guessed, a paper figure number quoted from memory — each costs more
  credibility than the missing result would have.
- **Stating limits is a strength here.** §1.6, the "what this does not prove" section of
  `docs/README_math.md`, and the README's Limitations section are load-bearing. A reviewer who
  finds a limitation you did not list stops trusting the ones you did.

Downstream, `MpcCbfSolver` is the per-agent inner problem of the author's
`transition-viable-swarm`, and the reproductions establish the baselines that work will cite.
Keep the solver interface stable and free of ROS dependencies.

## §1.2 Deliverables beyond the code

These are part of "done", not extras:

| Deliverable | Produced by | Notes |
| --- | --- | --- |
| `media/obstacle_avoidance.gif` | `reproduction/zeng_acc2021/reproduce_acc2021.ipynb` | trajectory + `h(t)` inset, three values of `γ` |
| `media/cbfqp_vs_mpccbf_side_by_side.gif` | `analysis/cbfqp_vs_mpccbf_comparison.ipynb` | myopic filter colliding, MPC-CBF not, same seed |
| `media/tube_robustness.gif` | `analysis/disturbance_robustness_sweep.ipynb` | nominal clipping the obstacle under wind, tube holding |
| `reproduction/REPRODUCTION_REPORT.md` | both reproduction notebooks | no `TBD` may remain |
| README results table | `analysis/` + the test suite | every row names its hardware and git SHA |
| **One documented case where MPC-CBF loses** | `analysis/cbfqp_vs_mpccbf_comparison.ipynb` | see §12.3 — deliberate, not an oversight |

## §1.3 Acceptance criteria

The whole project is done when all nine hold.

| # | Criterion | Verified by |
| --- | --- | --- |
| A1 | Clean-container build succeeds, Release, warnings clean, linters clean | `colcon_build.yml` ([§14.1](14_CI.md)) |
| A2 | p95 solve time < 10 ms — RTI, `N = 8`, 8 obstacles, i.e. 10 % of a 10 Hz period | `SolveMeetsRealTimeBudget` ([§10.1](10_TESTS.md)) |
| A3 | The DCBF condition holds along every returned prediction to 1e-6 | `DcbfConstraintHoldsOverHorizon` ([§10.1](10_TESTS.md)) |
| A4 | Closed-loop forward invariance: `min h ≥ −1e-6` over 100 rollouts | `ClosedLoopStaysSafeForFullRollout` ([§10.1](10_TESTS.md)) |
| A5 | The separation result: MPC-DC fails at short horizon where MPC-CBF succeeds, with the parameters recorded | `DistanceOnlyBaselineFailsWhereCbfSucceeds` ([§10.1](10_TESTS.md)) |
| A6 | Relaxed decay recovers ≥ 90 % of fixed-decay infeasible states, with `max(ω·γ) ≤ 1 + 1e-9` | `test_recursive_feasibility.py` ([§11.4](11_PYTHON_REFERENCE.md)) |
| A7 | C++ and Python agree on `u0` to 1e-6 over 50 states | `test_python_and_cpp_agree_on_first_input` ([§11.5](11_PYTHON_REFERENCE.md)) |
| A8 | Tube: `min h ≥ 0` under worst-case disturbance, error stays in `Ω`, **and the untightened ablation violates under the same `W`** | `test_tube_mpc_robustness.cpp` ([§10.2](10_TESTS.md)) |
| A9 | Both reproduction notebooks execute in CI and assert their own numbers; `REPRODUCTION_REPORT.md` has no `TBD` | `colcon_build.yml` ([§14.5](14_CI.md)) |

**If a criterion cannot be met, change the criterion in this file with a written reason.**
Do not weaken a test in place — a threshold quietly relaxed to make CI green is a lie told to
every future reader.

A2 is the one most likely to be noisy: shared CI runners do not give reproducible timings. Make
the CI assertion generous and record the real measurement, with the CPU named, from a local
Release run.

A8's second half is the one people forget. A tube that is never shown to be *necessary* proves
nothing — if the untightened controller also stays safe under your `W`, the disturbance is too
small to be evidence, and the fix is a larger `W`, not a happier conclusion.

## §1.4 Design decisions already made

These are settled. They are recorded here so they are not re-litigated mid-implementation; each
is justified in its own document.

| Decision | Value | Where |
| --- | --- | --- |
| Barrier sign | `h > 0` **inside** the safe set | [16_CONVENTIONS.md §16.1](16_CONVENTIONS.md) |
| Barrier form | squared distance `‖p−p_obs‖² − r_eff²`, not the norm | [04_MODELS.md §4.4](04_MODELS.md) |
| CBF condition | `Δh ≥ −γ h`, linear class-K, `γ ∈ (0,1]` | [04_MODELS.md §4.5](04_MODELS.md) |
| Both barrier rows emitted | `h ≥ 0` **and** the decay row | [05_CODEGEN.md §5.4](05_CODEGEN.md) |
| Relaxation | `ω` per stage and obstacle, penalty on `(ω−1)²` | [05_CODEGEN.md §5.5](05_CODEGEN.md) |
| Integrator | acados `DISCRETE`, never `ERK` | [05_CODEGEN.md §5.2](05_CODEGEN.md) |
| Solver backend | acados + HPIPM, behind a PIMPL | [06_SOLVER.md §6.1](06_SOLVER.md) |
| Obstacle capacity | `kMaxObstacles = 8`, distance-pruned | [06_SOLVER.md §6.4](06_SOLVER.md) |
| Obstacle prediction | constant velocity over the horizon | [16_CONVENTIONS.md §16.3](16_CONVENTIONS.md) |
| Tube tightening | support-function form, Lipschitz as fallback | [08_TUBE.md §8.4](08_TUBE.md) |
| RPI set | Raković et al. outer approximation, zonotope iteration | [07_SETS.md §7.4](07_SETS.md) |
| Node type | plain `rclcpp::Node` with a timer, not lifecycle | [09_NODE.md §9.1](09_NODE.md) |
| Behaviour without acados | `initialize()` returns false; **never a silent fallback** | [03_BUILD_SYSTEM.md §3.2](03_BUILD_SYSTEM.md) |
| Licence | BSD-3-Clause (matches `LICENSE` and the sibling repository) | [02_ENVIRONMENT.md §2.6](02_ENVIRONMENT.md) |

## §1.5 What this skeleton adds beyond the original spec

[INFO.md](INFO.md) fixes the target structure. The skeleton implements all of it, plus the files
below, which exist because the specified files cannot be written without them. They are listed so
the deviation is explicit rather than discovered.

| Addition | Why |
| --- | --- |
| `codegen/models.py` | both generators, the pytest suite and every notebook must agree bit-for-bit on the dynamics and on `h`; one definition, four consumers |
| `codegen/requirements.txt` | the notebooks are a CI deliverable, so their dependencies must be pinned |
| `mpc_cbf_unified/scripts/` (5 rclpy nodes) | the four specified launch files have no plant without them ([09_NODE.md §9.6](09_NODE.md)) |
| `mpc_cbf_unified/test/cpp_solve_cli.cpp` | A7 needs a way to call the C++ solver from Python ([11_PYTHON_REFERENCE.md §11.5](11_PYTHON_REFERENCE.md)) |
| `media/README.md` | provenance for generated artefacts |
| `.gitignore` | keeps `c_generated_code/`, `build/`, executed notebooks out of the tree |

`INFO.md` lists `test_recursive_feasibility.py` under `mpc_cbf_unified/test/`, which is where the
skeleton keeps it, even though it is a pytest file rather than a gtest — moving it would diverge
from the author's specification for no gain.

Nothing specified in `INFO.md` was dropped or renamed.

## §1.6 What this repository does not prove

Load-bearing. Repeat these, in these words, wherever a guarantee is discussed.

- **Recursive feasibility is not guaranteed.** DCBF constraints inside an MPC do not confer it.
  Neither of the standard sufficient conditions — a terminal control-invariant set inside the
  safe set, or a horizon long enough to reach one — is implemented here. Feasibility at time `t`
  does not imply feasibility at `t+1`. The Python study measures the feasible set; the recovery
  strategies in [09_NODE.md §9.4](09_NODE.md) are mitigations, not proofs.
- **Safety is conditional on per-step feasibility.** Given `x_0` safe and a feasible solve at
  every step, `h(x_t) ≥ 0` for all `t`. That hypothesis is exactly the thing not guaranteed
  above; say both sentences together or neither.
- **The RPI certificate is exact only for the linear model.** For the nonlinear plants `Ω` is
  computed at a linearisation point and is a local certificate. State where.
- **Obstacle prediction error is not in `W`** unless deliberately added.
- **State-estimation error is not modelled at all.**
- **Nothing here addresses multi-agent coupling.** That is the thesis, not this repository.
