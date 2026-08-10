# §10 · Tests

**Governs:** everything under `mpc_cbf_unified/test/`
**Milestone:** with each milestone — never after
**Done when:** A2–A5 and A8 are green in a Release build.

Rule 5 in [00_RULES.md](00_RULES.md): every non-obvious claim lands with the test that proves it,
**in the same change**. In this repository that is not process hygiene — it is the only thing
standing between a sign error and a controller that drives into obstacles while reporting success.

---

## Ground rules

**Fixed RNG seeds everywhere, printed at the top of the test.** A randomised test that cannot be
replayed is not a test; it is a lottery that occasionally reports a bug you cannot reproduce.

**Release builds for anything timing-related.** A Debug solve time is meaningless.

**Assert on the assembled constraint values, not only on behaviour.** Pull `cbf_values` and
`cbf_slack` out of the diagnostics and check the numbers. End-to-end tests pass with inverted
signs far more often than intuition suggests, because a wrong-signed controller still produces
plausible motion until the geometry gets tight.

**A test that cannot pass yet uses `GTEST_SKIP()` with a reason.** Never delete a failing test,
and never loosen a tolerance to make one pass — rule 4 in §0 and the safety-tolerance note there.

**Every scenario constant lives in the fixture, named.** A `0.2` buried in three tests is three
different obstacles the day someone changes one.

---

## §10.1 `test_mpc_cbf_feasibility.cpp` — M3, extended at M4

Fixture: `double_integrator_2d`, `N = 8`, `dt = 0.1`, start `(0,0)`, goal `(1,1)`, one static
obstacle of radius `0.2` at `(0.5, 0.5)`, `u ∈ [−1,1]²`, `γ = 0.3`. This is the ACC 2021 2-D
example, and the same numbers appear in the reproduction notebook — keep them identical.

### Configuration validation — M3

| Test | Assertion |
| --- | --- |
| `RejectsGammaOutsideUnitInterval` | `γ ∈ {0.0, −0.1, 1.5}` all make `initialize()` false; `setGamma()` rejects them **without mutating the config** |
| `RejectsUnsafeOmegaBound` | `kRelaxedDecay` with `ω_max·γ > 1` rejected; and `ω_max·γ == 1.0` exactly is **accepted** ([§6.3](06_SOLVER.md)) |
| `RejectsMismatchedWeightDimensions` | `Q` of length `nx−1` → false |

The second half of `RejectsUnsafeOmegaBound` is the one that matters: the shipped default sits on
the boundary, so a strict comparison would reject the repository's own configuration.

### Barrier — M3

| Test | Assertion |
| --- | --- |
| `BarrierValueSignConvention` | `> 0` strictly outside the inflated obstacle, `= 0` on the boundary to 1e-9, `< 0` inside |
| `BarrierMatchesGeneratedCode` | 100 random states: `|barrierValue(...) − cbf_values[stage 0]| < 1e-9` |

`BarrierMatchesGeneratedCode` is the guard against rule 4's failure mode. Without it, the C++
helper and the CasADi expression drift and every diagnostic in the repository becomes wrong while
everything keeps running.

### Safety and feasibility — M3, plus M4 for the last two

| Test | Assertion | Criterion |
| --- | --- | --- |
| `SolvesFromSafeInitialState` | `kSuccess`; `x_pred` has `N+1` columns; `u0 ∈ [u_min, u_max]`; all finite | |
| `DcbfConstraintHoldsOverHorizon` | `h(x_{k+1}) − h(x_k) ≥ −γ h(x_k) − 1e-6` for `k < N_CBF`, on **every** solve of the rollout | **A3** |
| `ClosedLoopStaysSafeForFullRollout` | 200 steps applying `u0` to the true dynamics: `min h ≥ −1e-6`, goal reached within 0.05 m | **A4** |
| `SmallerGammaAvoidsEarlier` | `γ = 0.1` gives larger minimum clearance and longer path than `γ = 0.9`; assert the **ordering**, not absolute values | |
| `DistanceOnlyBaselineFailsWhereCbfSucceeds` | at `N = 3`, `kDistanceOnly` reports infeasible or `min h < 0`; `kFixedDecay` stays safe | **A5** |
| `RelaxedDecayRecoversFeasibilityAtTightGamma` | a state where `kFixedDecay` returns `kInfeasible` and `kRelaxedDecay` returns `kSuccess` with `min h ≥ 0` in the prediction | |

**On `DistanceOnlyBaselineFailsWhereCbfSucceeds` (A5).** This asymmetry is the reason the
repository exists. Two failure modes to avoid:

- If MPC-DC also succeeds, the scenario is too easy. Shorten the horizon or move the obstacle
  closer until the separation appears, then **record the parameters that produced it** in the test
  comment and in `REPRODUCTION_REPORT.md`. A separation you had to search for is still a result;
  a separation you did not record is not reproducible.
- Do not handicap MPC-DC to manufacture the gap. Same cost, same bounds, same horizon; the only
  difference is the missing decay row ([§5.5](05_CODEGEN.md)).

**On `ClosedLoopStaysSafeForFullRollout` (A4).** If it fails at `dt = 0.1` but passes at
`dt = 0.01`, the implementation is right and the rate is the problem — document the required rate
([16_CONVENTIONS.md §16.5](16_CONVENTIONS.md)), do not relax the tolerance. Record `min h` and the
step count; `REPRODUCTION_REPORT.md` quotes them.

### Diagnostics — M3

| Test | Assertion |
| --- | --- |
| `ReportsFirstActiveConstraint` | driving straight at the obstacle: `first_active_cbf_step ≥ 0`, `first_active_obstacle == 0`, and the reported step matches the smallest entry of `cbf_slack` |
| `InfeasibilityReasonIsPopulated` | start inside the obstacle → `kInfeasible`, reason non-empty and naming the obstacle index |
| `RejectsNonFiniteState` | NaN in `x0` → `kNanDetected`, no crash, **no acados call** |

### Runtime contract — M3

| Test | Assertion | Criterion |
| --- | --- | --- |
| `HandlesMoreObstaclesThanSlots` | `kMaxObstacles + 4` obstacles: the nearest 8 appear in `cbf_values`, solve still succeeds | |
| `WarmStartReducesIterations` | second solve from the same state uses no more SQP iterations than the first | |
| `SolveMeetsRealTimeBudget` | 100 solves, `use_rti = true`, p95 `solve_time_ms < 10` | **A2** |
| `SolveDoesNotAllocate` | zero allocations across 10 solves after one warm-up | |

`SolveDoesNotAllocate`: override global `operator new`/`delete` with a counter guarded by a
`thread_local` flag. It is the only thing making the header's real-time claim true rather than
aspirational.

`SolveMeetsRealTimeBudget`: shared CI runners do not give reproducible timings. Assert a generous
ceiling in CI to catch a catastrophic regression, and take the real p95 from a local Release run
with the CPU named ([14_CI.md §14.4](14_CI.md)). The local number goes in the README; the CI
number does not.

## §10.2 `test_tube_mpc_robustness.cpp` — M6 (sets) and M7 (tube)

Implement in three layers, in this order. The primitives are cheap and everything above inherits
their errors.

### Set primitives — M6

| Test | Assertion |
| --- | --- |
| `BoxSupportFunctionIsExact` | unit box: `support(d) == ‖d‖₁` for 20 random `d` |
| `MinkowskiSumOfBoxesIsABox` | `box(a) ⊕ box(b)` has the vertices of `box(a+b)` |
| `PontryaginDifferenceIsInverseOfSumForBoxes` | `(box(a) ⊕ box(b)) ⊖ box(b) == box(a)` to 1e-9 |
| `PontryaginDifferenceCanBeEmpty` | `box(0.1) ⊖ box(0.5)` reports `isEmpty()` |
| `RedundancyRemovalPreservesTheSet` | duplicate every half-space, remove, sample 1000 points, containment unchanged |
| `SupportFunctionMatchesSampledMaximum` | zonotope closed form ≥ 10 000 samples and within 1e-6 |
| `OrderReductionOverApproximates` | `reduceOrder(n+2)` contains every sample of the original |

`PontryaginDifferenceCanBeEmpty` is what the tube relies on to fail loudly when `W` exceeds the
actuator authority ([08_TUBE.md §8.3](08_TUBE.md)).

`OrderReductionOverApproximates` guards the one place an under-approximation would be silently
unsafe ([07_SETS.md §7.2](07_SETS.md)).

### RPI — M6

| Test | Assertion |
| --- | --- |
| `ConvergesForSchurStableSystem` | `A_cl = 0.5·I`, `W = box(0.1)`: converged, `α < 1`, result contains the analytic `box(0.2)` within `epsilon` |
| `RejectsUnstableClosedLoop` | spectral radius 1.1 → `converged == false`, no hang |
| `ResultIsRobustPositivelyInvariant` | the support-function check passes **and** a 10 000-step random-disturbance simulation started inside `Ω` never leaves it |
| `LqrGainIsStabilising` | `spectralRadius(A + B·K) < 1` — pins the sign convention |
| `RpiMatchesPythonReference` | `Ω`'s support agrees with `compute_offline_sets()` in 32 directions to 1e-6 |

`ResultIsRobustPositivelyInvariant` runs both checks deliberately: the empirical simulation
catches sign errors that the support-function check can share with the implementation it is
checking.

### Tube solver — M7

| Test | Assertion | Criterion |
| --- | --- | --- |
| `InitializeRejectsOversizedDisturbance` | `W` large enough that `U ⊖ KΩ` is empty → `initialize()` false, reason logged | |
| `TighteningIsNonNegativeAndShrinksWithW` | `c ≥ 0` everywhere; halving `W`'s generators never increases it | |
| `SupportTighteningDominatesLipschitz` | support form `≤` Lipschitz form at every tested state | |
| `StaysSafeUnderWorstCaseDisturbance` | 200 steps, `w_k` the vertex of `W` maximising `−∇hᵀw`: `min h ≥ 0` | **A8** |
| `StaysSafeAcrossRandomDisturbanceSeeds` | 50 seeds × 200 steps of uniform samples: zero violations; report the clearance distribution | **A8** |
| `NominalCbfViolatesUnderSameDisturbance` | `kNone` under the same worst case: `min h < 0` | **A8** |
| `ErrorStaysInsideRpiSet` | `rpiSet().contains(x_k − z_k)` at every step | **A8** |
| `AncillaryInputRespectsBounds` | `u_applied` within bounds **and the clip counter stays zero** | |

**`NominalCbfViolatesUnderSameDisturbance` is what gives the other three meaning.** If the
untightened controller also stays safe under your `W`, the disturbance is too small to be
evidence of anything. Enlarge `W` until nominal fails, then keep that `W` for every test in this
group. Note the `allow_unsafe_ablation` guard in [08_TUBE.md §8.4](08_TUBE.md) — the test must set
it explicitly.

`ErrorStaysInsideRpiSet` validates the certificate directly rather than its consequence, which is
why it is worth having alongside the safety tests.

## §10.3 `test_recursive_feasibility.py` — M5

Specified in [11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md), because it is as much the Python
reference implementation as it is a test.

## §10.4 Registering a new test

Add an `ament_add_gtest` block ([03_BUILD_SYSTEM.md §3.8](03_BUILD_SYSTEM.md)). **Forgetting this
is silent** — the file compiles nowhere and CI stays green. Check the test count in the
`colcon test-result` output whenever you add one.
