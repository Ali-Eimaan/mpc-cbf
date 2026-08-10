# §4 · Models, dynamics and the barrier

**Governs:** `codegen/models.py`
**Milestone:** M1 — the first thing you implement
**Done when:** RK4 matches analytic solutions to 1e-9; the barrier and its gradient match finite
differences to 1e-6; `test_models.py` green.

This is the smallest document in the set and the one with the widest blast radius. Every other
subsystem consumes it: both generators, the C++ barrier helper, the pytest suite, and all five
notebooks. **A bug here is invisible everywhere and wrong everywhere.**

---

## §4.1 Why this file exists at all

It is not in `INFO.md`. It exists because the alternative is four independent definitions of the
same dynamics — one per consumer — which drift within a week and produce a repository whose
notebook plots do not describe its controller.

One definition, four consumers. Nothing else may define dynamics or `h`.

## §4.2 The four models

| Registry key | `ModelType` (C++) | nx | state | nu | input | position indices |
| --- | --- | --- | --- | --- | --- | --- |
| `double_integrator_2d` | `kDoubleIntegrator2D` | 4 | `[px, py, vx, vy]` | 2 | `[ax, ay]` | `(0, 1)` |
| `unicycle_2d` | `kUnicycle2D` | 3 | `[px, py, θ]` | 2 | `[v, ω]` | `(0, 1)` |
| `bicycle_kinematic` | `kBicycleKinematic` | 4 | `[px, py, θ, v]` | 2 | `[a, δ]` | `(0, 1)` |
| `quadrotor_planar` | `kQuadrotorPlanar` | 6 | `[px, pz, vx, vz, φ, φ̇]` | 2 | `[T, τ]` | `(0, 1)` |

`quadrotor_planar` lives in the **vertical** plane: index 1 is `pz`, not `py`. This is the single
most common source of confusion in the repository ([16_CONVENTIONS.md §16.2](16_CONVENTIONS.md)).

`MpcCbfSolver::stateDimOf` and `inputDimOf` mirror this table and MUST agree with it. `inputDim`
is 2 for all four models; under `kRelaxedDecay` the *generated* input vector is longer, but that
is a codegen detail and never surfaces in `inputDim()` ([06_SOLVER.md §6.2](06_SOLVER.md)).

## §4.3 Continuous dynamics and discretisation

```
double_integrator_2d:  ṗ = v,                        v̇ = u
unicycle_2d:           ṗ = [v cos θ, v sin θ],       θ̇ = ω
bicycle_kinematic:     ṗ = [v cos θ, v sin θ],       θ̇ = v tan(δ)/L,   v̇ = a       (L = 0.35 m)
quadrotor_planar:      ṗ = [vx, vz],
                       v̇ = [−T sin φ / m, T cos φ / m − g],           φ̈ = τ / I
```

`discretise(spec, dt, method)`:

| `method` | Use |
| --- | --- |
| `"rk4"` | default for every nonlinear model; explicit RK4, one step per `dt` |
| `"exact"` | `double_integrator_2d` only — the exact ZOH matrices below |
| `"euler"` | exists **only** so a notebook can demonstrate why it is inadequate at `dt = 0.1`. Never a default. |

For the double integrator, return the exact ZOH form:

```
A = [[I, dt·I], [0, I]]          B = [[dt²/2·I], [dt·I]]
```

This matters beyond tidiness: the tube's RPI set is computed from `(A, B)`, and using an
RK4-approximate `A` there would make the "exact for the linear model" claim in
[01_OVERVIEW.md §1.6](01_OVERVIEW.md) false.

`linearised_discrete(spec, dt, x_op, u_op)` returns `(A, B)` for the tube path. For the double
integrator it ignores the operating point and returns the exact matrices; for the others it
linearises about `(x_op, u_op)` and the caller must record that the certificate is local
([08_TUBE.md §8.3](08_TUBE.md)).

**Do not silently substitute Euler for RK4** in any path a notebook plots. If integration cost
becomes a problem, say so and change it deliberately.

## §4.4 The barrier — the single definition

```
h_j(x) = ‖P x − p_j‖² − r_eff²,        r_eff = r_obs + ego_radius + safety_margin
```

with `P` the position selector from §4.2. Positive means safe. **Squared distance, never the
norm** — the squared form is smooth at `p = p_j`, quadratic in `x`, and gives an affine constraint
Jacobian, which is what makes the Gauss-Newton Hessian exact for the barrier row.

`barrier_expression(spec, x, obstacle_params)` builds this as a CasADi expression from the
symbolic parameter slice `[ox, oy, oz, vx, vy, vz, radius]` ([05_CODEGEN.md §5.3](05_CODEGEN.md)).
The inflation (`ego_radius + safety_margin`) is folded into the radius **by the caller** before
the parameter is pushed, so the expression sees one effective radius. Document which side does
the inflation in both places; doing it twice is a real bug and produces a controller that is
mysteriously conservative.

The gradient, needed by the tube:

```
∇h_j(x) = 2 Pᵀ (P x − p_j)
```

**`MpcCbfSolver::barrierValue()` in C++ must produce identical values to 1e-9**
([06_SOLVER.md §6.6](06_SOLVER.md)), and `BarrierMatchesGeneratedCode`
([10_TESTS.md §10.1](10_TESTS.md)) enforces it. This is rule 4 in [00_RULES.md](00_RULES.md).

## §4.5 The DCBF condition — the other single definition

```python
def dcbf_constraint(h_next, h_now, gamma, omega=None):
    return h_next - h_now + (omega if omega is not None else 1.0) * gamma * h_now
```

constrained `≥ 0` by the caller. That is the whole of it, and both generators plus the notebooks
import this one function rather than re-typing the inequality.

Why this exact form:

- `h_next − h_now ≥ −γ h_now` rearranges to `h_next ≥ (1 − γ) h_now`, so `h_now ≥ 0` and
  `γ ≤ 1` give `h_next ≥ 0` — a two-line induction and the entire safety guarantee.
- With `ω`, the same argument needs `ω γ ≤ 1`. That is why `omega_max * gamma ≤ 1` is a hard
  configuration check rather than a warning ([06_SOLVER.md §6.3](06_SOLVER.md)).

`γ ∈ (0, 1]` and `ω ≥ 0` are validated by the caller, not here. This function stays a pure
expression so it can be used symbolically and numerically without branching.

## §4.6 Tests — `codegen/tests/test_models.py`, M1

Not in `INFO.md`; add it, and register it alongside the other pytest file
([03_BUILD_SYSTEM.md §3.8](03_BUILD_SYSTEM.md)).

| Test | Assertion |
| --- | --- |
| `ExactZohMatchesRk4ForLinearModel` | for `double_integrator_2d`, `"exact"` and `"rk4"` agree to 1e-12 — RK4 is exact for a linear system, so any disagreement is a coding error, not a truncation error |
| `Rk4MatchesAnalyticUnicycleArc` | constant `(v, ω)` traces a circular arc of radius `v/ω`; match the closed form to 1e-9 over 100 steps |
| `EulerIsWorseThanRk4` | the demonstration the notebook cites; assert the ordering of the errors, not their values |
| `BarrierSignConvention` | `h > 0` strictly outside the inflated obstacle, `= 0` on the boundary to 1e-12, `< 0` inside |
| `BarrierGradientMatchesFiniteDifference` | central differences, step `1e-5·max(1,|x_i|)`, tolerance 1e-6 |
| `DcbfConstraintReducesToFixedDecayAtOmegaOne` | `dcbf_constraint(a, b, g, 1.0) == dcbf_constraint(a, b, g)` exactly |
| `PositionSelectorIsVerticalForPlanarQuadrotor` | guards §16.2's favourite bug |

Fixed RNG seed, printed at the top of the module. A randomised test that cannot be replayed is a
lottery, not a test.
