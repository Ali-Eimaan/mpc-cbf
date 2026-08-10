# §16 · Conventions and known traps

**Read §16.1 before M2. Re-read the whole page whenever something behaves inexplicably.** Most of
the expensive bugs in a predictive safety controller are on this page, and the first section is
the one that turns a working controller into a dangerous one.

---

## §16.1 Sign conventions — memorise these

| Quantity | Convention |
| --- | --- |
| `h(x)` | **positive inside** the safe set, zero on the boundary, negative in collision |
| Barrier form | `h_j(x) = ‖P x − p_j‖² − r_eff²`, **squared**, never the norm |
| Effective radius | `r_eff = r_obs + ego_radius + safety_margin` — radii **add** |
| DCBF condition | `h(x_{k+1}) − h(x_k) ≥ −γ h(x_k)`, i.e. `h(x_{k+1}) ≥ (1−γ) h(x_k)` |
| acados row for the DCBF | `con_h_expr = h(x_{k+1}) − h(x_k) + γ h(x_k)`, `lh = 0`, `uh = +BIG` |
| acados row for the distance | `con_h_expr = h(x_k)`, `lh = 0`, `uh = +BIG` |
| `γ` | `∈ (0, 1]`. Smaller = **more** conservative (acts earlier) |
| `ω` (relaxed decay) | `≥ 0`; larger = **more** relaxed. Safety needs `ω·γ ≤ 1` |
| LQR gain `K` | returned such that **`A + B K`** is Schur (the minus is inside `K`) |
| Tube margin `c` | **subtracted** from `h`: `h(z) − c ≥ 0`, and `c ≥ 0` always |
| Pontryagin difference | `b_i ← b_i − h_Q(a_i)` — the support function is **subtracted** |
| Support function | `h_P(d) = max{ dᵀx : x ∈ P }`, so `h_P(d) ≥ 0` whenever `0 ∈ P` |

**Write a test for each of these rows that inspects the assembled constraint values directly.**
Do not rely on end-to-end trajectories to catch a sign error. A controller with an inverted
barrier row still produces smooth, plausible motion — right up until the geometry gets tight, at
which point it drives into the obstacle with the solver reporting `kSuccess`.

This is the failure mode of this repository. Everything else is recoverable.

### The four sign traps specific to this formulation

1. **`γ` direction.** `γ → 0` is *conservative*, `γ = 1` is *permissive*. It reads backwards to
   most people the first time: `γ` is the fraction of barrier value the system is allowed to give
   up per step, so a large `γ` allows a fast approach to the boundary.
2. **`ω` direction.** Opposite to `γ` in feel: a large `ω` *relaxes*. Because relaxation is what
   restores feasibility, the penalty is on `(ω − 1)²`, not on `ω` — penalising `ω` directly
   biases the solver toward `ω = 0`, which is *stricter* than the nominal problem and makes
   feasibility worse, not better.
3. **Tightening sign.** `c` makes the nominal problem **harder**. If adding the tube makes your
   trajectories hug the obstacle more closely than nominal MPC-CBF did, `c` is being added where
   it should be subtracted.
4. **LQR sign.** `K = −(R + BᵀPB)⁻¹BᵀPA`. Half the textbooks define `u = −Kx` and return the
   positive form. Check `spectralRadius(A + B·K) < 1` in the code, once, and fail loudly.

## §16.2 Units, frames and dimensions

SI throughout. Distances in metres, angles in **radians**, time in **seconds** at every internal
interface.

`h` has units of **metres squared**. That means **class-K gains are not transferable between
barrier forms** and `γ` is dimensionless only because the DCBF condition is a ratio. If you ever
add a non-squared barrier, `γ` tuned on one will not behave the same on the other — say so in the
tuning notes rather than discovering it in a demo.

State layouts are fixed in [04_MODELS.md §4.2](04_MODELS.md) and mirrored in
`MpcCbfSolver::stateDimOf`. The position sub-vector is `(px, py)` for every model **except**
`quadrotor_planar`, where it is `(px, pz)`. That single exception is responsible for more wasted
time than any other line in the table; the planar quadrotor lives in the vertical plane.

Positions come out of `nav_msgs/Odometry` in the odom frame; **velocities come out in
`child_frame_id`**, i.e. body frame. Assuming otherwise silently rotates your state, and the
symptom is a controller that behaves correctly when the robot faces along `+x` and wrongly
otherwise ([§9.5](09_NODE.md)).

## §16.3 Time and discretisation

`rclcpp::Time` in the node, plain `double` seconds at the solver boundary.

`dt` is a **property of the generated solver**, not a runtime knob. Changing it means
regenerating. `control_rate_hz` in the YAML must equal `1/dt` unless you have a stated reason;
a mismatch means the plant advances by a different amount than the prediction assumed, and the
DCBF condition then holds for a system you are not controlling.

Obstacle prediction is constant-velocity over the horizon: `o_j(k) = o_j(0) + k·dt·v_j`. This is
exact in the demos (the simulated obstacles move at constant velocity) and wrong in general.
**Prediction error is not covered by the tube unless you put it in `W` deliberately** — say so
wherever a robustness claim is made.

## §16.4 Numerical policy

- Bounds use `±1e9` as the "unbounded" sentinel in YAML, never a true infinity. acados handles
  large finite bounds; `inf` in a cost or weight is always a bug.
- LP tolerance `1e-9` for set containment; `1e-6` for solver residuals; `1e-4` for anything
  involving a numeric Hessian or an RPI approximation at depth. These come from the accuracy
  budget in [07_SETS.md §7.6](07_SETS.md) — do not tighten one because it happens to pass.
- `kActiveTolerance = 1e-3` decides when a DCBF row counts as "active" for diagnostics only. It
  must never influence a constraint.
- Every set approximation MUST be an **over**-approximation. An under-approximation in the tube
  path is silently unsafe ([07_SETS.md §7.2](07_SETS.md)).
- Warm starts must never change a result, only the iteration count. If a warm start changes the
  answer beyond solver tolerance, the problem is degenerate — investigate rather than accept it.

## §16.5 Control rate and the sampled-data gap

The DCBF theory is genuinely discrete-time, which is the main reason this formulation was chosen
over a continuous-time CBF filter — there is no discretisation gap in the *condition itself*.

There is still a gap between the condition and reality: the condition constrains the **predicted**
`x_{k+1}` under the model. Model error, obstacle-prediction error and a plant that integrates at
a finer rate than `dt` all break the link between "the condition held" and "the vehicle is safe".

Practical rule: **the plant must be simulated at a step no larger than `dt`**, and inter-sample
behaviour must be checked. When a forward-invariance test fails at `dt = 0.1` and passes at
`dt = 0.01`, the implementation is right and the rate is the problem. Document the required rate
in `docs/README_math.md` and the README limitations. Do not relax the tolerance.

## §16.6 Logging and allocation

Nothing unthrottled in the control loop — `RCLCPP_*_THROTTLE` with a 1–2 s period. A `printf` at
10 Hz is a latency bug, not a debugging aid.

**No allocation in `solve()` after `initialize()`.** A2 is a p95 criterion and an allocation that
happens 1 % of the time is precisely what a tail measurement catches. `SolveDoesNotAllocate`
([10_TESTS.md §10.1](10_TESTS.md)) enforces it.

## §16.7 C++ standard

The package builds at **C++20**, but the sources MUST stay **C++17-compatible**: no concepts, no
ranges, no `std::format`. Risk V3 ([§2.1](02_ENVIRONMENT.md)) is unresolved, and keeping the code
at 17 is what makes the fallback a one-line change to `CMAKE_CXX_STANDARD`.

## §16.8 What must never appear in the same commit as a green report

- A tolerance that moved
- A test converted to `GTEST_SKIP` without a written reason
- A number in a README or notebook that no longer has a cell producing it
- A `TODO(deepseek …)` deleted without the implementation

---

## The failures you will actually hit

| Symptom | First thing to check |
| --- | --- |
| **Controller drives into the obstacle, solver reports `kSuccess`** | §16.1. Print the assembled `h` and DCBF residual per stage. This is a sign error until proven otherwise. |
| Infeasible whenever the obstacle is close | expected if `γ` is small and the horizon short — check against the feasible-set map ([12_ANALYSIS.md §12.4](12_ANALYSIS.md)) before calling it a bug |
| Every solve returns `kNotInitialized` | built without acados; `ACADOS_SOURCE_DIR` unset ([03_BUILD_SYSTEM.md §3.2](03_BUILD_SYSTEM.md)). This is deliberate, not a silent fallback |
| Diagnostics disagree with the trajectory | the two `h` definitions have drifted ([§4.4](04_MODELS.md), rule 4 in §0) |
| Solve time spikes intermittently | allocation in the loop (§16.6), or the parameter vector being resized per call |
| `ω` pinned at its upper bound everywhere | `omega_weight` too small relative to the tracking cost — it is not a bug, but the plot is then meaningless |
| Relaxed decay is *less* feasible than fixed | the penalty is on `ω` instead of `(ω − 1)²` (§16.1 trap 2) |
| Tube trajectories hug the obstacle more than nominal | tightening added instead of subtracted (§16.1 trap 3) |
| RPI iteration never converges | `A + BK` not Schur — check the LQR sign (§16.1 trap 4) |
| `initialize()` rejects a workable configuration | `omega_max · γ > 1`; at the YAML defaults the product is exactly 1.0, so the comparison must be `≤ 1 + 1e-9` ([06_SOLVER.md §6.3](06_SOLVER.md)) |
| Tightened input set empty at startup | `W` is too large for the actuator bounds. Correct behaviour — shrink `W` or widen `U`, do not bypass the check |
| Planar quadrotor behaves as if the obstacle is in the wrong place | position is `(px, pz)`, not `(px, py)` (§16.2) |
| Node works in sim, misbehaves on a rotated start | body-frame twist not rotated (§16.2) |
| Notebook numbers differ from the C++ numbers | parity check ([11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md)) — run it before debugging either side |

## A note on debugging order

When something is wrong in a demo, the cost of checking goes: **sign conventions (minutes) →
per-stage `h` and DCBF residual dumps (minutes) → the C++/Python parity check (minutes) → the
running ROS graph (hours)**. Work in that order.

The tests in [10_TESTS.md](10_TESTS.md) and the parity check in
[11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md) exist precisely so the cheap checks are
available before you need them. Reaching for `ros2 topic echo` first is the expensive path, and
it is the one everyone takes.
