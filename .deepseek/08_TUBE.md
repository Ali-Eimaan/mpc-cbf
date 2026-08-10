# §8 · `TubeMpcCbfSolver`

**Governs:** `include/mpc_cbf_unified/tube_mpc_cbf_solver.hpp`, `src/tube_mpc_cbf_solver.cpp`
**Milestone:** M7
**Done when:** A8 is green — including its second half, the ablation.

This is the extension beyond the two reproduced papers, and the part of the repository a reviewer
will read most sceptically. It is also the part where a sign error is least likely to be caught by
a demo, because a wrongly-tightened tube still produces smooth, cautious-looking motion.

---

## §8.1 The problem

Plant `x_{k+1} = A x_k + B u_k + w_k`, `w_k ∈ W` compact convex with `0 ∈ W`.

```
nominal:   z_{k+1} = A z_k + B v_k
applied:   u_k = v_k + K (x_k − z_k)
error:     e_{k+1} = (A + BK) e_k + w_k,     e_k ∈ Ω for all k if e_0 ∈ Ω
tighten:   X ⊖ Ω,   U ⊖ KΩ,   and the barrier margin of §8.4
```

Nominal MPC-CBF guarantees safety of the *predicted* state. Under an additive disturbance the
true state can violate `h ≥ 0` even though every solved problem was feasible and every DCBF row
held. That gap is what this class closes, and it is worth stating in exactly those terms wherever
the tube is described.

## §8.2 What is shared with `MpcCbfSolver`

Reference handling, obstacle pruning and propagation ([§6.4](06_SOLVER.md)), the diagnostics block
([§6.7](06_SOLVER.md)) and the acados plumbing ([§6.5](06_SOLVER.md)) are identical.

**Factor the shared parts into file-local helpers rather than duplicating them.** Two copies of
the obstacle-propagation loop is how the tube path ends up inflating radii differently from the
nominal path, and the resulting discrepancy looks like a tube bug for a long time before anyone
checks the nominal side.

## §8.3 `initialize()` — order matters

```
1  validate    the MPC/CBF configs exactly as §6.3
2  (A, B)      exact ZOH for double_integrator_2d; otherwise linearise about the current
               reference and log at WARN that the certificate is local to that point (§1.6)
3  K           compute_gain_from_lqr ? discreteLqrGain(A, B, diag(lqr_Q), diag(lqr_R)) : tube.K
               reject if spectralRadius(A + B K) >= 1 - 1e-9
4  Omega       computeRpiSet(A + B K, W, rpi_epsilon, rpi_max_iterations)
               reject if !converged
5  tighten     tightened_x = X ⊖ Omega;  tightened_u = U ⊖ (K Omega)
               reject if either isEmpty()
6  verify      verifyInvariance(); reject on failure
7  log         alpha, s, Omega's bounding box, ||K||, at INFO
8  allocate    the acados tube solver with the tightened bounds
```

Steps 3–6 **are** the robustness argument. Each rejection is a correct outcome:

| Rejection | What it means | What NOT to do |
| --- | --- | --- |
| `A + BK` not Schur | the ancillary gain does not stabilise the error | do not "fix" it by shrinking `W` |
| RPI did not converge | `epsilon` unreachable in `max_iterations`, or `W` badly conditioned | do not use the partial set |
| tightened `X` empty | the tube does not fit between the state bounds | widen `X` or shrink `W` |
| tightened `U` empty | **the disturbance exceeds the actuator authority** | this is a physical statement about the system, not a numerical inconvenience |

Failing at startup is correct behaviour. A runtime infeasibility caused by an oversized `W` is
not — by then the vehicle is moving.

`initialize()` is expensive (expect O(100 ms)). It MUST NOT run inside the control loop. If
`relinearisation_threshold > 0` and you implement re-computation, do it on a separate thread and
swap the result in atomically, or do not implement it at all and say so.

## §8.4 `tighteningFor()` — the derivation you must not get wrong

Goal: a margin `c_j(z) ≥ 0` such that `h_j(z) − c_j(z) ≥ 0 ⟹ h_j(z + e) ≥ 0` for all `e ∈ Ω`.

With `h_j(x) = ‖P x − p_j‖² − r²`:

```
h_j(z + e) = h_j(z) + 2 (P z − p_j)ᵀ P e + ‖P e‖²
```

so

```
min_{e ∈ Ω} h_j(z + e)  ≥  h_j(z) − max_{e ∈ Ω} [ −2 (P z − p_j)ᵀ P e ]
                        =  h_j(z) − h_Ω( −∇h_j(z) ),      ∇h_j(z) = 2 Pᵀ (P z − p_j)
```

**The dropped `‖Pe‖²` term is non-negative in `h_j(z+e)`, so discarding it makes the bound
smaller — the margin is an over-estimate and therefore sound.** Do not "improve" the formula by
adding the quadratic back with a positive sign; that direction is unsound and it will not fail
any test you are likely to write.

| `TightenMode` | `c` |
| --- | --- |
| `kSupportFunction` | `Omega.support(−∇h_j(z))` — exact-up-to-the-dropped-term, the default |
| `kLipschitz` | `lipschitz_h * Omega.maxNorm()` — sound, always at least as conservative |
| `kNone` | `0` — **deliberately unsafe**, the A8 ablation. Never ship it |

`c ≥ 0` always, because `0 ∈ Ω` implies `h_Ω(d) ≥ 0`. Assert it.

`SupportTighteningDominatesLipschitz` ([10_TESTS.md §10.2](10_TESTS.md)) pins the ordering.

`kNone` exists only so `NominalCbfViolatesUnderSameDisturbance` can demonstrate that the
tightening is *necessary*. Guard it: refuse to run with `kNone` unless an explicit
`allow_unsafe_ablation` flag is set, so it cannot be reached from a launch file by accident.

## §8.5 `solve()` — the `z_0` policy

```
first solve, or (x0 − z_prev.col(1)) ∉ Omega  ->  z_0 = x0              (reset the tube)
otherwise                                     ->  z_0 = z_prev.col(1)   (shifted nominal)
```

**Anchoring `z_0` to the shifted nominal state is what makes the tube argument hold across
steps.** Setting `z_0 = x0` every step throws the guarantee away and silently degrades to nominal
MPC with tightened bounds — which still looks cautious, still passes a casual demo, and no longer
certifies anything. Log at DEBUG each time the tube resets, and count the resets in diagnostics;
a controller that resets every step is not running a tube.

Then:

1. Compute `c_{j,k}` for every obstacle and stage via `tighteningFor()` and push them as
   parameters ([§5.3](05_CODEGEN.md)).
2. Solve the nominal problem; read `z_pred`, `v_pred`.
3. `u_applied = clip(v_0 + K (x0 − z_0), u_min, u_max)`.
4. Fill `robust_cbf_values` and `tightening` for the plots.

**Count clip events.** A correctly computed tightening means the clip never triggers, because
`U ⊖ KΩ` was chosen precisely so the ancillary term fits. `AncillaryInputRespectsBounds`
([10_TESTS.md §10.2](10_TESTS.md)) asserts the counter stays zero. If it fires, the tightened
input set is wrong — surface it in diagnostics rather than absorbing it, because a saturating
ancillary law voids the guarantee and nothing else will tell you.

## §8.6 Accessors

`rpiSet()`, `ancillaryGain()`, `tightenedStateSet()`, `tightenedInputSet()` exist so the tests and
the notebooks can plot and check the certificate rather than trusting it. Keep them cheap and
`const`; the notebook in [12_ANALYSIS.md §12.5](12_ANALYSIS.md) plots `Ω` against the sampled
error cloud, which is the most convincing single figure in the repository.

`verifyInvariance(tol)` re-runs `isRobustPositivelyInvariant` on the stored data. Call it from
`initialize()` and expose it so the gtest can call it independently.

## §8.7 What the tube does not cover

Repeat these wherever a robustness claim appears ([01_OVERVIEW.md §1.6](01_OVERVIEW.md)):

- `Ω` is exact only for `double_integrator_2d`; elsewhere it is a **local** certificate at the
  linearisation point.
- **Obstacle prediction error is not in `W`** unless you put it there. The demos use
  constant-velocity obstacles, so the prediction is exact and this does not bite — which is
  precisely why it must be stated rather than demonstrated.
- State-estimation error is not modelled.
- The guarantee assumes the ancillary law is applied unsaturated (§8.5).
- Feasibility at `t = 0` is assumed, not proved (§1.6).
