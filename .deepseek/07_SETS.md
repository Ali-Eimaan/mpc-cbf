# §7 · Convex sets and invariant-set computation

**Governs:** `include/mpc_cbf_unified/disturbance_sets.hpp`, implemented in
`src/tube_mpc_cbf_solver.cpp` (split out per [03_BUILD_SYSTEM.md §3.1](03_BUILD_SYSTEM.md) when it
grows)
**Milestone:** M6
**Done when:** the set primitives and RPI tests in
[10_TESTS.md §10.2](10_TESTS.md) are green, including the analytic RPI reference.

This is cheap code that everything above it inherits. Implement the primitives first and test
them hard — a wrong Minkowski sum surfaces as an inexplicable tube result three layers up.

---

## §7.1 The LP backend — write exactly one

`Polytope::support`, `isEmpty` and `removeRedundantHalfspaces` all need a linear program. Write
**one** dense simplex (Big-M or two-phase, ~120 lines) in an anonymous namespace and route
everything through it.

Do not add an external LP dependency: the problems are `n ≤ 6` with at most a few hundred rows,
and a new dependency for that is not worth the build complexity. Do not write three ad-hoc
solvers either — the second one will disagree with the first at a tolerance nobody checks.

Prefer clarity and numerical care over speed: explicit tolerances, no division without a guard,
and a documented behaviour on an unbounded direction (`support` returns `+inf`).

## §7.2 The exactness table — keep it honest

| Operation | `Polytope` | `Zonotope` |
| --- | --- | --- |
| support | exact (LP) | exact, closed form `dᵀc + ‖Gᵀd‖₁` |
| Minkowski sum | **outer approximation** — support in the union of both normal sets | exact — concatenate generators |
| Pontryagin difference | exact — `b_i ← b_i − h_Q(a_i)` | n/a |
| linear map | exact if invertible; else via vertices (`n ≤ 3`) | exact |
| order reduction | n/a | **outer approximation** (Girard) |
| vertex enumeration | `n ≤ 3` only | via `toPolytope` |

**Every approximation MUST be an over-approximation.** An under-approximation anywhere in this
file is silently unsafe: the tube would certify an error bound the system can exceed, the RPI
check would still pass, and the only symptom would be an occasional collision under disturbance.
`OrderReductionOverApproximates` ([10_TESTS.md §10.2](10_TESTS.md)) samples and checks
containment for exactly this reason.

Where an operation degrades to an approximation, **say so in the log at the point of use**, not
only in a comment. A reader of a tube result needs to know whether `Ω` was exact.

## §7.3 `Polytope`

`{ x : A x ≤ b }`. Every constructor normalises rows to unit 2-norm, so tolerances have a
geometric meaning; zero rows with `b_i ≥ 0` are dropped and `b_i < 0` marks the set empty.

| Method | Notes |
| --- | --- |
| `box(half_widths)` / `box(lower, upper)` | `A = [I; −I]`; reject `lower > upper` |
| `fromVertices(points)` | 2-D monotone chain; 3-D incremental hull. **Throw `std::invalid_argument` above 3-D** rather than returning something plausible |
| `support(d)` | LP; `+inf` when unbounded |
| `contains(x, tol)` | `A x ≤ b + tol` |
| `minkowskiSum(other)` | supports in the union of normals, then redundancy removal |
| `pontryaginDifference(other)` | `b_i ← b_i − h_other(a_i)`. Exact. **The tightening workhorse** |
| `linearMap(M)` | `(A M⁻¹, b)` when invertible |
| `removeRedundantHalfspaces(tol)` | row `i` redundant iff `max{a_iᵀx : A_{−i}x ≤ b_{−i}} ≤ b_i + tol`; one LP per row |
| `maxNorm()` | vertices in low dimension; otherwise a direction grid — and then it is an **under**-estimate, so log it and do not use it for tightening |
| `isEmpty()` | Chebyshev-centre LP; cache the result |

`maxNorm()`'s fallback is the one place where an under-approximation is tolerable, because it is
only used by the Lipschitz tightening, which is itself the conservative option. Guard it anyway:
if `tighten_mode == kLipschitz` and `maxNorm()` took the grid path, refuse to initialise.

## §7.4 `Zonotope`

`{ c + G z : ‖z‖_∞ ≤ 1 }`. Minkowski sums and linear maps are exact and cheap here, which is why
the RPI iteration runs in this representation and converts to `Polytope` only at the end.

`support(d) = dᵀc + ‖Gᵀd‖₁` — closed form, no LP. This is what makes the RPI iteration tractable.

`reduceOrder(max_generators)` — Girard's method: sort generators by `‖g‖₁ − ‖g‖_∞`, keep the
largest `max_generators − n`, replace the rest by the diagonal box of their absolute row sums.
Must never shrink the set.

`toPolytope()` is exact for `n ≤ 3`; above that the half-space count is exponential in the
generator count, so **call `reduceOrder()` first** and say in the log how many generators were
dropped.

## §7.5 `computeRpiSet()` — Raković et al. (2005)

For `e_{k+1} = A_cl e_k + w_k`, `w_k ∈ W`, find `Ω` with `A_cl Ω ⊕ W ⊆ Ω`.

```
if spectralRadius(A_cl) >= 1:                    return {converged = false}
for s = 1 .. max_iterations:
    alpha_s = max over directions a of  h_W(A_clᵀ^s a) / h_W(a)
    if alpha_s >= 1: continue
    M(s) = max over axes i of  Σ_{j=0}^{s-1} [ h_W(A_clᵀ^j e_i) + h_W(−A_clᵀ^j e_i) ]
    if alpha_s / (1 − alpha_s) * M(s) <= epsilon: break
F_s   = ⊕_{j=0}^{s-1} A_cl^j W        # zonotope; reduceOrder every few steps
Omega = F_s scaled by 1 / (1 − alpha)
```

Fill `RpiResult` with the set in both representations, `s`, `alpha`, and `converged`.

**Analytic reference for the test:** `A_cl = λI`, `W = box(w)` gives `Ω = box(w/(1−λ))` exactly.
`RpiTest.ConvergesForSchurStableSystem` checks against it, and it is the only place in the tube
path where you have a closed form to compare with — do not skip it.

Non-convergence within `max_iterations` returns `converged = false`. The caller must reject, not
proceed with a partial set ([08_TUBE.md §8.3](08_TUBE.md)).

`isRobustPositivelyInvariant(A_cl, Ω, W, tol)`: for every row `a_i` of `Ω`, check
`h_Ω(A_clᵀ a_i) + h_W(a_i) ≤ b_i + tol`. Cheap, and it is the certificate itself rather than a
consequence of it — run it in `initialize()` even though `computeRpiSet` should guarantee it.

## §7.6 Numerical budget

| Quantity | Tolerance | Why |
| --- | --- | --- |
| set containment (`contains`) | 1e-9 | rows are unit-normalised, so this is a distance in metres |
| redundancy removal | 1e-9 | same |
| RPI invariance check | 1e-6 | accumulates `s` support evaluations |
| RPI Hausdorff target `epsilon` | 1e-3 | the default; it bounds how much larger `Ω` is than the true mRPI |
| C++ vs Python `Ω` support | 1e-6 | two different algorithms; tighter is not meaningful |

Do not tighten one because it happens to pass on your machine. Each of these is a statement about
how much error the layer above can absorb, not about how accurate your implementation happened to
be today.

## §7.7 `discreteLqrGain()`

Iterate `P ← Q + AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA` to a fixed point (`‖P − P_prev‖_∞ < tol`), then

```
K = −(R + BᵀPB)⁻¹ BᵀPA
```

**Sign convention: the returned `K` makes `A + BK` Schur.** The minus is inside `K`. Half the
literature defines `u = −Kx` and returns the positive form; if you copy from there, the RPI
iteration will not converge and you will spend an afternoon in `computeRpiSet`.

Use `Eigen::LLT` for the inverse and fail loudly if `R + BᵀPB` is not positive definite. Assert
`spectralRadius(A + B·K) < 1` before returning — one line, and it converts the most likely error
in this file into an immediate, named failure.

The Python side uses `scipy.linalg.solve_discrete_are` instead, deliberately
([05_CODEGEN.md §5.7](05_CODEGEN.md)).

## §7.8 `lipschitzBoundOnTube()`

`max ‖∇h(z + v)‖₂` over the vertices `v` of `Ω`. For the quadratic barrier the gradient is affine,
so the maximum is attained at a vertex and this is exact — say so in a comment, because it looks
like a sampling heuristic and is not.
