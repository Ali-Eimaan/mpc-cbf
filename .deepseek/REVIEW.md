# REVIEW.md — review protocol and findings log

**Status: no review has been run.** The repository is a skeleton; nothing has been implemented,
built, or tested. This file is the protocol for the first review sweep and the log it writes into.

Run a review sweep at the end of every milestone in [15_ROADMAP.md](15_ROADMAP.md), and a full
sweep before any release tag (§18).

---

## How to read an issue id

`R<round>-<number>` — e.g. `R1-8` is the eighth issue of round one. **Ids are permanent.** When a
later round revisits an issue it gets a new id that references the old one. Do not renumber.

## Agent assignment

| Agent | Use for |
| --- | --- |
| `deepseek-v4-flash` | Mechanical, well-specified fixes: a known sign error, a renamed constant, a missing include, a doc edit. The fix is stated exactly and the verification is a single test. |
| `deepseek-v4-pro` | Anything needing derivation, design judgement, or a change that ripples across files: re-deriving the tightening margin, restructuring the parameter layout, resolving a spec ambiguity. |

**Rule for both:** every fix lands with the test that proves it, in the same change (rule 5 in
[00_RULES.md](00_RULES.md)). If a fix cannot be verified by a test, say so in the commit message
and explain how it was verified instead.

## Severity

| Severity | Means | Examples in this repository |
| --- | --- | --- |
| **Critical** | Produces an unsafe control, or breaks the build | inverted DCBF row sign; an acados status mapping that reports infeasible as success; tightening added instead of subtracted; an under-approximating set operation |
| **Major** | Wrong results, or violates a spec decision | the two `h` definitions drifting; obstacle radius inflated twice; `ω` penalised instead of `(ω−1)²`; `z_0 = x0` every step; parameter-layout divergence between C++ and Python |
| **Minor** | Robustness, clarity, hygiene | missing throttle on a loop warning; an unnamed parameter in an error message; a stale comment |

A "solves cleanly and drives into the obstacle" bug is always **Critical**, however small the
diff. That is the class this repository exists to get right.

---

## §R.1 What to check, per sweep

Work down this list. It is ordered by cost-to-discover-later, not by where the code lives.

### Signs and inequalities — always first

- [ ] Every row in [16_CONVENTIONS.md §16.1](16_CONVENTIONS.md) checked against the **computed
      constraint values** (`cbf_values`, `cbf_slack`), not against trajectory behaviour
- [ ] DCBF rows lower-bounded at 0 with a large finite upper bound, never `inf`
- [ ] Both barrier rows emitted — distance **and** decay ([05_CODEGEN.md §5.4](05_CODEGEN.md))
- [ ] `h > 0` inside the safe set everywhere, including in the CasADi expression
- [ ] `γ ∈ (0,1]` validated; `ω_max·γ ≤ 1 + 1e-9` validated, and the boundary case **accepted**
- [ ] `(ω − 1)²` in the cost, not `ω` ([16_CONVENTIONS.md §16.1](16_CONVENTIONS.md) trap 2)
- [ ] Tightening **subtracted** from `h`, in both rows ([08_TUBE.md §8.4](08_TUBE.md))
- [ ] `discreteLqrGain` returns `K` with `A + BK` Schur, and it is asserted

### The four failure modes named in §0

- [ ] No path returns a solver iterate as a solution on a non-optimal status, except the
      documented `kMaxIterations`-with-finite-iterate case ([06_SOLVER.md §6.5.1](06_SOLVER.md))
- [ ] No silent fallback control anywhere in the solver library
      ([03_BUILD_SYSTEM.md §3.2](03_BUILD_SYSTEM.md))
- [ ] No claim of recursive feasibility in any comment, docstring, notebook or derivation
      ([01_OVERVIEW.md §1.6](01_OVERVIEW.md))
- [ ] `barrier_expression` and `barrierValue` agree, and the test enforcing it exists

### acados integration

- [ ] `integrator_type == 'DISCRETE'` — no `ERK` anywhere ([05_CODEGEN.md §5.2](05_CODEGEN.md))
- [ ] **The status mapping has been read out of the installed acados headers** and the verified
      mapping is in a comment (risk V12 — this is the Critical one)
- [ ] Parameter layout identical in the generator, the C++ solver and the Python path
- [ ] Obstacle radius inflated exactly once ([06_SOLVER.md §6.4](06_SOLVER.md))
- [ ] Obstacles propagated to each stage on both sides
- [ ] The `x_{k+1}` convention (substituted vs next node) is one choice, documented

### Sets and the tube

- [ ] Every set approximation is an **over**-approximation
      ([07_SETS.md §7.2](07_SETS.md))
- [ ] `Ω` verified invariant in `initialize()`, not merely computed
- [ ] Empty tightened `X` or `U` rejects at startup with a named reason
- [ ] `z_0` follows the shifted-nominal policy, and tube resets are counted
      ([08_TUBE.md §8.5](08_TUBE.md))
- [ ] The ancillary clip counter is exposed and asserted zero
- [ ] `kNone` unreachable without the explicit ablation flag

### Numerical and real-time

- [ ] No allocation in `solve()` after `initialize()`
- [ ] Tolerances match the budget in [07_SETS.md §7.6](07_SETS.md) — none tightened to "make it
      pass"
- [ ] `kActiveTolerance` influences diagnostics only, never a constraint or a status
- [ ] Timing numbers come from a Release build with the CPU named
- [ ] Nothing unthrottled in the control loop

### Honesty

- [ ] No number in the README, docs, notebooks or comments that is not reproducible
- [ ] No paper figure number quoted without having been checked against the paper
- [ ] No citation field invented ([13_DOCS.md §13.3](13_DOCS.md))
- [ ] Every `UNVERIFIED` either resolved or still labelled
- [ ] `REPRODUCTION_REPORT.md`'s Deviations section is populated, not empty
- [ ] The Limitations section lists everything in [01_OVERVIEW.md §1.6](01_OVERVIEW.md)
- [ ] No acceptance criterion weakened in place
- [ ] A7 verified by CI, or the README does not imply it is
      ([14_CI.md §14.5](14_CI.md))
- [ ] The A8 ablation actually fails — the tube has been shown to be *necessary*

### Hygiene

- [ ] No `TODO(deepseek …)` left on implemented code
- [ ] No `throw std::logic_error("… not implemented")` remaining
- [ ] Every source file carries the SPDX header ([02_ENVIRONMENT.md §2.6](02_ENVIRONMENT.md))
- [ ] Every new test registered in `CMakeLists.txt` ([03_BUILD_SYSTEM.md §3.8](03_BUILD_SYSTEM.md))
- [ ] `-Wconversion` clean, not silenced (generated acados code excepted, narrowly)

---

## §R.2 Status summary

Update this table at the end of each sweep.

| Severity | Open | Fixed this round |
| --- | --- | --- |
| Critical | — | — |
| Major | — | — |
| Minor | — | — |

*(No sweep has been run. Replace the dashes with counts, and add the findings below, when R1
happens.)*

---

## §R.3 Findings

Use one subsection per issue, in this shape:

```markdown
### R1-1 · [Critical] DCBF row lower bound has the wrong sign

**File:** `mpc_cbf_unified/src/mpc_cbf_solver.cpp:142`
**Spec:** [16_CONVENTIONS.md §16.1](16_CONVENTIONS.md)
**Agent:** deepseek-v4-flash

**What is wrong:** <one sentence>
**How it fails:** <the concrete state and obstacle geometry where it produces an unsafe control>
**Fix:** <the exact change>
**Verified by:** <the test, by name, added in the same change>
**Status:** open | fixed in <commit>
```

The **How it fails** line is not optional. A finding without a concrete failure scenario is an
opinion, and opinions do not get fixed in the right order.

---

## §R.4 Reviewing without a build

If the repository has never been compiled at the time of a sweep — which will be true for R1 —
say so at the top of the round and be explicit that findings come from reading the code and
re-deriving the mathematics, not from a failing build. Then:

- Do **not** report performance findings. You cannot see them.
- Do **not** report "this would not compile" unless you are certain; report it as a question.
- Do **not** report an acados API mismatch as a defect without having checked the installed
  version — the documents themselves flag those names as UNVERIFIED
  ([02_ENVIRONMENT.md §2.1](02_ENVIRONMENT.md)).
- **Do** report every sign, formula, unit and layout finding. Those are exactly what a reading
  review catches and a passing test suite can miss.

A review that overstates its evidence is worse than no review, for the same reason a green summary
over a broken build is (rule 9, [00_RULES.md](00_RULES.md)).
