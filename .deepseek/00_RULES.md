# §0 · Working rules

**Read this before touching any file.** These rules are ordered by priority; when two conflict,
the lower number wins.

---

1. **One file per change.** Implement a file completely, build it, then move on. Do not open six
   files and leave all of them half-finished.

2. **Do not change public signatures.** The headers in
   `mpc_cbf_unified/include/mpc_cbf_unified/`, the parameter-vector layout in
   [05_CODEGEN.md §5.3](05_CODEGEN.md), and the config keys in
   [09_NODE.md §9.2](09_NODE.md) are the contract between subsystems. The C++ tests, the ROS
   node, the notebooks and the parity check are all written against them. If a signature is
   genuinely wrong, **stop and say so** instead of quietly editing it.

3. **Delete the `TODO(deepseek …)` comment when you implement it.** A leftover TODO on
   implemented code is a lie that costs the next reader ten minutes. Keep the surrounding
   explanatory comment — it encodes the mathematics and is not the TODO.

4. **Never invent a number.** Not a measured solve time, not a gain presented as tuned, not a
   figure number from a paper you did not open, not a citation field. If you do not have a
   source, leave the placeholder and mark it `UNVERIFIED`. This repository's entire value is
   that a reader can trust its numbers — it is a *reproduction*, and a reproduction with one
   invented figure is worth less than no reproduction.

5. **Every claim needs a check.** If you implement something whose correctness is not obvious —
   a DCBF constraint row, a Minkowski sum, an RPI iteration, a tightening margin — add the test
   that proves it **in the same change**. See [10_TESTS.md](10_TESTS.md).

6. **Sign errors are the failure mode of this repository.** A flipped inequality in a barrier row
   does not crash, does not throw, and does not fail a smoke test. It produces a controller that
   converges cleanly and drives into the obstacle. [16_CONVENTIONS.md §16.1](16_CONVENTIONS.md)
   fixes every sign once. Follow it exactly, and test the assembled constraint values directly,
   not just the end-to-end trajectory.

7. **Build order matters.** Follow the milestones in [15_ROADMAP.md](15_ROADMAP.md). The
   dependency chain is real: the solver cannot be tested before the generated code exists, and
   the tube cannot be tested before the set arithmetic is proven.

8. **If a step is blocked** — a missing acados, an unresolved version question, a decision these
   documents do not settle — implement everything that is *not* blocked, then report exactly what
   is blocked and why. Do not stub around it silently and do not guess.

9. **Report honestly.** If a test fails, say so and show the output. If you skipped something,
   say that. A green summary over a broken build is the most expensive thing you can produce
   here, because it will be believed.

---

## What "implemented" means

A file is not done when it compiles. It is done when:

- every `TODO(deepseek …)` in it is implemented and the marker deleted, or converted into a
  specific written issue with a reason
- it builds with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` clean (the warnings are on
  deliberately — fix them, do not silence them)
- its tests pass, in a Release build
- any behaviour a reader would not predict from the signature is documented in a comment

The full definition is §17 in [15_ROADMAP.md](15_ROADMAP.md).

## Four rules specific to a safety-critical predictive controller

**Never weaken a safety tolerance to make a test pass.** If `ClosedLoopStaysSafeForFullRollout`
fails at `min h ≥ −1e-6`, the answer is never `−1e-3`. Either the implementation is wrong, or
`dt` is too large for the chosen `γ` — and in the second case the fix is to document the required
rate ([16_CONVENTIONS.md §16.5](16_CONVENTIONS.md)), not to move the threshold.

**Never make an infeasible solve look feasible.** When acados returns a non-success status,
`MpcCbfSolution::status` reflects it and the configured `infeasible_policy` applies at the node
level. Returning the last iterate as though it were a solution is exactly how an infeasible
problem becomes a collision. The temptation to "just use what the solver gave us" will be strong
the first time a demo stalls. Resist it. (`kMaxIterations` with a finite iterate is the one
deliberate exception, and `usable()` is where that judgement is made and documented.)

**Never claim recursive feasibility.** It is not implemented and not guaranteed
([01_OVERVIEW.md §1.6](01_OVERVIEW.md)). The Python study *measures* the feasible set; it does
not assert a theorem. Wording matters here because a reviewer who catches one overclaim discounts
every other claim in the repository.

**Never let the two barrier definitions drift.** `h` is defined in exactly two places —
`barrier_expression()` in Python ([§4.4](04_MODELS.md)) and `MpcCbfSolver::barrierValue()` in C++
([§6.6](06_SOLVER.md)). They must agree to 1e-9, and a test enforces it. When they diverge, every
diagnostic, plot and reported number in the repository becomes wrong by a nonlinear factor, and
nothing crashes.

## When a document is wrong

These documents were written before the code existed. Some of it will turn out to be wrong —
particularly the version pins in [02_ENVIRONMENT.md](02_ENVIRONMENT.md) and the acados API names
in [05_CODEGEN.md](05_CODEGEN.md) and [06_SOLVER.md](06_SOLVER.md), which change between acados
releases.

When you find an error: **fix the document in the same commit as the code.** A specification
that has silently drifted from the implementation is worse than no specification, because the
next reader will trust it.

Do not weaken an acceptance criterion or a test threshold in place. If a criterion cannot be
met, change it explicitly in [01_OVERVIEW.md §1.3](01_OVERVIEW.md) with a written reason.
