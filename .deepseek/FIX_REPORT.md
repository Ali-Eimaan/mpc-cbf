# FIX_REPORT.md — milestone completion report

**Status: nothing implemented.** The repository is a skeleton; no milestone has been completed.
This file is the template. Fill one section per milestone completed, newest at the top, and keep
the old ones — the history of what was fixed and how it was verified is the record that makes the
repository's claims checkable.

Write this at the end of every milestone in [15_ROADMAP.md](15_ROADMAP.md), before starting the
next one.

---

## How to fill it in

Three rules, all from [00_RULES.md](00_RULES.md):

1. **Report the real result.** If a test fails, show the output. If you skipped something, say
   which and why. A green table over a broken build is the most expensive thing you can produce.
2. **Every number is measured.** Solve times come from a Release build, with the CPU named. A time
   you did not measure does not go in the table.
3. **Name what is still open.** A milestone can be complete with known open issues; it cannot be
   complete with hidden ones.

---

## Template — copy this block per milestone

```markdown
## M<n> · <milestone name>

**Date:** YYYY-MM-DD
**Commit:** <sha or "working tree">
**Spec:** <the .deepseek document(s) implemented>
**Build:** Release · <compiler and version> · <CPU, if any timing is reported>
**acados:** <tag and commit — required for any milestone from M2 onward>

### Test results

| # | Test | Type | Time | Result |
|---|------|------|------|--------|
| 1 | test_models | pytest | — | PASS / FAIL / SKIP |

Total: <n> passed, <n> failed, <n> skipped.

### Acceptance criteria touched

| Criterion | Before | After | Evidence |
|---|---|---|---|
| A4 | not met | met | `ClosedLoopStaysSafeForFullRollout`, min h = <value> over <n> rollouts |

### What was implemented

<Per file: what the body now does, and anything a reader would not predict from the signature.>

### Deviations from the spec

<Every place the implementation differs from the .deepseek document, and why. If a document was
wrong, say which section and confirm you fixed it in the same commit (rule: "When a document is
wrong", §0).>

### UNVERIFIED resolved

| Risk | Was | Is |
|---|---|---|
| V12 | acados status mapping assumed | read from <header>, mapping is <...> |

### Still open

<Known issues, with severity and where they are logged in REVIEW.md.>

### Numbers for downstream documents

<Anything REPRODUCTION_REPORT.md or the README will quote, so it is recorded once and cited
rather than re-measured inconsistently.>
```

---

## Milestone-specific notes

Things that must appear in particular reports, because they are easy to complete without.

| Milestone | Must record |
| --- | --- |
| M1 | the RK4-vs-analytic error, so later integration questions have a baseline |
| M2 | **the acados tag**, and the six generated solver names |
| M3 | the p95 solve time with the CPU named (A2); `min h` from the rollout (A4) |
| M4 | **the exact parameters that produce the MPC-DC / MPC-CBF separation** (A5) — a separation you did not record is not reproducible |
| M5 | the measured recovery rate and `max(ω·γ)` (A6); the parity residual (A7) |
| M6 | `α` and `s` from the RPI iteration, and the C++/Python support-function agreement |
| M7 | `min h` under worst-case disturbance, **and the ablation's `min h`** — both numbers, or A8 is not evidence |
| M8 | whether `2d_obstacle.launch.py` ran with no arguments from a clean clone |
| M9 | whether A7 is verified *by CI* or only locally |
| M10 | which paper figure numbers were checked against the actual papers |
