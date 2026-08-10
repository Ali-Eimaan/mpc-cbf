# IMPLEMENTATION_GUIDE.md

**The implementation specification is this directory. Start at [README.md](README.md).**

This file used to sit at the repository root and hold the whole guide. It was folded into the
numbered documents here so there is exactly one specification rather than two that can drift
apart — a spec that has silently diverged from the code is worse than no spec, because the next
reader will trust it.

Do not re-expand this file. If you need to add specification content, add it to the document that
owns that section number.

---

## Where things went

The root guide's sections map onto this directory as follows. Nothing was dropped; several
sections grew.

| Root guide had | Now lives in |
| --- | --- |
| §0 Rules of engagement | [00_RULES.md](00_RULES.md) |
| §1 The mathematics (DCBF, the OCP, relaxed decay, feasibility, tube) | [04_MODELS.md §4.4–§4.5](04_MODELS.md), [05_CODEGEN.md §5.4](05_CODEGEN.md), [08_TUBE.md §8.1, §8.4](08_TUBE.md), [01_OVERVIEW.md §1.6](01_OVERVIEW.md) |
| §2 Code generation, acados prerequisites | [02_ENVIRONMENT.md §2.2](02_ENVIRONMENT.md), [05_CODEGEN.md](05_CODEGEN.md) |
| §2.2 Shared models | [04_MODELS.md](04_MODELS.md) |
| §2.3 Parameter vector layout | [05_CODEGEN.md §5.3](05_CODEGEN.md) |
| §3 `MpcCbfSolver` | [06_SOLVER.md](06_SOLVER.md) |
| §4 `TubeMpcCbfSolver` | [08_TUBE.md](08_TUBE.md) |
| §5 Convex sets | [07_SETS.md](07_SETS.md) |
| §6 ROS node | [09_NODE.md §9.1–§9.6](09_NODE.md) |
| §7 Launch files | [09_NODE.md §9.7](09_NODE.md) |
| §8 Tests | [10_TESTS.md](10_TESTS.md), [11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md) |
| §9 CI | [14_CI.md](14_CI.md) |
| §10 Milestones | [15_ROADMAP.md §15](15_ROADMAP.md) |
| §11 Notebooks | [12_ANALYSIS.md](12_ANALYSIS.md) |
| §12 Documentation | [13_DOCS.md](13_DOCS.md) |
| §13 Definition of done | [15_ROADMAP.md §17](15_ROADMAP.md) |
| Appendix A Pitfalls | [16_CONVENTIONS.md](16_CONVENTIONS.md) |
| Appendix B Commands | [02_ENVIRONMENT.md §2.4](02_ENVIRONMENT.md) |
| Appendix C Source papers | [13_DOCS.md §13.3](13_DOCS.md), [12_ANALYSIS.md](12_ANALYSIS.md) |

What the root guide did **not** have, and this directory adds: acceptance criteria A1–A9
([01_OVERVIEW.md §1.3](01_OVERVIEW.md)), a version risk register
([02_ENVIRONMENT.md §2.1](02_ENVIRONMENT.md)), a review protocol
([REVIEW.md](REVIEW.md)), a milestone reporting template ([FIX_REPORT.md](FIX_REPORT.md)), and a
file-to-spec map ([FILE_MAP.md](FILE_MAP.md)).

## Section numbering

A document's number matches its filename prefix, so `§8.4` always means "section 8.4, in
`08_TUBE.md`". The `TODO(deepseek §8.4)` markers in the source files carry the same numbers.

The command that lists the remaining work is in [README.md](README.md) — it is kept there, and
only there, so that running it does not match the documentation describing it.
