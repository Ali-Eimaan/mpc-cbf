# `.deepseek/` — implementation specification

**You are the implementing model.** This directory is your complete instruction set for turning
the `mpc-cbf` skeleton into working code. Everything you need is here; nothing outside this
directory instructs you.

**Scope:** every file in the repository containing a `TODO(deepseek …)` marker.
**Authority:** where a document here and a code comment disagree, the document wins — say so in
the commit message rather than silently diverging.

> **Provenance.** This directory was adapted from the `cbf-safety-filter` specification of the
> same author. The working rules, review protocol and toolchain notes carry over; every
> subsystem document was rewritten for this repository. If you find a sentence that still talks
> about safety filters, QP row assembly or OSQP, it is a leftover — report it.

---

## Read in this order

| Read first | Document | Covers |
| --- | --- | --- |
| 1 | [00_RULES.md](00_RULES.md) | How to work. Non-negotiable. Read before touching a file. |
| 2 | [01_OVERVIEW.md](01_OVERVIEW.md) | What is being built, why it exists, acceptance criteria A1–A9 |
| 3 | [02_ENVIRONMENT.md](02_ENVIRONMENT.md) | Ubuntu 26.04 / ROS 2 Lyrical Luth / acados, pinned versions, **version risk register** |
| 4 | [15_ROADMAP.md](15_ROADMAP.md) | Milestones M1–M10 in dependency order, definition of done |
| 5 | [16_CONVENTIONS.md](16_CONVENTIONS.md) | **Sign conventions**, units, numerical policy, traps. Re-read when confused. |

§16.1 is the single most important page in this directory. A flipped inequality in a barrier row
does not crash — it produces a controller that solves cleanly, reports success, and drives into
the obstacle. Read it before you assemble your first constraint row, not after your first
mysterious test failure.

Then work through the milestones, opening the subsystem document for each:

| Document | Implements | Milestone |
| --- | --- | --- |
| [03_BUILD_SYSTEM.md](03_BUILD_SYSTEM.md) | `mpc_cbf_unified/CMakeLists.txt`, `package.xml` | throughout |
| [04_MODELS.md](04_MODELS.md) | `codegen/models.py` — dynamics, discretisation, the barrier | M1 |
| [05_CODEGEN.md](05_CODEGEN.md) | `codegen/generate_mpc_cbf_solver.py`, `generate_tube_solver.py` | M2, M7 |
| [06_SOLVER.md](06_SOLVER.md) | `mpc_cbf_solver.hpp` / `.cpp` | M3, M4 |
| [07_SETS.md](07_SETS.md) | `disturbance_sets.hpp` — polytopes, zonotopes, RPI, LQR | M6 |
| [08_TUBE.md](08_TUBE.md) | `tube_mpc_cbf_solver.hpp` / `.cpp` | M7 |
| [09_NODE.md](09_NODE.md) | `mpc_cbf_node.*`, `config/`, `launch/`, `scripts/` | M8 |
| [10_TESTS.md](10_TESTS.md) | everything under `mpc_cbf_unified/test/` | with each milestone |
| [11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md) | the Python solver path and the parity check | M5 |
| [14_CI.md](14_CI.md) | `.github/workflows/` | M9 |
| [12_ANALYSIS.md](12_ANALYSIS.md) | `analysis/`, `reproduction/`, `media/` | M10 |
| [13_DOCS.md](13_DOCS.md) | `docs/` derivations, `REPRODUCTION_REPORT.md`, root `README.md` | M10 |

[FILE_MAP.md](FILE_MAP.md) — every skeleton file in the repository mapped to the document that
specifies it. Use it when you have a file and need its spec.

[REVIEW.md](REVIEW.md) and [FIX_REPORT.md](FIX_REPORT.md) are the review protocol and the
reporting template. Read REVIEW.md before you declare a milestone done.

[INFO.md](INFO.md) is the author's original portfolio specification for this repository —
**read-only**. It fixes the target file structure; [01_OVERVIEW.md §1.5](01_OVERVIEW.md) records
every file the skeleton adds beyond it, and why.

---

## Section numbering

Each document owns a section number that matches its filename prefix: `04_MODELS.md` contains §4,
`08_TUBE.md` contains §8, and so on. A cross-reference written `§6.5` therefore always means
"section 6.5, which lives in `06_SOLVER.md`". This holds everywhere, including in the
`TODO(deepseek §6.5)` comments inside the skeleton source files.

`15_ROADMAP.md` carries both §15 (implementation order) and §17 (definition of done) and §18
(release gate).

## Conventions used in these documents

- **MUST** — required for correctness or safety. Deviating is a bug.
- **SHOULD** — strong default. Deviate only with a stated reason in a code comment.
- **UNVERIFIED** — a value or assumption that has not been confirmed. Verify before relying on
  it, and update the document with what you found.

## Before you start

Run this to see the work remaining:

```bash
grep -rn "TODO(deepseek" --exclude-dir=.git --exclude-dir=.deepseek .
```

Every one of those markers is specified somewhere in this directory. If you find one that is
not, that is a gap in the spec — report it rather than guessing.

## What the repository is not

This is a **single-agent predictive safety controller**. It is not a planner, not a swarm
framework, and not a perception stack. Obstacles arrive as circles with velocities; goals arrive
as poses. If you find yourself writing obstacle detection, global path planning, or multi-agent
constraint negotiation, you have left the scope of this repository — that work belongs in the
author's `transition-viable-swarm`, which consumes this solver as its per-agent inner problem.

Two things this repository is *also* not, which matter more than they sound:

- **It is not a proof of recursive feasibility.** Discrete-time CBF constraints inside an MPC do
  not confer it, and none of the standard sufficient conditions are implemented here. See
  [01_OVERVIEW.md §1.6](01_OVERVIEW.md). Any comment, docstring or notebook claiming otherwise is
  a bug.
- **It is not a re-implementation of acados.** The solver back-end is a dependency. Your job is
  the formulation, the diagnostics, the tube machinery and the evidence — not the QP.
