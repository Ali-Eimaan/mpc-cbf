# §15 · Implementation order · §17 · Definition of done · §18 · Release

---

## §15 · Milestones

Each milestone is independently verifiable. **Do not start one before the previous one passes.**

| # | Milestone | Files | Spec | Done when |
| --- | --- | --- | --- | --- |
| M1 | Models | `codegen/models.py`, `codegen/tests/test_models.py` | [04](04_MODELS.md) | RK4 matches analytic solutions to 1e-9; barrier gradient matches finite differences |
| M2 | Nominal codegen | `codegen/generate_mpc_cbf_solver.py` (fixed decay, double integrator) | [05](05_CODEGEN.md) | a generated solver solves the 2-D scenario **from Python**, before any C++ exists |
| M3 | Solver core | `mpc_cbf_solver.*`, most of `test_mpc_cbf_feasibility.cpp` | [06](06_SOLVER.md) | **A2, A3, A4** |
| M4 | The other two variants | `distance_only` + `relaxed_decay` codegen and their tests | [05 §5.5](05_CODEGEN.md), [06](06_SOLVER.md) | **A5** |
| M5 | Python path and parity | `test_recursive_feasibility.py`, `cpp_solve_cli.cpp` | [11](11_PYTHON_REFERENCE.md) | **A6, A7** |
| M6 | Convex sets | `disturbance_sets` implementations, set + RPI tests | [07](07_SETS.md) | analytic RPI reference matched; `RpiMatchesPythonReference` green |
| M7 | Tube | `tube_mpc_cbf_solver.*`, `generate_tube_solver.py`, tube tests | [08](08_TUBE.md), [05 §5.7](05_CODEGEN.md) | **A8**, including the ablation failing as designed |
| M8 | Node, config, launch, simulators | `mpc_cbf_node.*`, `config/`, `launch/`, `scripts/` | [09](09_NODE.md) | `2d_obstacle.launch.py` reaches the goal safely with no arguments |
| M9 | CI | `.github/workflows/` | [14](14_CI.md) | **A1**, and A7 verified *by CI* |
| M10 | Reproductions, analysis, docs, media | `reproduction/`, `analysis/`, `docs/`, `media/`, root `README.md` | [12](12_ANALYSIS.md), [13](13_DOCS.md) | **A9**; notebooks execute; README numbers reproducible |

### Why this order

**M1–M7 need no ROS at all.** The solver library does not depend on ROS messages or a node
([03_BUILD_SYSTEM.md §3.1](03_BUILD_SYSTEM.md)), and that is not an accident — it is what makes
the first seven milestones debuggable in a unit test instead of inside a running graph. Getting a
sign convention wrong is a five-minute fix at M3 and a two-hour one at M8.

**M2 before M3.** A C++ wrapper around a solver that has never solved anything gives you two
unvalidated layers and no way to tell which is wrong. Make the generated solver work from Python
first; then the C++ has a known-good reference.

**M5 before M6/M7** is the non-obvious one. The temptation is to build the tube next, because it
is the interesting part. Resist it: the parity check is what tells you the *nominal* solver is
correct, and discovering a parameter-packing bug after the tube is built on top means unwinding
both. A tube around a wrong controller is worse than no tube.

**M5 before M8** for the same reason, more strongly. A demo of a wrong controller is worse than no
demo, because it is persuasive.

**M6 before M7** because the tube is set arithmetic wearing a solver. Assembling a tightening from
an unvalidated Minkowski sum means finding out at M7 that both were wrong and not knowing which.

**M9 after M8** rather than early: CI that runs before there is anything to run produces noise,
and noisy CI gets ignored.

### Parallelism

M9 and M10 are independent and can follow M8 in either order. Everything else is a strict chain.

Within M10, the notebooks ([12](12_ANALYSIS.md)) and the derivations
([13 §13.2](13_DOCS.md)) can proceed in parallel, but the derivations' worked numbers must be
cross-checked against the notebooks' simulations before either is called done
([13_DOCS.md §13.4](13_DOCS.md)).

### The two milestones people underestimate

**M6.** Polytope and zonotope arithmetic with an LP backend is a week of careful work, not an
afternoon. Budget for it, and do not start the tube until the set tests are green.

**M10.** Filling `REPRODUCTION_REPORT.md` honestly means re-reading two papers and checking every
figure number. That is the milestone that makes the repository worth having; it is also the one
most likely to be rushed because the code already works.

---

## §17 · Definition of done

### Per file

- every `TODO(deepseek …)` implemented and the marker **deleted**, or converted into a specific
  written issue with a reason
- builds with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` clean
- its tests pass, in a **Release** build
- any behaviour a reader would not predict from the signature is documented in a comment
- carries the SPDX header ([§2.6](02_ENVIRONMENT.md))

### Per repository

- all nine acceptance criteria in [01_OVERVIEW.md §1.3](01_OVERVIEW.md) hold
- the README's numbers are reproducible from `analysis/` or the test suite, on the hardware named
  next to them
- the README's Limitations section is complete and specific ([13_DOCS.md §13.6](13_DOCS.md))
- no `UNVERIFIED` marker remains in [02_ENVIRONMENT.md §2.1](02_ENVIRONMENT.md), `CMakeLists.txt`
  or the workflow without an accompanying note explaining why it could not be resolved — **V12
  (the acados status mapping) must be resolved, not annotated**
- `REPRODUCTION_REPORT.md` contains no `TBD`
- every citation in `docs/PRIOR_ART.md` checked against the actual publication
- A7 is verified **by CI**, not merely by a local run ([14_CI.md §14.5](14_CI.md))
- no `throw std::logic_error("… not implemented")` remains anywhere

### Progress check

```bash
grep -rn "TODO(deepseek" --exclude-dir=.git --exclude-dir=.deepseek . | wc -l
```

```bash
grep -rn "UNVERIFIED" --exclude-dir=.git --exclude-dir=.deepseek .
```

Both should trend to zero. **The second reaching zero matters more than the first** — an
unimplemented function is visible to everyone, an unverified assumption is visible to nobody until
it costs a day. In this repository the sharpest example is risk V12: an unverified acados status
mapping is invisible right up until an infeasible solve is applied to a plant.

---

## §18 · Release criteria

The project is pre-release at `0.1.0` (`mpc_cbf_unified/package.xml`). Do **not** tag `1.0.0`
until the gate below holds. Keeping `0.1.0` while anything is open is the honest number.

**Gate for `1.0.0`:** all nine acceptance criteria green, in CI, on the hardware named in the
README.

### Release checklist (run before every tag)

1. A1–A9 green in CI on the README's named hardware.
2. All three media files exist and are reproducible from the notebooks
   ([12_ANALYSIS.md §12.6](12_ANALYSIS.md)).
3. `LICENSE` and `package.xml` both say BSD-3-Clause ([§2.6](02_ENVIRONMENT.md)); every source file carries
   the SPDX header.
4. `CITATION.cff` version matches `package.xml`.
5. Every version pin in [02_ENVIRONMENT.md](02_ENVIRONMENT.md) reflects what was actually built,
   not what was assumed — **including the acados tag**.
6. The derivations compile and their worked numbers match the code
   ([13_DOCS.md §13.4](13_DOCS.md)).
7. `REPRODUCTION_REPORT.md` has no `TBD`, and its Deviations section is populated.
8. The README quick-start runs verbatim from a clean clone.

If a criterion cannot be met, change it in [01_OVERVIEW.md §1.3](01_OVERVIEW.md) with a written
reason — do not weaken a test in place.
