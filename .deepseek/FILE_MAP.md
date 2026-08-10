# File map

Every skeleton file in the repository, and the document that specifies it. Use this when you have
a file open and need its spec.

---

## Solver library — `mpc_cbf_unified/` (no ROS messages, no node)

| File | Spec | Milestone |
| --- | --- | --- |
| `include/mpc_cbf_unified/mpc_cbf_solver.hpp` · `src/mpc_cbf_solver.cpp` | [06_SOLVER.md](06_SOLVER.md) | M3, M4 |
| `include/mpc_cbf_unified/disturbance_sets.hpp` | [07_SETS.md](07_SETS.md) | M6 |
| `include/mpc_cbf_unified/tube_mpc_cbf_solver.hpp` · `src/tube_mpc_cbf_solver.cpp` | [08_TUBE.md](08_TUBE.md) (also hosts §7's implementations) | M6, M7 |

`src/disturbance_sets.cpp` does not exist yet by design — split it out of the tube translation
unit when that file passes ~600 lines ([03_BUILD_SYSTEM.md §3.1](03_BUILD_SYSTEM.md)).

## ROS layer

| File | Spec | Milestone |
| --- | --- | --- |
| `include/mpc_cbf_unified/mpc_cbf_node.hpp` · `src/mpc_cbf_node.cpp` | [09_NODE.md §9.1–§9.5](09_NODE.md) | M8 |
| `scripts/sim_double_integrator.py` · `sim_bicycle.py` · `sim_quadrotor_planar.py` · `track_reference.py` · `log_min_barrier.py` | [09_NODE.md §9.6](09_NODE.md) — **do not exist yet, you create them** | M8 |

## Code generation — `codegen/`

| File | Spec | Milestone |
| --- | --- | --- |
| `models.py` | [04_MODELS.md](04_MODELS.md) | M1 |
| `generate_mpc_cbf_solver.py` | [05_CODEGEN.md §5.1–§5.6](05_CODEGEN.md) | M2, M4 |
| `generate_tube_solver.py` | [05_CODEGEN.md §5.7](05_CODEGEN.md) | M7 |
| `requirements.txt` | [02_ENVIRONMENT.md §2.2](02_ENVIRONMENT.md) — pin the acados tag here | M2 |
| `tests/test_models.py` | [04_MODELS.md §4.6](04_MODELS.md) — **does not exist yet** | M1 |

`codegen/c_generated_code/` is a build artefact. Never commit it, never hand-edit it
([02_ENVIRONMENT.md §2.2](02_ENVIRONMENT.md)).

## Build files (complete — keep them correct, they have no TODOs)

| File | Spec |
| --- | --- |
| `mpc_cbf_unified/CMakeLists.txt` | [03_BUILD_SYSTEM.md](03_BUILD_SYSTEM.md) — §3.3 is the block you edit at M2 |
| `mpc_cbf_unified/package.xml` | [03_BUILD_SYSTEM.md §3.6](03_BUILD_SYSTEM.md) |
| `.gitignore` | complete |

## Configuration — [09_NODE.md §9.2](09_NODE.md), M8

| File | Section |
| --- | --- |
| `config/mpc_cbf_params.yaml` | §9.2 |
| `config/tube_mpc_params.yaml` | §9.2, [08_TUBE.md §8.3](08_TUBE.md) |

The YAML keys are the contract with `declareParameters()`. Change them in the same commit or not
at all.

## Launch — [09_NODE.md §9.7](09_NODE.md), M8

| File |
| --- |
| `launch/2d_obstacle.launch.py` |
| `launch/car_racing.launch.py` |
| `launch/quadrotor_dynamic_obstacle.launch.py` |
| `launch/tube_mpc_disturbance.launch.py` |

## Tests — [10_TESTS.md](10_TESTS.md)

| File | Section | Milestone |
| --- | --- | --- |
| `test/test_mpc_cbf_feasibility.cpp` | §10.1 | M3, M4 |
| `test/test_tube_mpc_robustness.cpp` | §10.2 (sets M6, tube M7) | M6, M7 |
| `test/test_recursive_feasibility.py` | [11_PYTHON_REFERENCE.md](11_PYTHON_REFERENCE.md) | M5 |
| `test/cpp_solve_cli.cpp` | [11_PYTHON_REFERENCE.md §11.5](11_PYTHON_REFERENCE.md) — **does not exist yet** | M5 |

Adding a test file means adding its `ament_add_gtest` block
([03_BUILD_SYSTEM.md §3.8](03_BUILD_SYSTEM.md)). Forgetting this is silent.

## Reproductions and analysis — [12_ANALYSIS.md](12_ANALYSIS.md), M10

| File | Section |
| --- | --- |
| `reproduction/zeng_acc2021/reproduce_acc2021.ipynb` | §12.1 |
| `reproduction/zeng_cdc2021/reproduce_cdc2021.ipynb` | §12.2 |
| `reproduction/REPRODUCTION_REPORT.md` | [13_DOCS.md §13.5](13_DOCS.md) |
| `analysis/cbfqp_vs_mpccbf_comparison.ipynb` | §12.3 |
| `analysis/feasibility_recovery_study.ipynb` | §12.4 |
| `analysis/disturbance_robustness_sweep.ipynb` | §12.5 |
| `media/README.md` | §12.6 |

## Documentation — [13_DOCS.md](13_DOCS.md), M10

| File | Section |
| --- | --- |
| `docs/README_math.md` | §13.1 |
| `docs/derivations/discrete_time_cbf.tex` | §13.2 |
| `docs/derivations/mpc_cbf_unified_formulation.tex` | §13.2 |
| `docs/derivations/tube_mpc_robustness.tex` | §13.2 |
| `docs/PRIOR_ART.md` | §13.3 |

## CI — [14_CI.md](14_CI.md), M9

| File | Section |
| --- | --- |
| `.github/workflows/colcon_build.yml` | §14.1–§14.7 |

## Repository root

| File | Notes |
| --- | --- |
| `README.md` | [13_DOCS.md §13.6](13_DOCS.md) — fill the results table from the test suite; badges only once the workflow is green |
| `LICENSE` | BSD-3-Clause. Complete ([02_ENVIRONMENT.md §2.6](02_ENVIRONMENT.md)) |

`INFO.md` and `IMPLEMENTATION_GUIDE.md` used to live at the root. They now live in this directory
([INFO.md](INFO.md), [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)) so that there is exactly
one specification rather than two that can drift apart.
