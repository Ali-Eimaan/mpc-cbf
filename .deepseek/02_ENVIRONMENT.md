# §2 · Environment

Target platform: **Ubuntu 26.04 LTS + ROS 2 Lyrical Luth** (distro id `lyrical`), matching the
author's `cbf-safety-filter` toolchain so that one environment serves both repositories.

Pinned, and pinned in one place each:

| Component | Version | Pinned in |
| --- | --- | --- |
| Ubuntu | 26.04 LTS | container image `ros:lyrical-ros-base` |
| ROS 2 | Lyrical Luth (`lyrical`) | `ROS_DISTRO` env in `colcon_build.yml` |
| C++ | 20 (sources stay C++17-compatible, §16.7) | `CMAKE_CXX_STANDARD` in `CMakeLists.txt` |
| CMake | ≥ 3.22 (range `3.22...3.31`) | `cmake_minimum_required` |
| Eigen | 3.4 (system) | `package.xml` (`eigen`) |
| **acados** | **UNVERIFIED — pin the tag you build** | `colcon_build.yml`, *Build and install acados* |
| HPIPM / BLASFEO | bundled with acados (submodules) | same step |
| CasADi | ≥ 3.6.5, < 3.7 | `codegen/requirements.txt` |
| Python | ≥ 3.12, CI on 3.13 | `codegen/requirements.txt`, `colcon_build.yml` |
| gtest | via `ament_cmake_gtest` | `package.xml` |

---

## §2.1 Version risk register — read this before writing any CI

Lyrical Luth is a **young distro** and acados moves quickly. Several rows above are
forward-looking assumptions, not verified facts. **Resolve each row below first.** Each fails in
a way that wastes hours if you discover it midway through M8.

Rows V1–V5 carry over verified from the sibling repository; V9–V12 are new and specific to acados.

| # | Assumption | How it fails | Verify by |
| --- | --- | --- | --- |
| V1 | GitHub provides an `ubuntu-26.04` runner label | workflow never starts: "no runner matching labels" | **already mitigated** — the workflow uses `ubuntu-latest` plus a `ros:lyrical-*` container, which is what determines the build environment. Do not "fix" this by pinning the runner unless you have confirmed the label exists. |
| V2 | The `ros:lyrical-ros-base` image exists on Docker Hub | `docker pull` fails in every job | pull it locally once |
| V3 | REP 2000 specifies **C++20** for Lyrical | builds locally, then fails or warns on the official toolchain | read REP 2000. If it says C++17, change `CMAKE_CXX_STANDARD` to `17` — the sources use no C++20 feature, so it is a one-line change. Keep it that way (§16.7). |
| V4 | Ubuntu 26.04 ships CPython ≥ 3.13 | `requires-python` mismatch; linters target the wrong version | `python3 --version` in the container |
| V5 | `ament_lint_auto` / `ament_cmake_gtest` are published as debs for `lyrical` | `colcon test` fails at configure | `apt-cache search ros-lyrical-ament` inside the container |
| V6 | `ament_target_dependencies` is deprecated from Kilted onward | deprecation warnings, removal in a future distro | **verified in the sibling repo** — use `target_link_libraries` with namespaced targets and `${pkg_TARGETS}` ([§3.4](03_BUILD_SYSTEM.md)) |
| V7 | `ament_copyright` does not understand bare SPDX identifiers | copyright lint fails with `license=<unknown>` | **verified in the sibling repo** — see §2.6 |
| V9 | acados builds against the Ubuntu 26.04 toolchain | compile errors in the acados or BLASFEO C sources | build it once locally **before** relying on the workflow |
| V10 | The acados Python API names in [05_CODEGEN.md](05_CODEGEN.md) match your installed version | `AttributeError` mid-generation, or a silently ignored option | check `${ACADOS_SOURCE_DIR}/examples/acados_python/` for the version you installed, and correct the document |
| V11 | The acados C API names in [06_SOLVER.md §6.5](06_SOLVER.md) match | compile error, or worse, a parameter push that silently does nothing | check the generated `acados_solver_*.h` and `acados_c/ocp_nlp_interface.h` |
| V12 | acados solver return codes map as documented in [§6.5](06_SOLVER.md) | an infeasible solve reported as success — **this one is unsafe** | read `acados_solver_common.h` in your version and put the verified mapping in a comment |

Where a row turns out false, **fix the pin and update the table row to say what is actually
true.** Do not leave a stale assumption here — a version register nobody trusts is worse than no
register.

V12 deserves special care. Every other row costs you time; V12 costs you the safety property.

## §2.2 acados

acados is not available through rosdep. Build from source with a pinned tag:

```bash
git clone --recursive https://github.com/acados/acados.git
cd acados && mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_HPIPM=ON \
      -DCMAKE_INSTALL_PREFIX=$PWD/../install ..
make -j"$(nproc)" install
export ACADOS_SOURCE_DIR=$PWD/..
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
pip install -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
```

**Record the tag you actually built** here and in `codegen/requirements.txt`. `main` is not a
pin, and "whatever was current in August" is not reproducible.

Keep the workflow, this document and the root README identical — when they drift, the README is
wrong.

Three consequences of the acados dependency that shape the code:

1. **`c_generated_code/` is a build artefact, not source.** It is in `.gitignore`. Regenerate it;
   never commit it, and never hand-edit it.
2. **Generated code is configuration-specific.** Model, horizon, obstacle count and CBF variant
   are baked in; `γ`, the weights and the obstacle poses are runtime parameters. The matrix of
   configurations CI must generate is fixed in [05_CODEGEN.md §5.6](05_CODEGEN.md).
3. **Only `mpc_cbf_solver.cpp` and `tube_mpc_cbf_solver.cpp` include acados headers**, both
   behind a PIMPL, so a backend change touches two files ([06_SOLVER.md §6.1](06_SOLVER.md)).

## §2.3 Local setup

```bash
mkdir -p ~/ws/src && cd ~/ws/src && git clone <this-repo> mpc-cbf
```

The repository lives on a Windows machine. **Build and test inside WSL2 (Ubuntu 26.04) or the
`ros:lyrical-ros-base` container**, exactly as CI does. Do not attempt a native Windows ROS 2
build; nothing here has been validated for it, and acados' code generation invokes a C compiler
at runtime.

If `ros:lyrical-ros-base` is not yet published when you start, use `ros:rolling-ros-base` with
`ROS_DISTRO=rolling` for **local work only**. Do not commit that to the workflow.

### PEP 668

Ubuntu 26.04 marks the system interpreter externally-managed, so `pip install` into it is
refused outright. Create the venv with `--system-site-packages` so the ROS 2 Python modules stay
importable:

```bash
python3 -m venv --system-site-packages ~/.venvs/mpccbf && . ~/.venvs/mpccbf/bin/activate && pip install -r ~/ws/src/mpc-cbf/codegen/requirements.txt
```

**Do not use `--break-system-packages`.** It works right up until it corrupts a rosdep-installed
package, and then the failure presents as a ROS bug and costs a day to trace.

The CI python job is the one exception — it uses `actions/setup-python`, so PEP 668 never enters
the picture there.

## §2.4 Build and test commands

```bash
source /opt/ros/lyrical/setup.bash && source "${ACADOS_SOURCE_DIR}/env.sh"
```

```bash
python codegen/generate_mpc_cbf_solver.py --all && python codegen/generate_tube_solver.py --all
```

```bash
colcon build --packages-select mpc_cbf_unified --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

```bash
colcon test --packages-select mpc_cbf_unified --event-handlers console_direct+ && colcon test-result --verbose
```

**Codegen runs before the build, always.** A `colcon build` against a stale `c_generated_code/`
compiles happily and enforces last week's constraints.

**Always benchmark in Release.** A Debug solve time is meaningless and quoting one in the README
would be worse than quoting none.

## §2.5 ROS 2 API notes for Lyrical Luth

Changes since the Jazzy-era idiom that this codebase touches. Carried over verified from the
sibling repository. If you find another, add it here rather than fixing it silently in one file.

| Area | Old idiom | What this package does |
| --- | --- | --- |
| CMake dependency linkage | `ament_target_dependencies(tgt dep…)` | deprecated from Kilted onward; use `target_link_libraries` with namespaced targets and `${pkg_TARGETS}` — see [03_BUILD_SYSTEM.md §3.4](03_BUILD_SYSTEM.md) |
| CMake minimum | `3.16` | `3.22...3.31`; CMake 4.x warns on older compatibility levels |
| Python packaging | `license = { text = … }` | PEP 639 SPDX string, requires `setuptools>=77` |

> **Note on the distro choice.** `INFO.md` was written against ROS 2 Jazzy. Lyrical is used here
> because the author's sibling repository has already been migrated and validated on it, and
> running two repositories on two distros doubles the environment work for no benefit. Nothing in
> this package depends on a Lyrical-only feature — falling back to Jazzy means changing
> `ROS_DISTRO`, the container tag and the CMake minimum, and nothing else. If you make that
> change, record it here.

## §2.6 Licence

**BSD-3-Clause** (`LICENSE` at the repository root, `Copyright (c) 2026, Ali-Eimaan`) — the
conventional choice for ROS 2 packages, what downstream ROS users expect, and the same licence as
the author's `cbf-safety-filter`, so the two repositories can share code without a compatibility
question.

> The repository was created under MIT and relicensed to BSD-3-Clause on the author's instruction
> while still unpublished and single-author, so no third-party contribution needed re-consenting.
> If that ever stops being true, a further licence change needs every contributor's agreement.

Every source file starts with:

```
// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
```

(`#` comments in Python, after any shebang line.) **New files MUST carry this header.**

Note the third BSD clause: the copyright holder's name may not be used to endorse derived
products. That is a real obligation on downstream users and a reason to keep the copyright line
accurate rather than generic.

> **Why an SPDX line may not be enough.** `ament_copyright` matches file content against full
> licence-text templates rather than parsing SPDX identifiers:
> `SourceDescriptor.identify_license()` compares against the templates in
> `ament_copyright/licenses.py`, so a header carrying only a copyright line and an SPDX id is
> reported as `license=<unknown>` and fails the copyright lint. This was **verified on Lyrical in
> the sibling repository** (risk V7), and it applies directly here now that both use
> BSD-3-Clause.
>
> Two routes, and the skeleton currently takes the second:
>
> 1. Append the short `bsd_3clause` `file_headers` template text below the SPDX line. **Copy it
>    out of the installed `ament_copyright/licenses.py`; do not retype it from memory** (rule 4,
>    [00_RULES.md](00_RULES.md)) — the match is textual, so an approximation fails silently.
> 2. Disable the copyright linter in `CMakeLists.txt` with a comment saying why —
>    `set(ament_cmake_copyright_FOUND TRUE)`, which is what the skeleton does. A deliberate
>    choice for a single-author research repository, not an oversight.
>
> Run `ament_copyright mpc_cbf_unified` before changing which route is in force. If a future
> distro ships an `ament_copyright` that understands SPDX identifiers, prefer route 1 without the
> template text and re-enable the linter.

`package.xml` declares `<license>BSD-3-Clause</license>`; it and `LICENSE` MUST agree.
