#!/usr/bin/env python3
"""acados code generation for the tube-MPC-CBF nominal solver.

Exactly three differences from generate_mpc_cbf_solver.py (.deepseek/05_CODEGEN.md §5.7);
everything else is shared and imported, never copied:
  1. decision variables are the *nominal* (z, v), not the true (x, u),
  2. the per-stage parameter vector is extended by one tightening scalar
     c_{j,k} per obstacle:  np = 7*n_obstacles + 1 + n_obstacles  (65 for 8),
  3. the barrier rows become (08_TUBE.md §8.4 — read the derivation before
     touching the sign):
         distance:  h_j(z_k) - c_{j,k}                          >= 0
         DCBF:      h_j(F(z_k,v_k)) - h_j(z_k) + gamma*(h_j(z_k) - c_{j,k}) >= 0
     with the tightening entering BOTH rows. c_{j,k} is filled at runtime by
     TubeMpcCbfSolver::tighteningFor(); the generator ships c = 0 defaults.

Bounds arrive ALREADY TIGHTENED from the caller. This script must never
compute X (-) Omega or U (-) K Omega — the C++ TubeMpcCbfSolver::initialize()
is the single source of truth for the tightening, and recomputing it from a
second implementation is how the two end up disagreeing.

z_{k+1} in the DCBF row is obtained by substituting the discrete dynamics
F(z_k, v_k) into the barrier expression, exactly as the nominal generator does
(05_CODEGEN.md §5.4 step 7).

Usage:
    python generate_tube_solver.py --model double_integrator_2d \
        --horizon 8 --dt 0.1 --n-obstacles 8 --tighten-mode support_function
    python generate_tube_solver.py --all      # both tube configurations CI needs
    python generate_tube_solver.py --print-reference-supports
        # reproduces the hardcoded reference list in
        # test_tube_mpc_robustness.cpp RpiMatchesPythonReference
"""

from __future__ import annotations

import argparse
import itertools
import os
import re

import casadi as ca
import numpy as np
from scipy.linalg import solve_discrete_are

try:
    from models import (
        MODEL_REGISTRY,
        barrier_expression,
        discretise,
        linearised_discrete,
    )
except ImportError:  # Imported as codegen.generate_tube_solver from the repo root.
    from codegen.models import (
        MODEL_REGISTRY,
        barrier_expression,
        discretise,
        linearised_discrete,
    )

# Reused, not copied (§5.7): the acados classes (with the lazy-unavailable
# stub), the parameter-layout constants, the large finite bound, and the
# default weights. The nominal solver_name/parse_solver_name are NOT reused:
# tube names end in the tighten mode, not a variant.
try:
    from generate_mpc_cbf_solver import (
        CONSTRAINT_UB,
        DEFAULT_OUTPUT_DIR,
        _ACADOS_AVAILABLE,
        AcadosModel,
        AcadosOcp,
        AcadosOcpSolver,
        default_weights,
        gamma_param_offset,
    )
except ImportError:  # Imported as codegen.generate_tube_solver from the repo root.
    from codegen.generate_mpc_cbf_solver import (
        CONSTRAINT_UB,
        DEFAULT_OUTPUT_DIR,
        _ACADOS_AVAILABLE,
        AcadosModel,
        AcadosOcp,
        AcadosOcpSolver,
        default_weights,
        gamma_param_offset,
    )

TUBE_TIGHTEN_MODES = ("support_function", "lipschitz", "none")
RPI_GENERATOR_CAP = 256  # mirrors kRpiGeneratorCap in tube_mpc_cbf_solver.cpp


def tube_param_count(n_obstacles: int) -> int:
    """Per-stage parameter count: obstacle block + gamma + one c per obstacle."""
    return 7 * n_obstacles + 1 + n_obstacles


def tightening_offset(n_obstacles: int) -> int:
    """Index of the first tightening scalar in the stage parameter vector."""
    return 7 * n_obstacles + 1


def tube_solver_name(model_key: str, horizon: int, tighten_mode: str) -> str:
    """tube_mpc_cbf_{model}_N{horizon}_{tighten_mode} — the tighten mode is the
    suffix, not a variant, so the nominal solver_name is not applicable."""
    if tighten_mode not in TUBE_TIGHTEN_MODES:
        raise ValueError(f"tube_solver_name: unknown tighten mode {tighten_mode!r}")
    return f"tube_mpc_cbf_{model_key}_N{horizon}_{tighten_mode}"


def parse_tube_solver_name(name: str) -> tuple[str, int, str]:
    """Inverse of tube_solver_name -> (model_key, horizon, tighten_mode)."""
    match = re.fullmatch(
        r"tube_mpc_cbf_(.+)_N(\d+)_(support_function|lipschitz|none)", name
    )
    if match is None:
        raise ValueError(f"parse_tube_solver_name: not a tube solver name: {name!r}")
    return match.group(1), int(match.group(2)), match.group(3)


def _coerce_model_key(model_name: str) -> str:
    """Accept either a bare model key or a full tube solver name."""
    try:
        return parse_tube_solver_name(model_name)[0]
    except ValueError:
        return model_name


def _girard_reduce(generators: np.ndarray, max_generators: int) -> np.ndarray:
    """Girard's order reduction (07_SETS.md §7.4): keep the max_generators - n
    generators with the largest ||g||_1 - ||g||_inf, replace the rest by the
    diagonal box of their absolute row sums. Never shrinks the set."""
    n, m = generators.shape
    if m <= max_generators:
        return generators
    scores = np.abs(generators).sum(axis=0) - np.abs(generators).max(axis=0)
    keep = np.argsort(scores)[::-1][: max_generators - n]
    dropped = np.delete(generators, keep, axis=1)
    box = np.abs(dropped).sum(axis=1)
    return np.hstack([generators[:, keep], np.diag(box)])


def _rakovic_rpi(
    A_cl: np.ndarray,
    half_widths: np.ndarray,
    epsilon: float = 1.0e-3,
    max_iterations: int = 100,
) -> tuple[float, int, np.ndarray]:
    """Raković mRPI iteration (07_SETS.md §7.5), mirroring
    computeRpiSet() in tube_mpc_cbf_solver.cpp: no in-loop invariance
    certificate, break on  alpha/(1-alpha) * M(s) <= epsilon.

    W is the axis-aligned box with the given half-widths, F_s accumulates the
    generators of the partial sum, and the returned generator matrix is
    F_s / (1 - alpha) — a zonotope parametrised by ||z||_inf <= 1.

    Returns (alpha, s, Omega_generators)."""
    n = A_cl.shape[0]
    w = np.asarray(half_widths, dtype=float).reshape(n)
    w_gen = np.diag(w)

    def h_w(d: np.ndarray) -> float:
        """Support function of W along d: sum_j w_j |d_j| for the box."""
        return float(np.sum(w * np.abs(d)))

    f_gens = np.zeros((n, 0))
    a_prev = np.eye(n)  # A_cl^{s-1}
    m_sum = np.zeros(n)
    for s in range(1, max_iterations + 1):
        f_gens = np.hstack([f_gens, a_prev @ w_gen])
        if f_gens.shape[1] > RPI_GENERATOR_CAP:
            f_gens = _girard_reduce(f_gens, RPI_GENERATOR_CAP)
        for i in range(n):
            a = a_prev.T[:, i]  # A_cl^{s-1}^T e_i
            m_sum[i] += h_w(a) + h_w(-a)
        a_pow = a_prev @ A_cl  # A_cl^s
        alpha = 0.0
        alpha_finite = True
        for i in range(n):
            e_i = np.eye(n)[:, i]
            hw_e = h_w(e_i)
            hw_d = h_w(a_pow.T @ e_i)
            if hw_e > 1e-14:
                alpha = max(alpha, hw_d / hw_e)
            elif hw_d > 1e-14:  # degenerate axis; RPI not certified
                alpha_finite = False
        if not alpha_finite or alpha >= 1.0:
            a_prev = a_pow
            continue
        if alpha / (1.0 - alpha) * float(m_sum.max()) <= epsilon:
            return alpha, s, f_gens / (1.0 - alpha)
        a_prev = a_pow
    raise RuntimeError(
        f"_rakovic_rpi: no convergence within {max_iterations} iterations"
    )


def compute_offline_sets(
    model_name: str,
    dt: float,
    disturbance_box,
    lqr_q,
    lqr_r,
    epsilon: float = 1.0e-3,
    max_iterations: int = 100,
):
    """Reference (Python) implementation of the offline tube computation.

    Deliberately INDEPENDENT of the C++ path: the LQR gain comes from
    scipy.linalg.solve_discrete_are (the C++ side iterates the Riccati
    recursion) and the RPI iteration is plain numpy. The gtest
    RpiMatchesPythonReference asserts the two agree on Omega's support in 32
    directions to 1e-6 — two implementations of different algorithms agreeing
    proves more than two implementations of the same algorithm agreeing.
    Do not let the two drift.

    `disturbance_box` is the vector of half-widths of the axis-aligned box W.

    Returns (K, Omega_vertices, alpha, s):
      K              the discrete LQR gain, u = K z (n x nu),
      Omega_vertices corners of the axis-aligned hull of Omega. Omega is a
                     centrally symmetric zonotope; for axis-aligned W (the
                     case this cross-check is defined for) the hull is exact,
                     for non-box W it would be an over-approximation,
      alpha          the contraction factor of the last iteration,
      s              the iteration count the stopping rule fired at.
    """
    model_key = _coerce_model_key(model_name)
    spec = MODEL_REGISTRY[model_key]()
    n = spec.nx

    A, B = linearised_discrete(spec, dt)  # exact ZOH for the double integrator
    Q = np.diag(np.asarray(lqr_q, dtype=float))
    R = np.diag(np.asarray(lqr_r, dtype=float))
    P = solve_discrete_are(A, B, Q, R)
    K = -np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    A_cl = A + B @ K
    rho = float(max(abs(np.linalg.eigvals(A_cl))))
    if rho >= 1.0 - 1e-12:
        raise ValueError(
            f"compute_offline_sets: A + B K not Schur (spectral radius {rho})"
        )

    w = np.asarray(disturbance_box, dtype=float).reshape(n)
    alpha, s, omega_gens = _rakovic_rpi(A_cl, w, epsilon, max_iterations)

    # Omega = {G z : ||z||_inf <= 1} is centrally symmetric; for axis-aligned W
    # it is exactly a box, so the corners of the axis hull are its vertices.
    half = np.abs(omega_gens).sum(axis=1)
    vertices = np.array(
        [half * np.array(signs) for signs in itertools.product((-1.0, 1.0), repeat=n)]
    )
    return K, vertices, alpha, s


def _tube_distance_rows(spec, n_obstacles: int, z, p):
    """h_j(z) - c_j >= 0, obstacle-major. Used at stage N (no v_N for a DCBF
    row); stages 0..N-1 carry _tube_barrier_rows (distance + DCBF)."""
    rows = []
    for j in range(n_obstacles):
        c_j = p[tightening_offset(n_obstacles) + j]
        obs = p[7 * j: 7 * (j + 1)]
        rows.append(barrier_expression(spec, z, obs) - c_j)
    return ca.vertcat(*rows)


def _tube_barrier_rows(spec, n_obstacles: int, z, v, p, dt: float):
    """Distance rows (with c) followed by DCBF rows (with c), obstacle-major
    within each type — so SolverDiagnostics::cbf_values indexes k*n_obs + j.

    The DCBF row substitutes F(z_k, v_k) for z_{k+1} — the same choice as the
    nominal generator (05_CODEGEN.md §5.4 step 7)."""
    gamma = p[gamma_param_offset(n_obstacles)]
    method = "exact" if spec.name == "double_integrator_2d" else "rk4"
    F = discretise(spec, dt, method)
    obs = [p[7 * j: 7 * (j + 1)] for j in range(n_obstacles)]
    c_j = [p[tightening_offset(n_obstacles) + j] for j in range(n_obstacles)]
    h_now = [barrier_expression(spec, z, o) for o in obs]
    h_next = [barrier_expression(spec, F(z, v), o) for o in obs]
    distance_rows = [h_now[j] - c_j[j] for j in range(n_obstacles)]
    dcbf_rows = [h_next[j] - h_now[j] + gamma * (h_now[j] - c_j[j]) for j in range(n_obstacles)]
    return ca.vertcat(*(distance_rows + dcbf_rows))


def build_tube_acados_model(model_name: str, dt: float, n_obstacles: int):
    """Nominal (z, v) model with the extended parameter vector.

    All p entries are CASADI SYMBOLS — bounds, obstacle parameters and
    tightenings are runtime quantities (TubeMpcCbfSolver::initialize/solve),
    never codegen constants."""
    model_key, _, _ = parse_tube_solver_name(model_name)
    spec = MODEL_REGISTRY[model_key]()
    z = ca.SX.sym("z", spec.nx)
    v = ca.SX.sym("v", spec.nu)
    p = ca.SX.sym("p", tube_param_count(n_obstacles))
    method = "exact" if spec.name == "double_integrator_2d" else "rk4"
    F = discretise(spec, dt, method)

    model = AcadosModel()
    model.name = model_name
    model.x = z
    model.u = v
    model.p = p
    model.disc_dyn_expr = F(z, v)
    # Stage 0 carries the same rows as every interior stage: with z_0 fixed
    # by the initial-state equality, the DCBF row constrains v_0 — the step
    # actually applied to the plant (10_TESTS.md A3; the nominal generator
    # makes the identical choice). The distance row h_j(z_0) - c_j >= 0 is
    # what makes an unsafe start surface as infeasible.
    model.con_h_expr_0 = _tube_barrier_rows(spec, n_obstacles, z, v, p, dt)
    return model


def build_tube_ocp(
    model_name: str = "double_integrator_2d",
    horizon: int = 8,
    dt: float = 0.1,
    n_obstacles: int = 8,
    tighten_mode: str = "support_function",
    use_rti: bool = False,
    max_sqp_iterations: int = 20,
):
    """Return the AcadosOcp for the nominal tube problem.

    Identical cost structure and solver options to the nominal generator; the
    three §5.7 differences live in build_tube_acados_model and the row
    expressions. Bounds are the YAML defaults — tightening is the C++ side's
    job, and it overwrites them from initialize() at runtime."""
    if tighten_mode not in TUBE_TIGHTEN_MODES:
        raise ValueError(f"build_tube_ocp: unknown tighten mode {tighten_mode!r}")
    if n_obstacles < 1:
        raise ValueError(f"build_tube_ocp: n_obstacles must be >= 1, got {n_obstacles}")
    if not _ACADOS_AVAILABLE:
        raise ImportError("acados_template is not installed — cannot build the OCP")
    model_key = _coerce_model_key(model_name)
    spec = MODEL_REGISTRY[model_key]()
    full_name = tube_solver_name(model_key, horizon, tighten_mode)

    ocp = AcadosOcp()
    ocp.model = build_tube_acados_model(full_name, dt, n_obstacles)
    ocp.dims.N = horizon

    # Cost: y = [z; v], W = blkdiag(Q, R); terminal y_e = z, W_e = Qf.
    q, r, qf = default_weights(spec)
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(ocp.model.x, ocp.model.u)
    ocp.model.cost_y_expr_e = ocp.model.x
    ocp.cost.W = np.diag(np.concatenate([q, r]))
    ocp.cost.W_e = np.diag(qf)
    ocp.cost.yref = np.zeros(spec.nx + spec.nu)
    ocp.cost.yref_e = np.zeros(spec.nx)

    # Bounds: input limits only; the state bounds arrive tightened at runtime
    # (initialize() sets the stage-0 equality via idxbx/lbx/ubx and the state
    # limits via the same mechanism for the remaining stages).
    ocp.constraints.idxbu = np.arange(spec.nu)
    ocp.constraints.lbu = -np.ones(spec.nu)
    ocp.constraints.ubu = np.ones(spec.nu)
    ocp.constraints.x0 = np.zeros(spec.nx)

    # Stage 0 carries distance + DCBF rows (the applied-step DCBF, A3); stage
    # N carries distance rows only — no v_N exists for a DCBF row. Tightening
    # enters both row types.
    ocp.model.con_h_expr = _tube_barrier_rows(
        spec, n_obstacles, ocp.model.x, ocp.model.u, ocp.model.p, dt
    )
    ocp.model.con_h_expr_e = _tube_distance_rows(
        spec, n_obstacles, ocp.model.x, ocp.model.p
    )
    ocp.constraints.lh = np.zeros(2 * n_obstacles)
    ocp.constraints.uh = CONSTRAINT_UB * np.ones(2 * n_obstacles)
    ocp.constraints.lh_0 = np.zeros(2 * n_obstacles)
    ocp.constraints.uh_0 = CONSTRAINT_UB * np.ones(2 * n_obstacles)
    ocp.constraints.lh_e = np.zeros(n_obstacles)
    ocp.constraints.uh_e = CONSTRAINT_UB * np.ones(n_obstacles)

    # Per-stage parameter defaults: dummy obstacles parked at (1e6, 1e6),
    # radius 0, gamma 0.3, tightenings zero (kNone behaviour). The C++
    # solver overwrites every entry before each solve.
    n_p = tube_param_count(n_obstacles)
    p_default = np.zeros(n_p)
    for j in range(n_obstacles):
        p_default[7 * j: 7 * j + 3] = 1.0e6
        p_default[7 * j + 6] = 0.0
    p_default[gamma_param_offset(n_obstacles)] = 0.3
    ocp.parameter_values = p_default

    # Solver options, identical to the nominal generator (§5.4 step 8):
    # DISCRETE dynamics (never ERK), SQP or SQP_RTI, Levenberg-Marquardt for
    # the DCBF nonlinearities.
    ocp.solver_options.tf = horizon * dt  # stage duration = tf/N = dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI" if use_rti else "SQP"
    ocp.solver_options.nlp_solver_max_iter = max_sqp_iterations
    # LM regularization: 1e-4 leaves HPIPM's Newton system ill-conditioned on
    # degenerate CBF QPs (spurious ACADOS_MINSTEP). 1e-2 matches
    # mpc_cbf_solver.cpp and generate_mpc_cbf_solver.py (05_CODEGEN.md §5.4).
    ocp.solver_options.levenberg_marquardt = 1.0e-2
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0
    # NLP termination tolerances, identical to the nominal generator (§5.4
    # step 8): 1e-3 for all four, matching the C++ runtime override in
    # tube_mpc_cbf_solver.cpp. See generate_mpc_cbf_solver.py for the
    # rationale (grazing-point res_stat/res_comp floors make the acados 1e-6
    # defaults unreachable, which stalls HPIPM and diverges the SQP; safety is
    # checked on the returned trajectory, not residuals).
    ocp.solver_options.nlp_solver_tol_stat = 1.0e-3
    ocp.solver_options.nlp_solver_tol_eq = 1.0e-3
    ocp.solver_options.nlp_solver_tol_ineq = 1.0e-3
    ocp.solver_options.nlp_solver_tol_comp = 1.0e-3
    return ocp


def generate(output_dir: str = DEFAULT_OUTPUT_DIR, **kwargs) -> str:
    """Code-generate one tube solver; prints its name to stdout."""
    ocp = build_tube_ocp(**kwargs)
    solver = ocp.model.name
    out = os.path.join(output_dir, solver)
    os.makedirs(out, exist_ok=True)
    json_file = os.path.join(out, f"{solver}.json")
    # acados >= 0.5.6: stable ocp.name + explicit export location (see the
    # nominal generator for the rationale).
    ocp.name = solver
    ocp.code_gen_options.json_file = json_file
    ocp.code_gen_options.code_export_directory = out
    AcadosOcpSolver(ocp, generate=True, build=True)
    generated_c = os.path.join(out, f"acados_solver_{solver}.c")
    if not os.path.isfile(generated_c):
        raise RuntimeError(
            f"generate: acados did not produce {generated_c} for {solver}"
        )
    print(solver)
    return out


def generate_all(output_dir: str = DEFAULT_OUTPUT_DIR) -> list[str]:
    """The three tube configurations CI needs: the default (support_function),
    the lipschitz variant pinned by SupportTighteningDominatesLipschitz, and
    the tighten_mode="none" ablation used by the robustness sweep."""
    matrix = [
        ("double_integrator_2d", 8, "support_function"),  # tube default (§8.4)
        ("double_integrator_2d", 8, "lipschitz"),         # §8.4 ordering test
        ("double_integrator_2d", 8, "none"),              # A8 ablation
    ]
    names = []
    for model_key, horizon, tighten_mode in matrix:
        generate(
            output_dir,
            model_name=model_key,
            horizon=horizon,
            dt=0.1,
            n_obstacles=8,
            tighten_mode=tighten_mode,
        )
        names.append(tube_solver_name(model_key, horizon, tighten_mode))
    return names


def fixture_reference_supports() -> list[tuple[np.ndarray, float]]:
    """32 (direction, support) pairs of the fixture Omega for
    RpiMatchesPythonReference: 8 axis directions plus 24 random unit vectors
    from np.random.default_rng(0x5EED). The gtest hardcodes this list; this
    function is the script that produces it — keep the two in lockstep."""
    n = 4
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    dt = 0.1
    w = np.array([0.005, 0.005, 0.02, 0.02])
    A, B = linearised_discrete(spec, dt)
    Q = np.diag([10.0, 10.0, 1.0, 1.0])
    R = np.diag([1.0, 1.0])
    P = solve_discrete_are(A, B, Q, R)
    K = -np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    _, _, omega_gens = _rakovic_rpi(A + B @ K, w)

    directions = []
    for i in range(n):
        directions.append(np.eye(n)[:, i])
        directions.append(-np.eye(n)[:, i])
    rng = np.random.default_rng(0x5EED)
    for _ in range(24):
        v = rng.normal(size=n)
        directions.append(v / np.linalg.norm(v))

    pairs = []
    for d in directions:
        support = float(np.abs(omega_gens.T @ d).sum())  # d^T c + ||G^T d||_1, c = 0
        pairs.append((d, support))
    return pairs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="acados code generation for the tube-MPC-CBF nominal solver"
    )
    parser.add_argument(
        "--model",
        default="double_integrator_2d",
        choices=tuple(MODEL_REGISTRY),
        help="model key from models.py (default: %(default)s)",
    )
    parser.add_argument("--horizon", type=int, default=8, help="MPC horizon N")
    parser.add_argument("--dt", type=float, default=0.1, help="sampling period")
    parser.add_argument("--n-obstacles", type=int, default=8, help="number of obstacles")
    parser.add_argument(
        "--tighten-mode",
        default="support_function",
        choices=TUBE_TIGHTEN_MODES,
        help="tube tightening mode (default: %(default)s)",
    )
    parser.add_argument(
        "--rti", action="store_true", help="generate an RTI (real-time iteration) solver"
    )
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--all",
        action="store_true",
        help="generate the full tube matrix, ignoring the per-configuration flags",
    )
    parser.add_argument(
        "--print-reference-supports",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def _fmt_component(x: float) -> str:
    x = float(x)
    if abs(x - round(x)) < 1e-12:
        return str(int(round(x)))
    return f"{x:.17g}"


def main() -> int:
    args = parse_args()
    if args.print_reference_supports:
        for d, val in fixture_reference_supports():
            inner = ", ".join(_fmt_component(v) for v in d)
            print(f"{{{{{inner}}}, {val:.17g}}},")
        return 0
    if args.all:
        generate_all(args.output_dir)
    else:
        generate(
            args.output_dir,
            model_name=args.model,
            horizon=args.horizon,
            dt=args.dt,
            n_obstacles=args.n_obstacles,
            tighten_mode=args.tighten_mode,
            use_rti=args.rti,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
