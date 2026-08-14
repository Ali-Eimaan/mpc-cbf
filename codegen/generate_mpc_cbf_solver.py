#!/usr/bin/env python3
"""acados code generation for the nominal MPC-CBF solver.

Emits C code consumed by mpc_cbf_unified/CMakeLists.txt. Regenerate whenever
the model, the horizon, the obstacle count or the CBF variant changes; gamma,
the weights and the obstacle poses are runtime *parameters* and do NOT require
regeneration.

Usage:
    python generate_mpc_cbf_solver.py --model double_integrator_2d \
        --horizon 8 --dt 0.1 --variant fixed_decay --n-obstacles 8
    python generate_mpc_cbf_solver.py --all      # every combination CI needs

Choice of x_{k+1} in the DCBF row:
we SUBSTITUTE the discrete dynamics F(x_k, u_k) into the barrier expression,
so the row h(F(x_k,u_k)) - h(x_k) + gamma*h(x_k) >= 0 is a function of stage
k's (x_k, u_k) only. The alternative (constraining the next shooting node's
state) would need cross-stage expressions acados does not expose through
con_h_expr. Use this same choice everywhere; mixing the two silently changes
what the constraint means.
"""

from __future__ import annotations

import argparse
import os
import re

import casadi as ca
import numpy as np

try:
    from models import MODEL_REGISTRY, barrier_expression, dcbf_constraint, discretise
except ImportError:  # Imported as codegen.generate_mpc_cbf_solver from the repo root.
    from codegen.models import MODEL_REGISTRY, barrier_expression, dcbf_constraint, discretise

# acados_template ships with the acados source tree, not PyPI. Code generation
# needs it, but the naming/row-assembly helpers below do not — keep the import
# optional so tests of those helpers run on machines without acados.
try:
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    _ACADOS_AVAILABLE = True
except ImportError:

    class _Unavailable:
        def __init__(self, *args, **kwargs):
            raise ImportError(
                "acados_template is not installed. Source <acados>/env.sh and "
                "`pip install -e ${ACADOS_SOURCE_DIR}/interfaces/acados_template` "
                "before generating solvers."
            )

    AcadosModel = AcadosOcp = AcadosOcpSolver = _Unavailable
    _ACADOS_AVAILABLE = False

DEFAULT_OUTPUT_DIR = "c_generated_code"

# Per-obstacle parameter layout (one slice per stage):
#   [ox, oy, oz, vx, vy, vz, radius]          7 doubles
#   ... repeated for n_obstacles
#   gamma                                     +1  (np = 7*n_obs + 1)
#   (tube only) c_j per obstacle, per stage   +n_obs
OBS_PARAMS_PER_OBSTACLE = 7


def gamma_param_offset(n_obstacles: int) -> int:
    return OBS_PARAMS_PER_OBSTACLE * n_obstacles


# Large finite upper bound — never np.inf.
# Must exceed the squared-distance barrier at the far-away dummy obstacles
# h ~ 2e12 (position 1e6); a bound of 1e9 makes the initial iterate
# infeasible (res_ineq ~ 2e12) and the QP dies with HPIPM_MINSTEP.
CONSTRAINT_UB = 1.0e13


VARIANTS = ("fixed_decay", "relaxed_decay", "distance_only")


def solver_name(model_key: str, horizon: int, variant: str, tube: bool = False) -> str:
    """The generated-solver name; the C++ side selects a solver by this string."""
    prefix = "tube_mpc_cbf_" if tube else "mpc_cbf_"
    return f"{prefix}{model_key}_N{horizon}_{variant}"


def parse_solver_name(name: str) -> tuple[str, int, str, bool]:
    """Inverse of solver_name(): (model_key, horizon, variant, tube)."""
    m = re.fullmatch(
        r"(mpc_cbf|tube_mpc_cbf)_(.+)_N(\d+)_(fixed_decay|relaxed_decay|distance_only)", name
    )
    if m is None:
        raise ValueError(f"unrecognised solver name {name!r}")
    return m.group(2), int(m.group(3)), m.group(4), m.group(1) == "tube_mpc_cbf"


def default_weights(spec) -> tuple[list[float], list[float], list[float]]:
    """YAML-default-style weights: 10 on the position states, 1 elsewhere;
    Qf = 10*Q; R = identity. Runtime-settable through the solver API."""
    q = [10.0 if i in spec.position_indices else 1.0 for i in range(spec.nx)]
    qf = [10.0 * qi for qi in q]
    r = [1.0] * spec.nu
    return q, r, qf


def build_acados_model(model_name: str, dt: float, n_obstacles: int) -> AcadosModel:
    """Assemble the AcadosModel: symbolic state, input, parameters, discrete dynamics.

    * x, u from the ModelSpec; p = obstacle block (7 entries per obstacle) +
      [gamma] — the layout is documented in one place and mirrored by
      the C++ side (MpcCbfSolver::Impl).
    * model.disc_dyn_expr = discretise(spec, dt) so acados uses the DISCRETE
      integrator type; an ERK integrator here would silently change the CBF
      semantics.
    * model.name encodes the configuration, e.g.
      "mpc_cbf_double_integrator_2d_N8_fixed_decay".
    """
    model_key, horizon, variant, _ = parse_solver_name(model_name)
    spec = MODEL_REGISTRY[model_key]()

    x = ca.SX.sym("x", spec.nx)
    u_phys = ca.SX.sym("u", spec.nu)
    if variant == "relaxed_decay":
        omega = ca.SX.sym("omega", n_obstacles)
        u = ca.vertcat(u_phys, omega)
    else:
        u = u_phys

    n_p = OBS_PARAMS_PER_OBSTACLE * n_obstacles + 1
    p = ca.SX.sym("p", n_p)

    # The double integrator has an exact ZOH discretisation; use it so the
    # generated solver and the tube's (A, B) agree bit-for-bit on the dynamics.
    method = "exact" if spec.name == "double_integrator_2d" else "rk4"
    F = discretise(spec, dt, method)
    disc_dyn_expr = F(x, u_phys)  # dynamics depend on the physical inputs only

    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.u = u
    model.p = p
    model.disc_dyn_expr = disc_dyn_expr

    # Barrier rows (n_obs distance rows +, unless distance_only, n_obs DCBF
    # rows). Row order is obstacle-major within each row type; the C++
    # SolverDiagnostics::cbf_values uses index k*n_obs + j for the distance
    # rows, so a mismatch here makes every diagnostic point at the wrong
    # obstacle without failing anything.
    #
    # Stage 0 carries the same rows as every other interior stage: x_0 is
    # fixed, so the DCBF row h_j(F(x_0,u_0)) - h_j(x_0) + gamma*h_j(x_0) >= 0
    # reduces to a constraint on u_0 alone — the DCBF condition on the step
    # that is actually applied to the plant (A3 requires it at k = 0 too).
    # The distance row h_j(x_0) >= 0 is what makes an unsafe start surface as
    # infeasible instead of tracking a negative h.
    model.con_h_expr_0 = _barrier_rows(
        spec, variant, n_obstacles, x, u_phys, u, p, dt
    )
    return model


def _distance_rows(spec, n_obstacles, x, p):
    """Vertical stack of the n_obs distance rows h_j(x) >= 0 (obstacle-major)."""
    return ca.vertcat(*[
        barrier_expression(spec, x, p[OBS_PARAMS_PER_OBSTACLE * j : OBS_PARAMS_PER_OBSTACLE * (j + 1)])
        for j in range(n_obstacles)
    ])


def _barrier_rows(spec, variant, n_obstacles, x, u_phys, u, p, dt):
    """Vertical stack of the constraint rows attached to a stage.

    distance_only: n_obs rows h_j(x) >= 0 only.
    otherwise:     distance rows followed by n_obs DCBF rows
                   h_j(F(x, u)) - h_j(x) + (omega_j|1) * gamma * h_j(x) >= 0.
    Obstacle-major ordering within each row type.

    The DCBF row substitutes F(x_k, u_k) for x_{k+1} — the choice documented
    in the module docstring.
    """
    gamma = p[gamma_param_offset(n_obstacles)]
    method = "exact" if spec.name == "double_integrator_2d" else "rk4"
    F = discretise(spec, dt, method)

    h_now = [barrier_expression(spec, x, p[OBS_PARAMS_PER_OBSTACLE * j : OBS_PARAMS_PER_OBSTACLE * (j + 1)]) for j in range(n_obstacles)]
    if variant == "distance_only":
        return ca.vertcat(*h_now)

    h_next = [barrier_expression(spec, F(x, u_phys), p[OBS_PARAMS_PER_OBSTACLE * j : OBS_PARAMS_PER_OBSTACLE * (j + 1)]) for j in range(n_obstacles)]
    dcbf_rows = []
    for j in range(n_obstacles):
        if variant == "relaxed_decay":
            # omega_j lives in the extended input vector, after the physical nu.
            omega_j = u[spec.nu + j]
        else:
            omega_j = None
        dcbf_rows.append(dcbf_constraint(h_next[j], h_now[j], gamma, omega_j))
    return ca.vertcat(*(h_now + dcbf_rows))


def build_ocp(
    model_name: str = "double_integrator_2d",
    horizon: int = 8,
    dt: float = 0.1,
    variant: str = "fixed_decay",
    n_obstacles: int = 8,
    cbf_horizon: int | None = None,
    use_rti: bool = False,
    max_sqp_iterations: int = 20,
) -> AcadosOcp:
    """Return a fully configured AcadosOcp.

    `model_name` is the MODEL KEY (registry key) unless it already parses as a
    full generated-solver name. cbf_horizon defaults to `horizon`.
    """
    if variant not in VARIANTS:
        raise ValueError(f"unknown CBF variant {variant!r}, expected one of {VARIANTS}")
    if n_obstacles < 1:
        raise ValueError(f"n_obstacles must be >= 1, got {n_obstacles}")
    if cbf_horizon is None:
        cbf_horizon = horizon
    if cbf_horizon < 1:
        raise ValueError(f"cbf_horizon must be >= 1, got {cbf_horizon}")
    if not _ACADOS_AVAILABLE:
        raise ImportError("acados_template is not installed — cannot build the OCP")

    try:
        model_key, _, _, _ = parse_solver_name(model_name)
    except ValueError:
        model_key = model_name
    full_name = solver_name(model_key, horizon, variant)
    spec = MODEL_REGISTRY[model_key]()

    # 1. model + horizon.
    ocp = AcadosOcp()
    ocp.model = build_acados_model(full_name, dt, n_obstacles)
    ocp.dims.N = horizon

    # 2. NONLINEAR_LS cost: y = [x; u] (relaxed: + sqrt(w)*(omega - 1)), y_e = x.
    # The residual is pre-scaled by sqrt(w), so W is identity on the omega block
    # (cost = w*(omega-1)^2), not w^2*(omega-1)^2.
    q, r, qf = default_weights(spec)
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    u_phys = ocp.model.u[: spec.nu]
    omega_weight = 1000.0  # YAML default (mpc_cbf_params.yaml); runtime-settable
    if variant == "relaxed_decay":
        omega = ocp.model.u[spec.nu :]
        cost_y = ca.vertcat(ocp.model.x, u_phys, ca.sqrt(omega_weight) * (omega - 1.0))
        ocp.cost.W = np.diag(np.concatenate([q, r, np.ones(n_obstacles)]))
    else:
        cost_y = ca.vertcat(ocp.model.x, u_phys)
        ocp.cost.W = np.diag(np.concatenate([q, r]))
    ocp.model.cost_y_expr = cost_y
    ocp.model.cost_y_expr_e = ocp.model.x
    ocp.cost.W_e = np.diag(qf)
    ocp.cost.yref = np.zeros(spec.nx + spec.nu + (n_obstacles if variant == "relaxed_decay" else 0))
    ocp.cost.yref_e = np.zeros(spec.nx)

    # 3. Bounds: all inputs; only the FINITE state bounds (skip ±1e9 entries).
    ocp.constraints.idxbu = np.arange(spec.nu)
    ocp.constraints.lbu = -1.0 * np.ones(spec.nu)
    ocp.constraints.ubu = 1.0 * np.ones(spec.nu)
    if variant == "relaxed_decay":
        ocp.constraints.idxbu = np.arange(spec.nu + n_obstacles)
        ocp.constraints.lbu = np.concatenate([-np.ones(spec.nu), np.zeros(n_obstacles)])
        ocp.constraints.ubu = np.concatenate([np.ones(spec.nu), 3.0 * np.ones(n_obstacles)])

    # 4. Initial state: zeros, overwritten every solve.
    ocp.constraints.x0 = np.zeros(spec.nx)

    # 5-6. Barrier rows. con_h_expr (stages 0..N-1) carries distance + DCBF
    # rows — including stage 0, whose DCBF row constrains u_0, the step
    # actually applied to the plant (A3); con_h_expr_e (stage N)
    # carries distance rows only, since stage N has no u_N for a DCBF row.
    # The stage-0 distance row is what makes an unsafe start report infeasible
    # (InfeasibilityReasonIsPopulated).
    n_dcbf = 0 if variant == "distance_only" else n_obstacles
    ocp.model.con_h_expr = _barrier_rows(
        spec, variant, n_obstacles, ocp.model.x, u_phys, ocp.model.u, ocp.model.p, dt
    )
    ocp.model.con_h_expr_e = _distance_rows(spec, n_obstacles, ocp.model.x, ocp.model.p)
    ocp.constraints.lh = np.zeros(n_obstacles + n_dcbf)
    ocp.constraints.uh = CONSTRAINT_UB * np.ones(n_obstacles + n_dcbf)
    ocp.constraints.lh_0 = np.zeros(n_obstacles + n_dcbf)
    ocp.constraints.uh_0 = CONSTRAINT_UB * np.ones(n_obstacles + n_dcbf)
    ocp.constraints.lh_e = np.zeros(n_obstacles)
    ocp.constraints.uh_e = CONSTRAINT_UB * np.ones(n_obstacles)

    # Default parameter values: far-away dummy obstacles + gamma.
    p_default = np.zeros(7 * n_obstacles + 1)
    for j in range(n_obstacles):
        p_default[7 * j : 7 * j + 3] = 1.0e6  # (ox, oy, oz) far away
        p_default[7 * j + 6] = 0.0            # radius 0
    p_default[7 * n_obstacles] = 0.3          # gamma
    ocp.parameter_values = p_default

    # 8. Solver options.
    ocp.solver_options.tf = horizon * dt  # stage duration = tf/N = dt
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"  # discrete-time formulation; never ERK
    ocp.solver_options.nlp_solver_type = "SQP_RTI" if use_rti else "SQP"
    ocp.solver_options.nlp_solver_max_iter = max_sqp_iterations
    # Levenberg-Marquardt regularization of the Gauss-Newton Hessian. 1e-4
    # (acados default) leaves HPIPM's Newton system ill-conditioned on
    # degenerate CBF QPs (grazing trajectories, tight gamma), producing
    # spurious ACADOS_MINSTEP -> QP_FAILURE. 1e-2 regularizes the singular
    # directions; it must match mpc_cbf_solver.cpp (mpc_cbf_unified) so the
    # JSON and the C++ runtime solve the same problem.
    ocp.solver_options.levenberg_marquardt = 1.0e-2
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.print_level = 0
    # NLP termination tolerances. acados_template defaults all four to 1e-6;
    # the runtime overrides them to the SAME values (mpc_cbf_solver.cpp), so
    # the JSON, the Python reference (A7 parity) and the C++ runtime all solve
    # the same problem. All four are loosened to 1e-3 together because acados
    # forwards them to the inner QP solver: at grazing points res_stat floors
    # at ~3e-6 and res_comp at ~2e-4, so a tight tol_stat makes HPIPM stall at
    # qp_iter_max and the SQP diverge from an otherwise-good KKT point. This
    # is not a safety weakening — the CBF safety contract is checked on the
    # returned trajectory (tests use 1e-6 on the barrier rows), not on solver
    # residuals.
    ocp.solver_options.nlp_solver_tol_stat = 1.0e-3
    ocp.solver_options.nlp_solver_tol_eq = 1.0e-3
    ocp.solver_options.nlp_solver_tol_ineq = 1.0e-3
    ocp.solver_options.nlp_solver_tol_comp = 1.0e-3
    return ocp


def generate(output_dir: str = DEFAULT_OUTPUT_DIR, **kwargs) -> str:
    """Generate C code and return the path to the generated directory.

    Layout: <output_dir>/<solver_name>/<solver_name>.json plus the acados
    generated tree directly under it (acados >= 0.5.6 exports the C code to
    code_gen_options.code_export_directory; the pre-0.5.6 c_generated_code
    subdirectory no longer exists). CMake GLOB_RECURSEs the whole tree, so
    distinct solver names keep the six configurations' symbols from colliding.
    """
    ocp = build_ocp(**kwargs)
    solver = ocp.model.name
    out = os.path.join(output_dir, solver)
    os.makedirs(out, exist_ok=True)
    json_file = os.path.join(out, f"{solver}.json")
    # acados >= 0.5.6: an explicit ocp.name fixes the C entry-point prefix
    # (otherwise the solver is named ocp_<model>_<hash>), and the export
    # directory + json path are code_gen_options, not constructor args.
    ocp.name = solver
    ocp.code_gen_options.json_file = json_file
    ocp.code_gen_options.code_export_directory = out
    # generate=True + build=True compile the C code; requires the acados env.
    AcadosOcpSolver(ocp, generate=True, build=True)
    # acados exports the generated tree next to the json file, at out/.
    generated_c = os.path.join(out, f"acados_solver_{solver}.c")
    if not os.path.isfile(generated_c):
        raise RuntimeError(f"acados did not produce {generated_c} — generation failed")
    print(solver)
    return out


def generate_all(output_dir: str = DEFAULT_OUTPUT_DIR) -> list[str]:
    """Generate every configuration the tests and launch files need."""
    matrix = [
        ("double_integrator_2d", 8, "fixed_decay"),    # 2d_obstacle, gtests, ACC notebook
        ("double_integrator_2d", 3, "distance_only"),  # MPC-DC baseline (A5)
        ("double_integrator_2d", 8, "relaxed_decay"),  # CDC 2021 (A6)
        ("bicycle_kinematic", 11, "fixed_decay"),      # car_racing
        ("quadrotor_planar", 15, "fixed_decay"),       # dynamic obstacle demo
        ("quadrotor_planar", 1, "fixed_decay"),        # myopic CBF-QP baseline
    ]
    names = []
    for model_key, horizon, variant in matrix:
        name = solver_name(model_key, horizon, variant)
        generate(output_dir, model_name=model_key, horizon=horizon, variant=variant)
        names.append(name)
    return names


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="double_integrator_2d",
                        choices=list(MODEL_REGISTRY.keys()), help="model key")
    parser.add_argument("--horizon", type=int, default=8, help="prediction horizon N")
    parser.add_argument("--dt", type=float, default=0.1, help="sampling period [s]")
    parser.add_argument("--n-obstacles", type=int, default=8, help="obstacle slots in the parameter vector")
    parser.add_argument("--variant", default="fixed_decay", choices=VARIANTS,
                        help="CBF variant: fixed_decay | relaxed_decay | distance_only")
    parser.add_argument("--rti", action="store_true", help="use SQP_RTI instead of full SQP")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="output directory")
    parser.add_argument("--all", action="store_true",
                        help="generate the full configuration matrix (ignores per-config flags)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.all:
        generate_all(args.output_dir)
    else:
        generate(
            args.output_dir,
            model_name=args.model,
            horizon=args.horizon,
            dt=args.dt,
            variant=args.variant,
            n_obstacles=args.n_obstacles,
            use_rti=args.rti,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
