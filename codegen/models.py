"""Shared CasADi dynamics models.

Added because the two generator scripts, the
pytest suite and every notebook must agree bit-for-bit on the dynamics. One
definition, four consumers.

This module is the single source of truth for:

  * the four plants,
  * their discrete-time dynamics,
  * the barrier function h(x) = ||P x - p_obs||^2 - r_eff^2,
  * the DCBF constraint expression.

Note: `acados_template.AcadosModel` is deliberately NOT imported at module level
— this file is imported by tests and notebooks that must run on machines
without acados (the CI Python job installs only codegen/requirements.txt). The
generators import it lazily where they build the AcadosModel. A bug in the
barrier here is invisible everywhere and wrong everywhere; keep
`MpcCbfSolver::barrierValue()` in C++ in lock-step with `barrier_expression()`
(test: BarrierMatchesGeneratedCode).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import casadi as ca
import numpy as np

# Fixed RNG seed for the randomised tests; a test that cannot be replayed is a
# lottery, not a test.
RNG_SEED = 0xC0FFEE


@dataclass
class ModelSpec:
    """Everything the generators need to know about one plant."""

    name: str
    nx: int
    nu: int
    position_indices: tuple[int, ...]  # which state entries form the position p(x)
    state_names: tuple[str, ...] = field(default_factory=tuple)
    input_names: tuple[str, ...] = field(default_factory=tuple)
    # Continuous-time f(x, u) as a CasADi Function; the generators discretise
    # it, so keep this continuous.
    dynamics: Callable | None = None


# ---------------------------------------------------------------------------
# Model definitions
# ---------------------------------------------------------------------------


def double_integrator_2d() -> ModelSpec:
    """x = [px, py, vx, vy], u = [ax, ay];  pdot = v, vdot = u.

    The only model with an exact discretisation — `linearised_discrete()`
    returns the exact ZOH matrices so the tube solver's RPI set is exact rather
    than approximate.
    """
    x = ca.SX.sym("x", 4)
    u = ca.SX.sym("u", 2)
    xdot = ca.vertcat(x[2], x[3], u[0], u[1])
    f = ca.Function("f_double_integrator_2d", [x, u], [xdot], ["x", "u"], ["xdot"])
    return ModelSpec(
        name="double_integrator_2d",
        nx=4,
        nu=2,
        position_indices=(0, 1),
        state_names=("px", "py", "vx", "vy"),
        input_names=("ax", "ay"),
        dynamics=f,
    )


def unicycle_2d() -> ModelSpec:
    """x = [px, py, theta], u = [v, omega];  pdot = v [cos t, sin t], tdot = omega."""
    x = ca.SX.sym("x", 3)
    u = ca.SX.sym("u", 2)
    th = x[2]
    xdot = ca.vertcat(u[0] * ca.cos(th), u[0] * ca.sin(th), u[1])
    f = ca.Function("f_unicycle_2d", [x, u], [xdot], ["x", "u"], ["xdot"])
    return ModelSpec(
        name="unicycle_2d",
        nx=3,
        nu=2,
        position_indices=(0, 1),
        state_names=("px", "py", "theta"),
        input_names=("v", "omega"),
        dynamics=f,
    )


def bicycle_kinematic(wheelbase: float = 0.35) -> ModelSpec:
    """x = [px, py, theta, v], u = [a, delta]; thetadot = v tan(delta) / L."""
    x = ca.SX.sym("x", 4)
    u = ca.SX.sym("u", 2)
    th = x[2]
    v = x[3]
    delta = u[1]
    xdot = ca.vertcat(v * ca.cos(th), v * ca.sin(th), v * ca.tan(delta) / wheelbase, u[0])
    f = ca.Function("f_bicycle_kinematic", [x, u], [xdot], ["x", "u"], ["xdot"])
    return ModelSpec(
        name="bicycle_kinematic",
        nx=4,
        nu=2,
        position_indices=(0, 1),
        state_names=("px", "py", "theta", "v"),
        input_names=("a", "delta"),
        dynamics=f,
    )


def quadrotor_planar(mass: float = 1.0, inertia: float = 0.01, g: float = 9.81) -> ModelSpec:
    """x = [px, pz, vx, vz, phi, phidot], u = [T, tau] (thrust, torque).

    Lives in the VERTICAL plane: index 1 is pz, not py. This is the single most
    common source of confusion in the repository; the test PositionSelectorIsVerticalForPlanarQuadrotor guards it.
    """
    x = ca.SX.sym("x", 6)
    u = ca.SX.sym("u", 2)
    phi = x[4]
    phidot = x[5]
    T = u[0]
    tau = u[1]
    xdot = ca.vertcat(
        x[2],
        x[3],
        -T * ca.sin(phi) / mass,
        T * ca.cos(phi) / mass - g,
        phidot,
        tau / inertia,
    )
    f = ca.Function("f_quadrotor_planar", [x, u], [xdot], ["x", "u"], ["xdot"])
    return ModelSpec(
        name="quadrotor_planar",
        nx=6,
        nu=2,
        position_indices=(0, 1),
        state_names=("px", "pz", "vx", "vz", "phi", "phidot"),
        input_names=("T", "tau"),
        dynamics=f,
    )


MODEL_REGISTRY: dict[str, Callable[[], ModelSpec]] = {
    "double_integrator_2d": double_integrator_2d,
    "unicycle_2d": unicycle_2d,
    "bicycle_kinematic": bicycle_kinematic,
    "quadrotor_planar": quadrotor_planar,
}


# ---------------------------------------------------------------------------
# Discretisation and barriers
# ---------------------------------------------------------------------------


def _exact_zoh(dim: int, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact zero-order-hold matrices for x = [p; v], u = a:
    A = [[I, dt I], [0, I]], B = [[dt^2/2 I], [dt I]].
    """
    eye = np.eye(dim)
    zeros = np.zeros((dim, dim))
    A = np.block([[eye, dt * eye], [zeros, eye]])
    B = np.block([[0.5 * dt * dt * eye], [dt * eye]])
    return A, B


def discretise(spec: ModelSpec, dt: float, method: str = "rk4"):
    """Continuous f -> discrete x_{k+1} = F(x_k, u_k) as a CasADi Function.

    * "rk4"    — explicit RK4, one step per dt (default for nonlinear models).
    * "exact"  — double_integrator_2d only: the exact ZOH map A x + B u.
    * "euler"  — exists only so the notebooks can demonstrate why it is
                  inadequate at dt = 0.1. Never a default.

    Do not silently substitute Euler for RK4 in any path a notebook plots.
    """
    if method == "exact":
        if spec.name != "double_integrator_2d":
            raise ValueError(
                f"'exact' discretisation is defined for double_integrator_2d only, got {spec.name}"
            )
        A, B = _exact_zoh(spec.nx // 2, dt)
        x = ca.SX.sym("x", spec.nx)
        u = ca.SX.sym("u", spec.nu)
        x_next = A @ x + B @ u
        return ca.Function("F", [x, u], [x_next], ["x", "u"], ["x_next"])

    if spec.dynamics is None:
        raise ValueError(f"model {spec.name!r} has no continuous dynamics")

    if method == "rk4":
        f = spec.dynamics
        x = ca.SX.sym("x", spec.nx)
        u = ca.SX.sym("u", spec.nu)
        k1 = f(x, u)
        k2 = f(x + 0.5 * dt * k1, u)
        k3 = f(x + 0.5 * dt * k2, u)
        k4 = f(x + dt * k3, u)
        x_next = x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return ca.Function("F", [x, u], [x_next], ["x", "u"], ["x_next"])

    if method == "euler":
        f = spec.dynamics
        x = ca.SX.sym("x", spec.nx)
        u = ca.SX.sym("u", spec.nu)
        x_next = x + dt * f(x, u)
        return ca.Function("F", [x, u], [x_next], ["x", "u"], ["x_next"])

    raise ValueError(f"unknown discretisation method {method!r}")


def linearised_discrete(spec: ModelSpec, dt: float, x_op=None, u_op=None):
    """Return (A, B) of the discrete-time linearisation about (x_op, u_op).

    Used by the tube solver for K and the RPI set. For the double integrator,
    ignore the operating point and return the exact ZOH matrices — the tube's
    RPI certificate is then exact rather than linearisation-local. For the
    nonlinear models, linearise the RK4-discretised map about the operating
    point; the caller must record that the certificate is local.
    """
    if spec.name == "double_integrator_2d":
        return _exact_zoh(spec.nx // 2, dt)

    if x_op is None:
        x_op = np.zeros(spec.nx)
    if u_op is None:
        u_op = np.zeros(spec.nu)
    x_op = np.asarray(x_op, dtype=float).reshape(spec.nx)
    u_op = np.asarray(u_op, dtype=float).reshape(spec.nu)

    F = discretise(spec, dt, "rk4")
    x = ca.SX.sym("x", spec.nx)
    u = ca.SX.sym("u", spec.nu)
    x_next = F(x, u)
    A_fn = ca.Function("A", [x, u], [ca.jacobian(x_next, x)])
    B_fn = ca.Function("B", [x, u], [ca.jacobian(x_next, u)])
    return np.asarray(A_fn(x_op, u_op), dtype=float), np.asarray(B_fn(x_op, u_op), dtype=float)


def barrier_expression(spec: ModelSpec, x, obstacle_params):
    """h(x) = ||p(x) - p_obs||^2 - r_eff^2 as a CasADi expression.

    THE definition of h in the repo. `obstacle_params` is the symbolic slice of
    the acados parameter vector for one obstacle:
        [px, py, pz, vx, vy, vz, radius]
    The radius passed in must already be the
    *effective* radius r_eff = r_obs + ego_radius + safety_margin; the caller
    does the inflation, the expression does not — inflating twice is a real bug
    that produces a mysteriously conservative controller.
    """
    p = ca.vertcat(*[x[i] for i in spec.position_indices])
    p_obs = ca.vertcat(*[obstacle_params[i] for i in range(len(spec.position_indices))])
    radius = obstacle_params[6]
    # Squared distance, never the norm: smooth at p = p_obs, quadratic in x,
    # and gives an affine constraint Jacobian (exact Gauss-Newton Hessian).
    return ca.sumsqr(p - p_obs) - radius * radius


def dcbf_constraint(h_next, h_now, gamma, omega=None):
    """The discrete-time CBF condition, returned as an expression that must be >= 0.

        fixed decay:    h_next - h_now + gamma * h_now
        relaxed decay:  h_next - h_now + omega * gamma * h_now

    Single source of truth for the constraint — both generators and the
    notebooks import it. gamma in (0, 1] and omega >= 0 are validated by the
    caller, not here; this function stays a pure expression so it can be used
    symbolically and numerically without branching.
    """
    return h_next - h_now + (omega if omega is not None else 1.0) * gamma * h_now
