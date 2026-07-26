"""SKELETON — shared CasADi dynamics models.

Not listed in INFO.md's tree; added because the two generator scripts, the
pytest suite and every notebook must agree bit-for-bit on the dynamics. One
definition, four consumers.

Implement per IMPLEMENTATION_GUIDE.md §2.2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

# TODO(deepseek): import casadi as ca, numpy as np, and
#   from acados_template import AcadosModel


@dataclass
class ModelSpec:
    """Everything the generators need to know about one plant."""

    name: str
    nx: int
    nu: int
    position_indices: tuple[int, ...]  # which state entries form the position p(x)
    state_names: tuple[str, ...] = field(default_factory=tuple)
    input_names: tuple[str, ...] = field(default_factory=tuple)
    # TODO(deepseek): continuous-time f(x, u) as a CasADi function; the
    # generators discretise it, so keep this continuous.
    dynamics: Callable | None = None


# ---------------------------------------------------------------------------
# Model definitions
# ---------------------------------------------------------------------------


def double_integrator_2d() -> ModelSpec:
    """x = [px, py, vx, vy], u = [ax, ay];  pdot = v, vdot = u.

    TODO(deepseek): the only model with an exact discretisation — return the
    exact ZOH matrices from `linearised_discrete()` for this one so the tube
    solver's RPI set is exact rather than approximate.
    """
    raise NotImplementedError


def unicycle_2d() -> ModelSpec:
    """x = [px, py, theta], u = [v, omega];  pdot = v [cos t, sin t], tdot = omega."""
    raise NotImplementedError


def bicycle_kinematic(wheelbase: float = 0.35) -> ModelSpec:
    """x = [px, py, theta, v], u = [a, delta]; thetadot = v tan(delta) / L."""
    raise NotImplementedError


def quadrotor_planar(mass: float = 1.0, inertia: float = 0.01, g: float = 9.81) -> ModelSpec:
    """x = [px, pz, vx, vz, phi, phidot], u = [T, tau] (thrust, torque)."""
    raise NotImplementedError


MODEL_REGISTRY: dict[str, Callable[[], ModelSpec]] = {
    "double_integrator_2d": double_integrator_2d,
    "unicycle_2d": unicycle_2d,
    "bicycle_kinematic": bicycle_kinematic,
    "quadrotor_planar": quadrotor_planar,
}


# ---------------------------------------------------------------------------
# Discretisation and barriers
# ---------------------------------------------------------------------------


def discretise(spec: ModelSpec, dt: float, method: str = "rk4"):
    """Continuous f -> discrete x_{k+1} = F(x_k, u_k).

    TODO(deepseek): explicit RK4 by default, exact ZOH when the model is
    linear. `method` in {"rk4", "euler", "exact"}; "euler" exists only so the
    notebooks can show why it is a bad idea at dt = 0.1.
    """
    raise NotImplementedError


def linearised_discrete(spec: ModelSpec, dt: float, x_op=None, u_op=None):
    """Return (A, B) of the discrete-time linearisation about (x_op, u_op).

    TODO(deepseek): used by the tube solver for K and the RPI set. For the
    double integrator, ignore the operating point and return the exact ZOH
    matrices A = [[I, dt I], [0, I]], B = [[dt^2/2 I], [dt I]].
    """
    raise NotImplementedError


def barrier_expression(spec: ModelSpec, x, obstacle_params):
    """h(x) = ||p(x) - p_obs||^2 - (r_obs + inflation)^2 as a CasADi expression.

    TODO(deepseek): this is THE definition of h in the repo. The C++
    MpcCbfSolver::barrierValue mirrors it; the gtest
    BarrierMatchesGeneratedCode enforces that they agree. `obstacle_params` is
    the symbolic slice of the acados parameter vector for one obstacle:
    [px, py, pz, vx, vy, vz, radius] — see §3.5 for the layout.
    """
    raise NotImplementedError


def dcbf_constraint(h_next, h_now, gamma, omega=None):
    """The discrete-time CBF condition, returned as an expression that must be >= 0.

    TODO(deepseek):
        fixed decay:    h_next - h_now + gamma * h_now
        relaxed decay:  h_next - h_now + omega * gamma * h_now
    Keep this one function as the single source of truth for the constraint —
    both generators and the notebooks import it.
    """
    raise NotImplementedError
