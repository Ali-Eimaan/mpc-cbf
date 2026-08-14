# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Tests for codegen/models.py.

Fixed RNG seed, printed at the top of the module. A randomised test that cannot
be replayed is a lottery, not a test.
"""

from __future__ import annotations

import casadi as ca
import numpy as np

from codegen.models import (
    MODEL_REGISTRY,
    barrier_expression,
    dcbf_constraint,
    discretise,
    linearised_discrete,
)

RNG_SEED = 0xC0FFEE
print(f"test_models: RNG seed = {RNG_SEED:#x}")
RNG = np.random.default_rng(RNG_SEED)


def _analytic_unicycle(p0, th0, v, w, t):
    """Closed form of the unicycle with constant (v, w): a circular arc of
    radius v/w centred at p0 + (v/w)[-sin th0, cos th0]."""
    th = th0 + w * t
    p = p0 + (v / w) * np.array([np.sin(th) - np.sin(th0), -(np.cos(th) - np.cos(th0))])
    return p, th


def test_ExactZohMatchesRk4ForLinearModel():
    """'exact' and 'rk4' must agree to 1e-12 for the double integrator — RK4 is
    exact for a linear system, so any disagreement is a coding error, not a
    truncation error."""
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    F_exact = discretise(spec, 0.1, "exact")
    F_rk4 = discretise(spec, 0.1, "rk4")
    for _ in range(20):
        x = RNG.uniform(-2.0, 2.0, spec.nx)
        u = RNG.uniform(-1.0, 1.0, spec.nu)
        assert np.max(np.abs(F_exact(x, u) - F_rk4(x, u))) < 1e-12


def test_Rk4MatchesAnalyticUnicycleArc():
    """Constant (v, w) traces a circular arc of radius v/w; match the closed
    form to 1e-9 over 100 steps."""
    spec = MODEL_REGISTRY["unicycle_2d"]()
    dt = 0.02  # global RK4 error scales as dt^4; 0.02 keeps 100 steps well inside 1e-9
    v, w = 1.0, 0.8
    x0 = np.array([0.0, 0.0, 0.3])
    F = discretise(spec, dt, "rk4")
    x = x0.copy()
    for k in range(1, 101):
        x = np.asarray(F(x, np.array([v, w]))).ravel()
        p_an, th_an = _analytic_unicycle(x0[:2], x0[2], v, w, k * dt)
        assert np.linalg.norm(x[:2] - p_an) < 1e-9
        assert abs(x[2] - th_an) < 1e-9


def test_EulerIsWorseThanRk4():
    """The demonstration the notebooks cite: assert the ordering of the errors,
    not their values."""
    spec = MODEL_REGISTRY["unicycle_2d"]()
    dt = 0.1
    v, w = 1.5, 0.6
    x0 = np.array([0.2, -0.1, 0.0])
    F_rk4 = discretise(spec, dt, "rk4")
    F_euler = discretise(spec, dt, "euler")
    x_rk4 = x0.copy()
    x_euler = x0.copy()
    err_rk4 = 0.0
    err_euler = 0.0
    for k in range(1, 51):
        x_rk4 = np.asarray(F_rk4(x_rk4, np.array([v, w]))).ravel()
        x_euler = np.asarray(F_euler(x_euler, np.array([v, w]))).ravel()
        p_an, _ = _analytic_unicycle(x0[:2], x0[2], v, w, k * dt)
        err_rk4 = max(err_rk4, np.linalg.norm(x_rk4[:2] - p_an))
        err_euler = max(err_euler, np.linalg.norm(x_euler[:2] - p_an))
    assert err_euler > err_rk4


def _barrier_function(spec, n_obs: int = 1):
    x = ca.SX.sym("x", spec.nx)
    p = ca.SX.sym("p", 7 * n_obs)
    h_expr = barrier_expression(spec, x, p)
    H = ca.Function("h", [x, p], [h_expr])
    G = ca.Function("grad_h", [x, p], [ca.jacobian(h_expr, x)])
    return H, G


def test_BarrierSignConvention():
    """h > 0 strictly outside the inflated obstacle, == 0 on its boundary
    (within 1e-12), < 0 inside."""
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    H, _ = _barrier_function(spec)
    obs = np.array([0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.2])
    r = obs[6]
    assert float(H(np.array([2.0, 2.0, 0.0, 0.0]), obs)) > 0.0
    on_boundary = np.array([0.5 + r / np.sqrt(2.0), 0.5 + r / np.sqrt(2.0), 0.0, 0.0])
    assert abs(float(H(on_boundary, obs))) < 1e-12
    assert float(H(np.array([0.5, 0.5, 0.0, 0.0]), obs)) < 0.0


def test_BarrierGradientMatchesFiniteDifference():
    """Central differences, step 1e-5 * max(1, |x_i|), tolerance 1e-6."""
    spec = MODEL_REGISTRY["bicycle_kinematic"]()
    H, G = _barrier_function(spec)
    obs = np.array([0.3, 0.7, 0.0, 0.0, 0.0, 0.0, 0.25])
    for _ in range(10):
        x0 = RNG.uniform(-1.5, 1.5, spec.nx)
        g_an = np.asarray(G(x0, obs)).ravel()
        g_fd = np.zeros(spec.nx)
        for i in range(spec.nx):
            h_i = 1e-5 * max(1.0, abs(x0[i]))
            xp = x0.copy()
            xm = x0.copy()
            xp[i] += h_i
            xm[i] -= h_i
            g_fd[i] = (float(H(xp, obs)) - float(H(xm, obs))) / (2.0 * h_i)
        assert np.max(np.abs(g_an - g_fd)) < 1e-6


def test_DcbfConstraintReducesToFixedDecayAtOmegaOne():
    """dcbf_constraint(a, b, g, 1.0) == dcbf_constraint(a, b, g) exactly."""
    x = ca.SX.sym("x")
    y = ca.SX.sym("y")
    g = ca.SX.sym("g")
    with_omega = ca.Function("f", [x, y, g], [dcbf_constraint(y, x, g, 1.0)])
    without = ca.Function("f", [x, y, g], [dcbf_constraint(y, x, g)])
    for _ in range(10):
        h1, h2 = RNG.uniform(-2.0, 2.0), RNG.uniform(-2.0, 2.0)
        assert float(with_omega(h1, h2, 0.3)) == float(without(h1, h2, 0.3))


def test_PositionSelectorIsVerticalForPlanarQuadrotor():
    """Guards the repository's favourite bug: the planar quadrotor lives in the vertical
    plane, so index 1 is pz and must never be called py."""
    spec = MODEL_REGISTRY["quadrotor_planar"]()
    assert spec.position_indices == (0, 1)
    assert spec.state_names[0] == "px"
    assert spec.state_names[1] == "pz"
    assert "py" not in spec.state_names


def test_LinearisedDiscreteIsExactForDoubleIntegrator():
    """The tube's RPI set is computed from (A, B); for the double integrator
    these must be the exact ZOH matrices, independent of the operating point."""
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    dt = 0.1
    A, B = linearised_discrete(spec, dt)
    assert A.shape == (4, 4)
    assert B.shape == (4, 2)
    eye = np.eye(2)
    assert np.allclose(A, np.block([[eye, dt * eye], [np.zeros((2, 2)), eye]]))
    assert np.allclose(B, np.block([[0.5 * dt * dt * eye], [dt * eye]]))
    A2, B2 = linearised_discrete(spec, dt, x_op=np.ones(4), u_op=np.ones(2))
    assert np.array_equal(A, A2) and np.array_equal(B, B2)
