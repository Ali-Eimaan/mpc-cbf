# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Tests for codegen/generate_tube_solver.py.

Everything here runs WITHOUT acados: the numpy/scipy cross-check
(compute_offline_sets) and the casadi-only row assembly are exactly the parts
the laptop must be able to exercise. The acados OCP assembly itself is only
exercised in CI, where acados_template exists.

The parity numbers below are the same fixture as
test_tube_mpc_robustness.cpp RpiMatchesPythonReference: double integrator,
dt=0.1, Q=(10,10,1,1), R=I, W = box(0.005,0.005,0.02,0.02), epsilon=1e-3.
"""

from __future__ import annotations

import casadi as ca
import numpy as np
import pytest

from codegen.generate_tube_solver import (
    _tube_barrier_rows,
    _tube_distance_rows,
    compute_offline_sets,
    fixture_reference_supports,
    parse_tube_solver_name,
    tightening_offset,
    tube_param_count,
    tube_solver_name,
)
from codegen.models import (
    barrier_expression,
    dcbf_constraint,
    discretise,
    MODEL_REGISTRY,
)

RNG_SEED = 0xC0FFEE
print(f"test_generate_tube_solver: RNG seed = {RNG_SEED:#x}")
RNG = np.random.default_rng(RNG_SEED)

FIXTURE_DT = 0.1
FIXTURE_W = [0.005, 0.005, 0.02, 0.02]
FIXTURE_Q = [10.0, 10.0, 1.0, 1.0]
FIXTURE_R = [1.0, 1.0]


def test_SolverNameRoundTrip():
    for model_key in MODEL_REGISTRY:
        for mode in ("support_function", "lipschitz", "none"):
            name = tube_solver_name(model_key, 8, mode)
            assert name.startswith(f"tube_mpc_cbf_{model_key}_N8_")
            assert parse_tube_solver_name(name) == (model_key, 8, mode)


def test_SolverNameRejectsUnknownMode():
    with pytest.raises(ValueError):
        tube_solver_name("double_integrator_2d", 8, "gauss_newton")


def test_ParameterLayoutMatchesCpp():
    """np = 7*n_obs + 1 + n_obs = 65 for 8 obstacles; gamma at 7*n_obs, the
    tightening block at 7*n_obs + 1 .. 7*n_obs + n_obs — the layout
    TubeMpcCbfSolver::solve() packs p[7*n_obs + 1 + j] = c_{k,j} into."""
    n_obs = 8
    assert tube_param_count(n_obs) == 65
    assert tube_param_count(1) == 9
    assert tightening_offset(n_obs) == 57
    assert tube_param_count(0) == 1  # degenerate: just gamma


def test_OfflineSetsMatchFixture():
    K, vertices, alpha, s = compute_offline_sets(
        "double_integrator_2d", FIXTURE_DT, FIXTURE_W, FIXTURE_Q, FIXTURE_R
    )
    assert s == 49  # the gtest asserts this exact iteration count
    assert alpha == pytest.approx(0.002862378, abs=1e-9)
    assert K == pytest.approx(
        np.array(
            [
                [-2.76234997, 0.0, -2.50754016, 0.0],
                [0.0, -2.76234997, 0.0, -2.50754016],
            ]
        ),
        abs=1e-8,
    )
    assert vertices.shape == (16, 4)
    assert np.abs(vertices).max(axis=0) == pytest.approx(
        [0.11553617, 0.11553617, 0.16415113, 0.16415113], abs=1e-8
    )


def test_ReferenceSupportsAreUnitDirectionsAndMatchGtest():
    """32 directions: 8 axis, then 24 seeded random unit vectors; the axis
    supports equal the hull half-widths by construction, and the first random
    entry reproduces the value hardcoded in RpiMatchesPythonReference."""
    pairs = fixture_reference_supports()
    assert len(pairs) == 32
    for d, _ in pairs:
        assert np.linalg.norm(d) == pytest.approx(1.0, abs=1e-12)
    axis = pairs[0][1]
    assert axis == pytest.approx(0.11553617254755075, abs=1e-12)
    _, support = pairs[8]  # first random direction
    assert support == pytest.approx(0.18714357740722548, abs=1e-6)


def test_TubeRowsMatchNominalWhenTighteningZero():
    """With c = 0 the tube rows must reduce to the nominal generator's rows:
    distance h(z), DCBF h(F(z,v)) - h(z) + gamma*h(z)."""
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    n_obs = 8
    z = ca.SX.sym("z", spec.nx)
    v = ca.SX.sym("v", spec.nu)
    p = ca.SX.sym("p", tube_param_count(n_obs))
    F = discretise(spec, FIXTURE_DT, "exact")

    p_zero = np.zeros(tube_param_count(n_obs))
    p_zero[tightening_offset(n_obs):] = 0.0  # c = 0
    gamma = 0.3
    p_zero[7 * n_obs] = gamma

    distance = ca.Function("d", [z, p], [_tube_distance_rows(spec, n_obs, z, p)])
    barrier = ca.Function("b", [z, v, p], [_tube_barrier_rows(spec, n_obs, z, v, p, FIXTURE_DT)])

    for _ in range(5):
        x = RNG.uniform(-1.0, 1.0, spec.nx)
        u = RNG.uniform(-1.0, 1.0, spec.nu)
        obs_p = np.zeros(7 * n_obs)
        for j in range(n_obs):
            obs_p[7 * j: 7 * j + 3] = RNG.uniform(0.5, 2.0, 3)
            obs_p[7 * j + 6] = RNG.uniform(0.1, 0.5)
        p_val = np.concatenate([obs_p, [gamma], np.zeros(n_obs)])

        for j in range(n_obs):
            h_expr = barrier_expression(spec, z, ca.SX(p_val[7 * j: 7 * j + 7]))
            h_j = ca.Function("h", [z], [h_expr])
            h_now = float(h_j(x))
            h_next = float(h_j(np.asarray(F(x, u)).ravel()))
            expected_dcbf = dcbf_constraint(h_next, h_now, gamma)
            actual_dcbf = float(barrier(x, u, p_val)[n_obs + j])
            assert actual_dcbf == pytest.approx(expected_dcbf, abs=1e-9)
            expected_distance = h_now
            actual_distance = float(distance(x, p_val)[j])
            assert actual_distance == pytest.approx(expected_distance, abs=1e-9)


def test_TubeRowsCarryTighteningInBothRows():
    """The c term enters distance as h - c and the DCBF as gamma*(h - c): bump
    c by delta and check both rows move the way the derivation says."""
    spec = MODEL_REGISTRY["double_integrator_2d"]()
    n_obs = 2  # small, so the index arithmetic is easy to eyeball
    z = ca.SX.sym("z", spec.nx)
    v = ca.SX.sym("v", spec.nu)
    p = ca.SX.sym("p", tube_param_count(n_obs))
    F = discretise(spec, FIXTURE_DT, "exact")

    distance = ca.Function("d", [z, p], [_tube_distance_rows(spec, n_obs, z, p)])
    barrier = ca.Function("b", [z, v, p], [_tube_barrier_rows(spec, n_obs, z, v, p, FIXTURE_DT)])

    x = np.array([0.0, 0.0, 0.0, 0.0])
    u = np.array([0.1, -0.2])
    gamma = 0.3
    obs_p = np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, -1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.3])
    p0 = np.concatenate([obs_p, [gamma], [0.4, 0.9]])

    d0 = float(distance(x, p0)[0])
    b0 = float(barrier(x, u, p0)[n_obs])  # first DCBF row

    h_j = ca.Function("h", [z], [barrier_expression(spec, z, ca.SX(obs_p[0:7]))])
    h_now = float(h_j(x))
    h_next = float(h_j(np.asarray(F(x, u)).ravel()))

    assert d0 == pytest.approx(h_now - 0.4, abs=1e-9)
    assert b0 == pytest.approx(h_next - h_now + gamma * (h_now - 0.4), abs=1e-9)
