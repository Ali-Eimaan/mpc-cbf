#!/usr/bin/env python3
"""SKELETON — acados code generation for the tube-MPC-CBF nominal solver.

Differs from generate_mpc_cbf_solver.py in three places only:
  1. state/input bounds are the *tightened* ones (X (-) Omega, U (-) K Omega),
  2. the DCBF row carries a per-stage tightening parameter c_k,
  3. the decision variables are the nominal (z, v), not (x, u).

Everything else — cost structure, solver options, parameter layout — is shared,
so import from generate_mpc_cbf_solver rather than copying.

Usage (target CLI):
    python generate_tube_solver.py --model double_integrator_2d \
        --horizon 8 --dt 0.1 --n-obstacles 4
    python generate_tube_solver.py --all

Implement per .deepseek/05_CODEGEN.md §5.7 and .deepseek/08_TUBE.md.
"""

from __future__ import annotations

import argparse

# TODO(deepseek §5.7):
#   import numpy as np, casadi as ca
#   from acados_template import AcadosOcp, AcadosOcpSolver
#   from models import MODEL_REGISTRY, discretise, barrier_expression, linearised_discrete
#   from generate_mpc_cbf_solver import build_acados_model

DEFAULT_OUTPUT_DIR = "c_generated_code"


def compute_offline_sets(model_name: str, dt: float, disturbance_box, lqr_q, lqr_r):
    """Reference (Python) implementation of the offline tube computation.

    TODO(deepseek §5.7): mirror the C++ path — discreteLqrGain, then the Rakovic
    mRPI iteration — using scipy.linalg.solve_discrete_are and polytope
    arithmetic in numpy. Its purpose is to CHECK the C++ implementation:
    test_tube_mpc_robustness.cpp asserts the two agree on Omega's support
    function in 32 directions to 1e-6. Do not let the two drift.

    Returns (K, Omega_vertices, alpha, s).
    """
    raise NotImplementedError


def build_tube_ocp(
    model_name: str = "double_integrator_2d",
    horizon: int = 8,
    dt: float = 0.1,
    n_obstacles: int = 4,
    tighten_mode: str = "support_function",
):
    """Return the AcadosOcp for the nominal tube problem.

    TODO(deepseek §5.7):
      1. Same model/cost as the nominal generator, with (z, v) naming.
      2. Parameter vector extended by one tightening scalar per (obstacle,
         stage): p = [obstacle block ...] + [gamma] + [c_{k,j} ...]. The C++
         solver fills c_{k,j} from TubeMpcCbfSolver::tighteningFor().
      3. DCBF row becomes
             h(F(z_k, v_k)) - h(z_k) + gamma * (h(z_k) - c_{k,j}) >= 0
         and the distance row becomes h(z_k) - c_{k,j} >= 0.
         Deriving why the tightening enters both rows is in .deepseek/08_TUBE.md §8.4 — read it
         before touching the sign.
      4. Bounds come in already tightened from the caller; this script must not
         recompute them (single source of truth is the C++ initialize()).
    """
    raise NotImplementedError


def generate(output_dir: str = DEFAULT_OUTPUT_DIR, **kwargs) -> str:
    """TODO(deepseek §5.7): as in generate_mpc_cbf_solver.generate, with solver names
    prefixed "tube_mpc_cbf_"."""
    raise NotImplementedError


def generate_all(output_dir: str = DEFAULT_OUTPUT_DIR) -> list[str]:
    """TODO(deepseek §5.7): (double_integrator_2d, N=8, support_function) and the
    tighten_mode="none" ablation used by the robustness sweep."""
    raise NotImplementedError


def parse_args() -> argparse.Namespace:
    raise NotImplementedError


def main() -> int:
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit(main())
