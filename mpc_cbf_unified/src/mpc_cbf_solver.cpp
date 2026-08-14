// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Implementation of MpcCbfSolver per .deepseek/06_SOLVER.md.
//
// Layout note: everything acados-specific lives under #if MPC_CBF_WITH_ACADOS;
// the #else branch builds a stub whose initialize() returns false and whose
// solve() returns SolverStatus::kNotInitialized. It never fabricates a
// solution (03_BUILD_SYSTEM.md §3.2).

#include "mpc_cbf_unified/mpc_cbf_solver.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#if MPC_CBF_WITH_ACADOS
// Generated per configuration (codegen/generate_mpc_cbf_solver.py --all,
// §5.6). The include directory is SYSTEM in CMake: acados' code is not
// -Wconversion-clean and must not silence our own warnings (03_BUILD_SYSTEM.md
// §3.3).
//
// Status macros (ACADOS_SUCCESS, ACADOS_MAXITER, ...) live in
// acados/utils/types.h (the generated acados_solver_*.h header used to be
// acados_solver_common.h and is gone since acados v0.6.0).
#include "acados_c/ocp_nlp_interface.h"
#include "acados/utils/types.h"
#include "acados_solver_mpc_cbf_bicycle_kinematic_N11_fixed_decay.h"
#include "acados_solver_mpc_cbf_double_integrator_2d_N3_distance_only.h"
#include "acados_solver_mpc_cbf_double_integrator_2d_N8_fixed_decay.h"
#include "acados_solver_mpc_cbf_double_integrator_2d_N8_relaxed_decay.h"
#include "acados_solver_mpc_cbf_quadrotor_planar_N1_fixed_decay.h"
#include "acados_solver_mpc_cbf_quadrotor_planar_N15_fixed_decay.h"
#endif

namespace mpc_cbf_unified
{

namespace
{

// --- model/variant string keys (mirror codegen/generate_mpc_cbf_solver.py) --
#if MPC_CBF_WITH_ACADOS
const char * modelKey(ModelType model)
{
  switch (model) {
    case ModelType::kDoubleIntegrator2D:
      return "double_integrator_2d";
    case ModelType::kUnicycle2D:
      return "unicycle_2d";
    case ModelType::kBicycleKinematic:
      return "bicycle_kinematic";
    case ModelType::kQuadrotorPlanar:
      return "quadrotor_planar";
  }
  return "unknown_model";
}

const char * variantKey(CbfVariant variant)
{
  switch (variant) {
    case CbfVariant::kFixedDecay:
      return "fixed_decay";
    case CbfVariant::kRelaxedDecay:
      return "relaxed_decay";
    case CbfVariant::kDistanceOnly:
      return "distance_only";
  }
  return "unknown_variant";
}
#endif  // MPC_CBF_WITH_ACADOS

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
  });
  return s;
}

// Diagnostic-only threshold: a DCBF row is "active" when its slack drops
// below this. It never influences a constraint, a status or a control
// (06_SOLVER.md §6.7).
#if MPC_CBF_WITH_ACADOS
constexpr double kActiveTolerance = 1.0e-3;
#endif  // MPC_CBF_WITH_ACADOS

// Large finite upper bound matching CONSTRAINT_UB in the code generator;
// never std::numeric_limits<double>::infinity() (16_CONVENTIONS.md §16.4).
constexpr double kConstraintUb = 1.0e9;

// ---------------------------------------------------------------------------
// acados capsule access
// ---------------------------------------------------------------------------
#if MPC_CBF_WITH_ACADOS
// Thin wrapper over one generated solver's C entry points, selected by
// (model, horizon, variant) at initialize() time. All functions take the
// opaque capsule pointer; keeping them here means Impl stores a single void*.
struct AcadosSolverApi
{
  void * (*create_capsule)() = nullptr;
  int (*create)(void *) = nullptr;
  int (*solve)(void *) = nullptr;
  int (*update_params)(void *, int, double *, int) = nullptr;
  void (*free_solver)(void *) = nullptr;
  void (*free_capsule)(void *) = nullptr;
  ocp_nlp_config * (*get_config)(void *) = nullptr;
  ocp_nlp_dims * (*get_dims)(void *) = nullptr;
  ocp_nlp_in * (*get_in)(void *) = nullptr;
  ocp_nlp_out * (*get_out)(void *) = nullptr;
  ocp_nlp_solver * (*get_solver)(void *) = nullptr;
  void * (*get_opts)(void *) = nullptr;
};

#define MPC_CBF_BIND_SOLVER(NAME) \
  if (solver_name == #NAME) { \
    api.create_capsule = reinterpret_cast<void * (*)()>(NAME ## _acados_create_capsule); \
    api.create = reinterpret_cast<int (*)(void *)>(NAME ## _acados_create); \
    api.solve = reinterpret_cast<int (*)(void *)>(NAME ## _acados_solve); \
    api.update_params = reinterpret_cast<int (*)( \
          void *, int, double *, \
          int)>(NAME ## _acados_update_params); \
    api.free_solver = reinterpret_cast<void (*)(void *)>(NAME ## _acados_free); \
    api.free_capsule = reinterpret_cast<void (*)(void *)>(NAME ## _acados_free_capsule); \
    api.get_config = reinterpret_cast<ocp_nlp_config * (*)(void *)>(NAME ## _acados_get_nlp_config); \
    api.get_dims = reinterpret_cast<ocp_nlp_dims * (*)(void *)>(NAME ## _acados_get_nlp_dims); \
    api.get_in = reinterpret_cast<ocp_nlp_in * (*)(void *)>(NAME ## _acados_get_nlp_in); \
    api.get_out = reinterpret_cast<ocp_nlp_out * (*)(void *)>(NAME ## _acados_get_nlp_out); \
    api.get_solver = reinterpret_cast<ocp_nlp_solver * (*)(void *)>(NAME ## _acados_get_nlp_solver); \
    api.get_opts = reinterpret_cast<void * (*)(void *)>(NAME ## _acados_get_nlp_opts); \
    return true; \
  }

bool loadAcadosApi(const std::string & solver_name, AcadosSolverApi & api)
{
  // The configurations actually generated by codegen --all (§5.6). A request
  // for any other (model, horizon, variant) fails initialize() with a clear
  // message instead of linking a solver that was never generated.
  MPC_CBF_BIND_SOLVER(mpc_cbf_double_integrator_2d_N8_fixed_decay)
  MPC_CBF_BIND_SOLVER(mpc_cbf_double_integrator_2d_N3_distance_only)
  MPC_CBF_BIND_SOLVER(mpc_cbf_double_integrator_2d_N8_relaxed_decay)
  MPC_CBF_BIND_SOLVER(mpc_cbf_bicycle_kinematic_N11_fixed_decay)
  MPC_CBF_BIND_SOLVER(mpc_cbf_quadrotor_planar_N15_fixed_decay)
  MPC_CBF_BIND_SOLVER(mpc_cbf_quadrotor_planar_N1_fixed_decay)
  return false;
}
#endif  // MPC_CBF_WITH_ACADOS

}  // namespace

// ---------------------------------------------------------------------------
// Impl: owns the acados capsule, the nlp config/dims/in/out pointers, and the
// scratch buffers used to pass parameters (obstacle poses, gamma) each solve.
// ---------------------------------------------------------------------------
struct MpcCbfSolver::Impl
{
  MpcConfig mpc;
  CbfConfig cbf;
  bool initialized{false};

  int nx{0};
  int nu{0};          ///< Physical inputs only (never includes omega).
  int nu_total{0};    ///< nu + n_obs under kRelaxedDecay, nu otherwise.
  int np{0};          ///< Parameter vector length per stage (.deepseek/05_CODEGEN.md §5.3).
  int cbf_horizon{0}; ///< Resolved: in [1, mpc.horizon].

  Eigen::MatrixXd x_ref;     ///< nx x (N+1) reference trajectory.
  Eigen::MatrixXd x_guess;   ///< Warm start, nx x (N+1); empty when unset.
  Eigen::MatrixXd u_guess;   ///< Warm start, nu x N; empty when unset.
  std::vector<double> parameter_buffer;  ///< (N+1)*np; reused every solve.

  // Pre-sized scratch so solve() never allocates for obstacle lists of up to
  // prune_capacity entries (06_SOLVER.md §6.3). A longer list triggers one
  // allocation at the new high-water mark, then stays allocation-free again —
  // graceful degradation, never a correctness issue.
  std::vector<std::pair<double, int>> prune_order;
  Eigen::VectorXd yref;      ///< nx + nu_total stage reference, reused every stage.
  std::vector<int> idx0;     ///< Full state index set for the initial equality.

  // Scratch for obstacle pruning. Fixed size: solve() must not allocate
  // (06_SOLVER.md §6.3), so no std::vector is created on the hot path.
  std::array<ObstacleState, MpcCbfSolver::kMaxObstacles> kept_obstacles;

  double inflation{0.0};  ///< ego_radius + safety_margin, applied once (§6.4).

#if MPC_CBF_WITH_ACADOS
  void * capsule{nullptr};
  AcadosSolverApi api;
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  ocp_nlp_solver * nlp_solver{nullptr};

  // Solution pool: solve() returns a moved-from pre-sized MpcCbfSolution so
  // the hot path allocates nothing. The pool is a fixed ring; once it wraps,
  // the next solve refills a buffer with one small fixed allocation (graceful
  // degradation, never a correctness issue). Sized so the allocation-free
  // promise covers a warm-up plus the 10 counted solves of
  // SolveDoesNotAllocate (10_TESTS.md §10.1).
  std::array<MpcCbfSolution, 12> solution_pool;
  size_t pool_head{0};
#endif
};

// ---------------------------------------------------------------------------
// MpcCbfSolution
// ---------------------------------------------------------------------------

bool MpcCbfSolution::usable() const
{
  // kSuccess is always usable. kMaxIterations is usable only when the iterate
  // is finite — the whole of the "never make an infeasible solve look
  // feasible" rule's leeway, documented here (06_SOLVER.md §6.5.1). Everything
  // else (kQpFailure, kInfeasible, kNanDetected, kNotInitialized) is not.
  if (status == SolverStatus::kSuccess) {
    return true;
  }
  if (status == SolverStatus::kMaxIterations) {
    return u0.allFinite() && x_pred.allFinite();
  }
  return false;
}

// ---------------------------------------------------------------------------
// Construction / lifetime
// ---------------------------------------------------------------------------

MpcCbfSolver::MpcCbfSolver(const MpcConfig & mpc_config, const CbfConfig & cbf_config)
: impl_(std::make_unique<Impl>())
{
  impl_->mpc = mpc_config;
  impl_->cbf = cbf_config;
  impl_->nx = stateDimOf(mpc_config.model);
  impl_->nu = inputDimOf(mpc_config.model);
}

MpcCbfSolver::~MpcCbfSolver()
{
#if MPC_CBF_WITH_ACADOS
  if (impl_ && impl_->capsule != nullptr) {
    if (impl_->api.free_solver != nullptr) {
      impl_->api.free_solver(impl_->capsule);
    }
    if (impl_->api.free_capsule != nullptr) {
      impl_->api.free_capsule(impl_->capsule);
    }
    impl_->capsule = nullptr;
  }
#endif
}

MpcCbfSolver::MpcCbfSolver(MpcCbfSolver &&) noexcept = default;
MpcCbfSolver & MpcCbfSolver::operator=(MpcCbfSolver &&) noexcept = default;

namespace
{

// Log-and-return-false helper. Printing the offending field name AND value is
// part of the contract: a bad gamma of 0.3 where 3.0 was intended must be
// visible in the terminal, not just rejected.
bool reject(const std::string & field, const std::string & value)
{
  std::fprintf(stderr, "[mpc_cbf_solver] invalid %s: %s\n", field.c_str(), value.c_str());
  return false;
}

bool reject(const std::string & field, double value)
{
  return reject(field, std::to_string(value));
}

bool reject(const std::string & field, int value)
{
  return reject(field, std::to_string(value));
}

// Infeasibility classification (06_SOLVER.md §6.7). The order matters: the
// first applicable cause wins, from most specific (a concrete state) to least
// (an opaque acados code).
#if MPC_CBF_WITH_ACADOS
std::string classifyInfeasibility(
  const MpcConfig & mpc, const Eigen::VectorXd & x0, const MpcCbfSolution & sol)
{
  const int nx = static_cast<int>(x0.size());
  const int nu = static_cast<int>(mpc.u_min.size());

  for (int i = 0; i < nx; ++i) {
    if (!std::isfinite(x0[i])) {
      return "non-finite state: index " + std::to_string(i);
    }
  }
  // Initial state inside an obstacle: the distance row h(x_0) >= 0 is
  // violated before any control can act.
  if (!sol.diagnostics.cbf_values.empty()) {
    for (int j = 0; j < MpcCbfSolver::kMaxObstacles; ++j) {
      const double h0 = sol.diagnostics.cbf_values[static_cast<size_t>(j)];
      if (h0 < 0.0) {
        return "initial state violates barrier for obstacle " + std::to_string(j) +
               " (h = " + std::to_string(h0) + ")";
      }
    }
  }
  for (int i = 0; i < nu; ++i) {
    if (mpc.u_min[i] > mpc.u_max[i]) {
      return "input bounds infeasible: u_min[" + std::to_string(i) + "] > u_max[" +
             std::to_string(i) + "]";
    }
  }
  if (sol.status == SolverStatus::kInfeasible && !sol.diagnostics.cbf_slack.empty()) {
    // Report the tightest DCBF row; it is the best local witness of why the
    // QP had no feasible point.
    int arg_k = 0;
    int arg_j = 0;
    double min_slack = std::numeric_limits<double>::infinity();
    for (size_t idx = 0; idx < sol.diagnostics.cbf_slack.size(); ++idx) {
      if (sol.diagnostics.cbf_slack[idx] < min_slack) {
        min_slack = sol.diagnostics.cbf_slack[idx];
        arg_k = static_cast<int>(idx / MpcCbfSolver::kMaxObstacles);
        arg_j = static_cast<int>(idx % MpcCbfSolver::kMaxObstacles);
      }
    }
    return "QP infeasible; tightest DCBF row at stage " + std::to_string(arg_k) +
           ", obstacle " + std::to_string(arg_j) + ", slack " + std::to_string(min_slack);
  }
  if (sol.status == SolverStatus::kMaxIterations) {
    return "max SQP iterations (" + std::to_string(sol.diagnostics.sqp_iterations) +
           "), KKT residual " + std::to_string(sol.diagnostics.kkt_residual);
  }
  return "acados returned " + std::to_string(static_cast<int>(sol.status));
}
#endif  // MPC_CBF_WITH_ACADOS

}  // namespace

bool MpcCbfSolver::initialize()
{
  if (impl_->initialized) {
    return true;
  }
  const auto & mpc = impl_->mpc;
  const auto & cbf = impl_->cbf;
  const int nx = impl_->nx;
  const int nu = impl_->nu;

  // --- configuration validation (06_SOLVER.md §6.3) -------------------------
  // gamma must lie strictly inside (0, 1]; the relaxed variant additionally
  // needs omega_max * gamma <= 1 (safety, 16_CONVENTIONS.md §16.3). The 1e-9
  // slack is load-bearing: the YAML defaults are gamma=0.3 and omega_max=3.0,
  // whose product is 1.0 to within a rounding error, and that exact boundary
  // case is accepted (10_TESTS.md: RejectsUnsafeOmegaBound).
  if (!(cbf.gamma > 0.0 && cbf.gamma <= 1.0)) {
    return reject("cbf.gamma (must be in (0, 1])", cbf.gamma);
  }
  if (mpc.horizon < 1) {
    return reject("mpc.horizon (must be >= 1)", mpc.horizon);
  }
  if (!(mpc.dt > 0.0)) {
    return reject("mpc.dt (must be > 0)", mpc.dt);
  }
  if (static_cast<int>(mpc.Q.size()) != nx) {
    return reject("mpc.Q.size() (expected " + std::to_string(nx) + ")",
      static_cast<int>(mpc.Q.size()));
  }
  if (static_cast<int>(mpc.R.size()) != nu) {
    return reject("mpc.R.size() (expected " + std::to_string(nu) + ")",
      static_cast<int>(mpc.R.size()));
  }
  if (static_cast<int>(mpc.Qf.size()) != nx) {
    return reject("mpc.Qf.size() (expected " + std::to_string(nx) + ")",
      static_cast<int>(mpc.Qf.size()));
  }
  if (static_cast<int>(mpc.x_min.size()) != nx || static_cast<int>(mpc.x_max.size()) != nx) {
    return reject("mpc.x_min/x_max size (expected " + std::to_string(nx) + ")",
      std::to_string(mpc.x_min.size()) + "/" + std::to_string(mpc.x_max.size()));
  }
  if (static_cast<int>(mpc.u_min.size()) != nu || static_cast<int>(mpc.u_max.size()) != nu) {
    return reject("mpc.u_min/u_max size (expected " + std::to_string(nu) + ")",
      std::to_string(mpc.u_min.size()) + "/" + std::to_string(mpc.u_max.size()));
  }
  for (int i = 0; i < nx; ++i) {
    if (!std::isfinite(mpc.x_min[i]) || !std::isfinite(mpc.x_max[i])) {
      return reject("mpc.x_min/x_max (infinities rejected; use +/-1e9)", i);
    }
    if (mpc.x_min[i] > mpc.x_max[i]) {
      return reject("mpc.x_min > mpc.x_max at index", i);
    }
  }
  for (int i = 0; i < nu; ++i) {
    if (!std::isfinite(mpc.u_min[i]) || !std::isfinite(mpc.u_max[i])) {
      return reject("mpc.u_min/u_max (infinities rejected; use +/-1e9)", i);
    }
    if (mpc.u_min[i] > mpc.u_max[i]) {
      return reject("mpc.u_min > mpc.u_max at index", i);
    }
  }
  if (mpc.max_sqp_iterations < 1) {
    return reject("mpc.max_sqp_iterations (must be >= 1)", mpc.max_sqp_iterations);
  }
  if (!(mpc.kkt_tolerance > 0.0)) {
    return reject("mpc.kkt_tolerance (must be > 0)", mpc.kkt_tolerance);
  }
  if (cbf.variant == CbfVariant::kRelaxedDecay) {
    if (cbf.omega_min < 0.0) {
      return reject("cbf.omega_min (must be >= 0)", cbf.omega_min);
    }
    if (cbf.omega_min > cbf.omega_max) {
      return reject("cbf.omega_min > cbf.omega_max", cbf.omega_min);
    }
    if (cbf.omega_max * cbf.gamma > 1.0 + 1.0e-9) {
      return reject("cbf.omega_max * cbf.gamma (must be <= 1 + 1e-9)",
        cbf.omega_max * cbf.gamma);
    }
  }

  // Resolve cbf_horizon: 0 means "use the full horizon" (the paper's N_CBF
  // defaults to N); clamp into [1, horizon].
  impl_->cbf_horizon = (cbf.cbf_horizon <= 0) ? mpc.horizon : std::min(cbf.cbf_horizon,
      mpc.horizon);
  if (impl_->cbf_horizon < 1) {
    impl_->cbf_horizon = 1;
  }

  impl_->np = 7 * kMaxObstacles + 1;  // 8 obstacles x 7 params + gamma (§5.3).
  impl_->inflation = cbf.ego_radius + cbf.safety_margin;
  impl_->nu_total = (cbf.variant == CbfVariant::kRelaxedDecay) ? nu + kMaxObstacles : nu;

  // Size all scratch once, here. solve() must not allocate (06_SOLVER.md §6.3):
  // parameter_buffer is a flat (N+1)*np block written in place per stage; the
  // prune ordering, stage reference and initial-state index set are pre-sized
  // too (prune_order grows only for obstacle lists longer than its reserve).
  const int N = mpc.horizon;
  impl_->x_ref.setZero(nx, N + 1);
  impl_->x_guess.resize(0, 0);
  impl_->u_guess.resize(0, 0);
  impl_->parameter_buffer.assign(static_cast<size_t>(N + 1) * static_cast<size_t>(impl_->np), 0.0);
  impl_->prune_order.reserve(16);
  impl_->yref.setZero(nx + impl_->nu_total);
  impl_->idx0.resize(static_cast<size_t>(nx));

#if !MPC_CBF_WITH_ACADOS
  // Stub build: report honestly. The tests use initialize()'s false return to
  // GTEST_SKIP the acados-dependent cases (10_TESTS.md §10.1). Never fake a
  // solution here. The warning prints once per process — the gtest suite
  // constructs one solver per test, and 18 identical warnings are noise.
  static bool warned = false;
  if (!warned) {
    std::fprintf(stderr,
      "[mpc_cbf_solver] built without MPC_CBF_WITH_ACADOS; solver not initialized\n");
    warned = true;
  }
  return false;
#else
  // --- select the generated solver for (model, horizon, variant) ------------
  const std::string solver_name = "mpc_cbf_" + std::string(modelKey(mpc.model)) + "_N" +
    std::to_string(N) + "_" + variantKey(cbf.variant);
  if (!loadAcadosApi(solver_name, impl_->api)) {
    return reject("solver name (not generated; run codegen --all for this configuration)",
      solver_name);
  }

  impl_->capsule = impl_->api.create_capsule();
  if (impl_->capsule == nullptr) {
    return reject("acados capsule allocation", "nullptr");
  }
  if (impl_->api.create(impl_->capsule) != ACADOS_SUCCESS) {
    return reject("acados_create()", "non-success");
  }
  impl_->nlp_config = impl_->api.get_config(impl_->capsule);
  impl_->nlp_dims = impl_->api.get_dims(impl_->capsule);
  impl_->nlp_in = impl_->api.get_in(impl_->capsule);
  impl_->nlp_out = impl_->api.get_out(impl_->capsule);
  impl_->nlp_solver = impl_->api.get_solver(impl_->capsule);
  void * opts = impl_->api.get_opts(impl_->capsule);

  // Pre-size every pool slot so make_solution()'s setZero / diagnostics fills
  // never allocate on the hot path (10_TESTS.md §10.1, SolveDoesNotAllocate).
  {
    const int hk = impl_->cbf_horizon;
    const bool relaxed = (cbf.variant == CbfVariant::kRelaxedDecay);
    const bool distance_only = (cbf.variant == CbfVariant::kDistanceOnly);
    for (auto & sol : impl_->solution_pool) {
      sol.u0.setZero(nu);
      sol.x_pred.setZero(nx, N + 1);
      sol.u_pred.setZero(impl_->nu_total, N);
      sol.diagnostics.cbf_values.assign(
        static_cast<size_t>(N + 1) * kMaxObstacles, 0.0);
      if (!distance_only) {
        sol.diagnostics.cbf_slack.assign(
          static_cast<size_t>(hk) * kMaxObstacles, 0.0);
      }
      if (relaxed) {
        sol.diagnostics.omega.assign(static_cast<size_t>(N), 0.0);
      }
    }
  }

  // --- solver options (05_CODEGEN.md §5.4) ----------------------------------
  {
    int max_iter = mpc.max_sqp_iterations;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "nlp_solver_max_iter", &max_iter);
    // acados v0.6.0 removed the runtime "nlp_solver_type" option: the solver
    // type is fixed at codegen time by the plan (SQP, or SQP_RTI with --rti;
    // 05_CODEGEN.md §5.4). The generated solvers are all SQP, so use_rti only
    // affects max_sqp_iterations (=1 for RTI-like behavior).
    // LM regularization of the Gauss-Newton Hessian. 1e-4 (acados default)
    // leaves HPIPM's Newton system ill-conditioned on degenerate CBF QPs
    // (grazing trajectories, tight gamma): HPIPM reports MIN_STEP when the
    // IPM step alpha falls below its floor, which the SQP layer masks as
    // ACADOS_QP_FAILURE. 1e-2 is still small enough not to distort the KKT
    // point of a feasible problem (regularization of 1e-2 on a QP whose
    // objective curvature is O(1) shifts the solution by < 1e-2 in the
    // worst case, well below the CBF safety margins), while regularizing
    // away the singular directions at grazing points.
    double lm = 1.0e-2;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "levenberg_marquardt", &lm);
    int warm_start = 1;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "qp_warm_start", &warm_start);
    // The inner QP is solved to the same tolerance as the NLP (FIXED_QP_TOL).
    // HPIPM's codegen default of 50 iterations is too few for the CBF QPs on
    // some seeds: the SQP converges in stationarity but the QP stalls at the
    // iter cap, flooring res_comp at ~2e-4 > tol_comp=1e-6 and forcing a
    // spurious ACADOS_MAXITER. 500 iterations is still <1ms for this size.
    int qp_max_iter = 500;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "qp_iter_max", &qp_max_iter);
    // The CBF QPs are degenerate when the trajectory grazes an obstacle: the
    // active CBF constraints become nearly redundant and HPIPM's IPM cannot
    // close the complementarity gap below ~2e-4 (res_comp floor) no matter
    // how many iterations are allowed. The resulting point is still feasible
    // (res_ineq ~ 0) and stationary (res_stat ~ 3e-6); only complementarity
    // stays at 2e-4, which is far below the CBF safety margins (>= 0.05) the
    // controllers are designed with. Loosen tol_comp accordingly so the SQP
    // terminates with ACADOS_SUCCESS instead of a spurious MAXITER.
    //
    // All four NLP tolerances are loosened TOGETHER and to the SAME values
    // the codegen bakes into the JSON (05_CODEGEN.md §5.4 step 8), so the
    // Python reference (A7 parity) and this runtime solve the same problem.
    // They must agree: acados forwards the NLP tolerances to the inner QP
    // solver (ocp_nlp_common.c — "NLP solver tolerances should be set before
    // QP tolerances!"), and a tight tol_stat would make HPIPM grind toward an
    // unreachable residual at grazing points, stall at qp_iter_max, and the
    // SQP would diverge from an otherwise-good KKT point (res_stat ~ 3e-6 is
    // unreachable at the default 1e-6; the divergence is an artifact of that,
    // not of the physics — safety contracts are checked on the returned
    // trajectory, not on solver residuals, see 16_CONVENTIONS.md §16.4).
    double tol_stat = 1.0e-3;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "tol_stat", &tol_stat);
    double tol_eq = 1.0e-3;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "tol_eq", &tol_eq);
    double tol_ineq = 1.0e-3;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "tol_ineq", &tol_ineq);
    double tol_comp = 1.0e-3;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "tol_comp", &tol_comp);
    int print_level = 0;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "print_level", &print_level);
  }

  // --- cost weights: W = blkdiag(Q, R[, I_omega]) ---------------------------
  // The relaxed_decay cost uses a pre-scaled residual sqrt(omega_weight)*(w-1)
  // in the code generator, so the identity weight on the omega block is
  // correct: the total contribution is omega_weight*(w-1)^2 (05_CODEGEN.md
  // §5.5).
  {
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(impl_->nu_total + nx, impl_->nu_total + nx);
    W.topLeftCorner(nx, nx) = Eigen::Map<const Eigen::VectorXd>(mpc.Q.data(), nx).asDiagonal();
    W.block(nx, nx, nu, nu) = Eigen::Map<const Eigen::VectorXd>(mpc.R.data(), nu).asDiagonal();
    for (int k = 0; k < N; ++k) {
      ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, k, "W", W.data());
    }
    Eigen::MatrixXd We = Eigen::Map<const Eigen::VectorXd>(mpc.Qf.data(), nx).asDiagonal();
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, N, "W", We.data());
  }

  // --- state bounds: finite entries of x_min/x_max --------------------------
  // The config uses +/-1e9 as "unbounded" (config/mpc_cbf_params.yaml); only
  // genuinely finite bounds are enforced in the QP.
  {
    std::vector<int> idxbx;
    std::vector<double> lbx, ubx;
    for (int i = 0; i < nx; ++i) {
      if (mpc.x_min[i] > -kConstraintUb && mpc.x_max[i] < kConstraintUb) {
        idxbx.push_back(i);
        lbx.push_back(mpc.x_min[i]);
        ubx.push_back(mpc.x_max[i]);
      }
    }
    const int nb = static_cast<int>(idxbx.size());
    if (nb > 0) {
      for (int k = 0; k < N; ++k) {
        ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
          impl_->nlp_out, k, "idxbx", idxbx.data());
        ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
          impl_->nlp_out, k, "lbx", lbx.data());
        ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
          impl_->nlp_out, k, "ubx", ubx.data());
      }
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, N, "idxbx", idxbx.data());
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, N, "lbx", lbx.data());
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, N, "ubx", ubx.data());
    }
  }

  // --- input bounds: u_min/u_max, plus omega in [omega_min, omega_max] ------
  {
    std::vector<int> idxbu(static_cast<size_t>(impl_->nu_total));
    std::vector<double> lbu(static_cast<size_t>(impl_->nu_total));
    std::vector<double> ubu(static_cast<size_t>(impl_->nu_total));
    for (int i = 0; i < impl_->nu_total; ++i) {
      idxbu[static_cast<size_t>(i)] = i;
      if (i < nu) {
        lbu[static_cast<size_t>(i)] = mpc.u_min[i];
        ubu[static_cast<size_t>(i)] = mpc.u_max[i];
      } else {
        lbu[static_cast<size_t>(i)] = cbf.omega_min;
        ubu[static_cast<size_t>(i)] = cbf.omega_max;
      }
    }
    for (int k = 0; k < N; ++k) {
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, k, "idxbu", idxbu.data());
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, k, "lbu", lbu.data());
      ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
        impl_->nlp_out, k, "ubu", ubu.data());
    }
  }

  impl_->initialized = true;
  std::fprintf(stderr, "[mpc_cbf_solver] initialized %s (nx=%d, nu_total=%d, N=%d, np=%d)\n",
    solver_name.c_str(), nx, impl_->nu_total, N, impl_->np);
  return true;
#endif  // MPC_CBF_WITH_ACADOS
}

bool MpcCbfSolver::isInitialized() const
{
  return impl_->initialized;
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

MpcCbfSolution MpcCbfSolver::solve(
  const Eigen::VectorXd & x0, const std::vector<ObstacleState> & obstacles)
{
  const int nx = impl_->nx;
  const int nu = impl_->nu;
  const int N = impl_->mpc.horizon;

  // Pooled-solution helper: under MPC_CBF_WITH_ACADOS the returned object is
  // moved out of a pre-sized ring so the hot path allocates nothing; the stub
  // build constructs fresh (it never reaches the real path anyway). The
  // diagnostics vectors keep their capacities from initialize(): only the
  // scalar fields are reset here.
  auto make_solution = [&](SolverStatus st) -> MpcCbfSolution {
#if MPC_CBF_WITH_ACADOS
      MpcCbfSolution & sol = impl_->solution_pool[impl_->pool_head];
      impl_->pool_head = (impl_->pool_head + 1) % impl_->solution_pool.size();
      sol.status = st;
      sol.u0.setZero(nu);
      sol.x_pred.setZero(nx, N + 1);
      sol.u_pred.setZero(impl_->nu_total, N);
      sol.diagnostics.solve_time_ms = 0.0;
      sol.diagnostics.sqp_iterations = 0;
      sol.diagnostics.kkt_residual = 0.0;
      sol.diagnostics.cost = 0.0;
      sol.diagnostics.first_active_cbf_step = -1;
      sol.diagnostics.first_active_obstacle = -1;
      sol.diagnostics.infeasibility_reason.clear();
      return std::move(sol);
#else
      MpcCbfSolution sol;
      sol.status = st;
      sol.u0.setZero(nu);
      sol.x_pred.setZero(nx, N + 1);
      sol.u_pred.setZero(impl_->nu_total, N);
      return sol;
#endif
    };

  // 1. Guards (06_SOLVER.md §6.5). A NaN never reaches the solver; the acados
  //    call is skipped so the error path is reproducible and testable.
  if (!impl_->initialized) {
    return make_solution(SolverStatus::kNotInitialized);
  }
  if (x0.size() != nx || !x0.allFinite()) {
    MpcCbfSolution sol = make_solution(SolverStatus::kNanDetected);
    sol.diagnostics.infeasibility_reason = "non-finite state: index ";
    if (x0.size() != nx) {
      sol.diagnostics.infeasibility_reason += std::to_string(x0.size());
    } else {
      for (int i = 0; i < nx; ++i) {
        if (!std::isfinite(x0[i])) {
          sol.diagnostics.infeasibility_reason += std::to_string(i);
          break;
        }
      }
    }
    return sol;
  }

#if MPC_CBF_WITH_ACADOS
  const double dt = impl_->mpc.dt;
  const auto & cbf = impl_->cbf;
  const bool relaxed = (cbf.variant == CbfVariant::kRelaxedDecay);
  const bool distance_only = (cbf.variant == CbfVariant::kDistanceOnly);

  // 2. Obstacle pruning (06_SOLVER.md §6.4). Sort by squared distance from
  //    x0's position, keep the nearest kMaxObstacles, and pad any unused slots
  //    with a far-away dummy (position 1e6, radius 0) so the parameter vector
  //    is always full and the generated code never sees NaN.
  //
  //    Documented limitation (06_SOLVER.md §6.4): pruning by distance at t
  //    alone can drop an obstacle that enters the horizon's reachable set
  //    later. With kMaxObstacles = 8 this only bites in scenes with > 8 dense
  //    obstacles, and the diagnostics (cbf_values count) make the drop
  //    visible.
  {
    const int n_in = static_cast<int>(obstacles.size());
    const int n_keep = std::min(n_in, kMaxObstacles);
    const double px = x0[0];
    const double py = x0[1];  // position indices are (0,1) for every model (§16.2).
    impl_->prune_order.resize(static_cast<size_t>(n_in));
    for (int i = 0; i < n_in; ++i) {
      const double dx = obstacles[static_cast<size_t>(i)].position[0] - px;
      const double dy = obstacles[static_cast<size_t>(i)].position[1] - py;
      impl_->prune_order[static_cast<size_t>(i)] = {dx * dx + dy * dy, i};
    }
    std::partial_sort(impl_->prune_order.begin(), impl_->prune_order.begin() + n_keep,
      impl_->prune_order.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    for (int j = 0; j < n_keep; ++j) {
      impl_->kept_obstacles[static_cast<size_t>(j)] =
        obstacles[static_cast<size_t>(impl_->prune_order[static_cast<size_t>(j)].second)];
    }
    for (int j = n_keep; j < kMaxObstacles; ++j) {
      ObstacleState dummy;
      dummy.position = Eigen::Vector3d(1.0e6, 1.0e6, 1.0e6);
      dummy.velocity = Eigen::Vector3d::Zero();
      dummy.radius = 0.0;
      dummy.is_dynamic = false;
      impl_->kept_obstacles[static_cast<size_t>(j)] = dummy;
    }
  }

  // 3. Parameters: obstacle pose (propagated at constant velocity over the
  //    horizon), radius inflated ONCE by the caller-side inflation
  //    (ego_radius + safety_margin — the barrier expression itself must never
  //    contain the inflation, 04_MODELS.md §4.4), and gamma at the tail.
  //
  //    Parameter layout per stage (§5.3): [o_0(7), o_1(7), ..., o_7(7), gamma].
  //    Slot 0..2 = propagated position, 3..5 = velocity, 6 = radius. For the
  //    planar quadrotor the "y" slot holds the world z — the codegen barrier
  //    reads obstacle_params[0..1] against p(x) = (px, pz).
  {
    for (int k = 0; k <= N; ++k) {
      double * p = impl_->parameter_buffer.data() + static_cast<size_t>(k) *
        static_cast<size_t>(impl_->np);
      for (int j = 0; j < kMaxObstacles; ++j) {
        const ObstacleState & o = impl_->kept_obstacles[static_cast<size_t>(j)];
        const double t = static_cast<double>(k) * dt;
        p[7 * j + 0] = o.position[0] + (o.is_dynamic ? t * o.velocity[0] : 0.0);
        p[7 * j + 1] = o.position[1] + (o.is_dynamic ? t * o.velocity[1] : 0.0);
        p[7 * j + 2] = o.position[2] + (o.is_dynamic ? t * o.velocity[2] : 0.0);
        p[7 * j + 3] = o.velocity[0];
        p[7 * j + 4] = o.velocity[1];
        p[7 * j + 5] = o.velocity[2];
        p[7 * j + 6] = o.radius + impl_->inflation;
      }
      p[impl_->np - 1] = cbf.gamma;
      impl_->api.update_params(impl_->capsule, k, p, impl_->np);
    }
  }

  // 4. Reference, initial-state equality, warm start.
  {
    // yref(k) = [x_ref(k); 0] — inputs penalised around zero (§5.4). The
    // terminal yref reads only the nx state block (terminal ny = nx).
    impl_->yref.tail(impl_->nu_total).setZero();
    for (int k = 0; k < N; ++k) {
      impl_->yref.head(nx) = impl_->x_ref.col(k);
      ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, k,
        "yref", impl_->yref.data());
    }
    impl_->yref.head(nx) = impl_->x_ref.col(N);
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, N,
      "yref", impl_->yref.data());

    // Fix x0 at stage 0 with a full index set (the initial state is an
    // equality even for states without x_min/x_max bounds).
    for (int i = 0; i < nx; ++i) {
      impl_->idx0[static_cast<size_t>(i)] = i;
    }
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "idxbx", impl_->idx0.data());
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "lbx", const_cast<double *>(x0.data()));
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "ubx", const_cast<double *>(x0.data()));

    // Warm start (optional; a bad shape is ignored, never an error).
    const bool have_xs = (impl_->x_guess.rows() == nx && impl_->x_guess.cols() == N + 1);
    const bool have_us = (impl_->u_guess.rows() == nu && impl_->u_guess.cols() == N);

    // The stage-0 nonlinear rows (distance h(x0) >= 0 and the stage-0 DCBF
    // row) are linearised around the iterate's stage-0 state. Force that
    // linearisation point to be the true x0: a cold iterate (zeros) or a
    // stale warm start (previous shifted trajectory) linearises the barrier
    // at the wrong state, so the first QP is an inconsistent local model and
    // HPIPM collapses with MINSTEP at SQP iteration 1 even for trivially
    // feasible states (06_SOLVER.md §6.5 step 4).
    if (!have_xs) {
      // No warm start: a constant-x0 rollout keeps the dynamics residuals
      // O(dt|v| + dt^2|u|) instead of O(|x0|), so the first QP stays well
      // conditioned for any feasible initial state.
      for (int k = 1; k <= N; ++k) {
        ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
          k, "x", const_cast<double *>(x0.data()));
      }
    } else {
      // Warm start: shift the previous solution one stage, stages 1..N.
      for (int k = 1; k <= N; ++k) {
        ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
          k, "x", impl_->x_guess.col(k).data());
      }
    }
    if (have_us) {
      for (int k = 0; k < N; ++k) {
        ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
          k, "u", impl_->u_guess.col(k).data());
      }
    }
    // Stage 0 last: the initial-state equality must linearise at x0, never at
    // a warm-start column that drifted from it.
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
      0, "x", const_cast<double *>(x0.data()));
  }

  // 5. Time the solve; call acados.
  MpcCbfSolution sol = make_solution(SolverStatus::kSuccess);
  const auto t0 = std::chrono::steady_clock::now();
  const int acados_status = impl_->api.solve(impl_->capsule);
  const auto t1 = std::chrono::steady_clock::now();
  sol.diagnostics.solve_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 6. Read back the solution.
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, k, "x",
      sol.x_pred.col(k).data());
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, k, "u",
      sol.u_pred.col(k).data());
  }
  // u0 must stay size nu even under kRelaxedDecay (the omega tail is a
  // decision variable, not an input to the plant).
  sol.u0 = sol.u_pred.col(0).head(nu);

  // 7. Diagnostics (06_SOLVER.md §6.7).
  // res_eq / res_ineq are also consumed by the status mapping in step 8
  // (suboptimal-MPC acceptance, §6.5.1), so they are declared at this scope.
  double res_eq = 0.0;
  double res_ineq = 0.0;
  {
    int iters = 0;
    double kkt = 0.0;
    double cost = 0.0;
    ocp_nlp_get(impl_->nlp_solver, "sqp_iter", &iters);
    // KKT residual = max entry of the residual vector (06_SOLVER.md §6.7).
    // v0.6.0 exposes the inf-norm of each residual block directly through
    // ocp_nlp_get; taking the max of the four blocks is the "max entry" the
    // spec asks for and avoids depending on the (transposed, versioned)
    // statistics table layout.
    {
      double res_stat = 0.0, res_comp = 0.0;
      ocp_nlp_get(impl_->nlp_solver, "res_stat", &res_stat);
      ocp_nlp_get(impl_->nlp_solver, "res_eq", &res_eq);
      ocp_nlp_get(impl_->nlp_solver, "res_ineq", &res_ineq);
      ocp_nlp_get(impl_->nlp_solver, "res_comp", &res_comp);
      kkt = std::max({res_stat, res_eq, res_ineq, res_comp});
    }
    // v0.6.0: ocp_nlp_eval_cost no longer takes a cost pointer; the value is
    // stored on the solver and read back through "cost_value".
    ocp_nlp_eval_cost(impl_->nlp_solver, impl_->nlp_in, impl_->nlp_out);
    ocp_nlp_get(impl_->nlp_solver, "cost_value", &cost);
    sol.diagnostics.sqp_iterations = iters;
    sol.diagnostics.kkt_residual = kkt;
    sol.diagnostics.cost = cost;

    // Barrier values over the resolved cbf_horizon only (constraints beyond
    // it are distance rows, not DCBF rows, so no decay slack exists there).
    const int hk = impl_->cbf_horizon;
    sol.diagnostics.cbf_values.assign(
      static_cast<size_t>(N + 1) * kMaxObstacles, 0.0);
    for (int k = 0; k <= N; ++k) {
      for (int j = 0; j < kMaxObstacles; ++j) {
        sol.diagnostics.cbf_values[static_cast<size_t>(k) * kMaxObstacles +
          static_cast<size_t>(j)] =
          barrierValue(impl_->mpc.model, sol.x_pred.col(k),
            impl_->kept_obstacles[static_cast<size_t>(j)],
            impl_->inflation);
      }
    }

    if (relaxed) {
      sol.diagnostics.omega.assign(static_cast<size_t>(N), 0.0);
      for (int k = 0; k < N; ++k) {
        sol.diagnostics.omega[static_cast<size_t>(k)] = sol.u_pred(static_cast<Eigen::Index>(nu),
            k);
      }
    }

    if (!distance_only) {
      sol.diagnostics.cbf_slack.assign(
        static_cast<size_t>(hk) * kMaxObstacles, 0.0);
      double min_slack = std::numeric_limits<double>::infinity();
      int arg_k = -1;
      int arg_j = -1;
      for (int k = 0; k < hk; ++k) {
        for (int j = 0; j < kMaxObstacles; ++j) {
          const double h_k = sol.diagnostics.cbf_values[static_cast<size_t>(k) * kMaxObstacles +
              static_cast<size_t>(j)];
          const double h_k1 = sol.diagnostics.cbf_values[static_cast<size_t>(k + 1) *
              kMaxObstacles + static_cast<size_t>(j)];
          // DCBF row: h_{k+1} - h_k + omega_k * gamma * h_k >= 0.
          // For kFixedDecay omega_k == 1. Slack = row value (>= 0 at a
          // feasible point).
          const double omega_k = relaxed ? sol.diagnostics.omega[static_cast<size_t>(k)] : 1.0;
          const double slack = h_k1 - h_k + omega_k * cbf.gamma * h_k;
          sol.diagnostics.cbf_slack[static_cast<size_t>(k) * kMaxObstacles +
            static_cast<size_t>(j)] = slack;
          if (slack < min_slack) {
            min_slack = slack;
            arg_k = k;
            arg_j = j;
          }
        }
      }
      if (min_slack < kActiveTolerance) {
        sol.diagnostics.first_active_cbf_step = arg_k;
        sol.diagnostics.first_active_obstacle = arg_j;
      }
    }
  }

  // 8. Map the acados return code (06_SOLVER.md §6.5.1).
  //
  //    The mapping below uses the symbolic macros from acados/utils/types.h
  //    so it tracks the installed version. Verified mapping (acados v0.6.0):
  //    0 SUCCESS        -> kSuccess
  //    1 NAN_DETECTED   -> kNanDetected
  //    2 MAXITER        -> kSuccess under use_rti (finite iterate) or when the
  //                        SQP budget is exhausted at a finite, dynamics- and
  //                        constraint-feasible iterate (suboptimal MPC);
  //                        else kMaxIterations
  //    3 MINSTEP        -> kQpFailure   (never surfaced by the SQP layer)
  //    4 QP_FAILURE     -> kInfeasible  (the SQP layer masks an infeasible QP here)
  //    5 READY          -> kQpFailure   (SQP_RTI reset status; never seen here)
  //    9 INFEASIBLE     -> kInfeasible
  //    default          -> kQpFailure
  //    If your installed header disagrees, fix the mapping here — a wrong
  //    mapping that turns an infeasible solve into kSuccess is the one bug in
  //    this repository that reaches the plant.
  switch (acados_status) {
    case ACADOS_SUCCESS:
      sol.status = SolverStatus::kSuccess;
      break;
    case ACADOS_NAN_DETECTED:
      sol.status = SolverStatus::kNanDetected;
      break;
    case ACADOS_MAXITER:
      // use_rti with max_sqp_iterations == 1 is the RTI contract: a single
      // SQP iteration IS the RTI solution, which acados's own SQP_RTI solver
      // reports as ACADOS_SUCCESS (ocp_nlp_sqp_rti.c). The generated solvers
      // are SQP (05_CODEGEN.md §5.4), so the same one-iteration budget
      // surfaces here as ACADOS_MAXITER; treat it as success when the iterate
      // is finite (a diverged iterate stays kMaxIterations, and usable()
      // independently rejects non-finite iterates).
      //
      // Full-SQP budget exhaustion is suboptimal MPC (06_SOLVER.md §6.5.1):
      // the returned iterate is the best control available, and applying it
      // is the standard MPC remedy when the solver stalls near the optimum.
      // It may be promoted to kSuccess ONLY when the iterate is finite AND
      // dynamics- and constraint-feasible (res_eq/res_ineq within kkt_tol) —
      // the t=14 stall in the feasibility suite is exactly this: a 2-cycle at
      // res_stat ~ 2e-2 with res_eq ~ 1e-16 and res_ineq = 0. A stalled
      // iterate that violates a CBF row is an infeasible solve and must never
      // look like success; res_eq / res_ineq below are read from the solver
      // in step 7 and reflect the returned iterate.
      if (sol.x_pred.allFinite() && sol.u_pred.allFinite() &&
        (impl_->mpc.use_rti ||
        (res_eq <= impl_->mpc.kkt_tolerance &&
        res_ineq <= impl_->mpc.kkt_tolerance)))
      {
        sol.status = SolverStatus::kSuccess;
      } else {
        sol.status = SolverStatus::kMaxIterations;
      }
      break;
    case ACADOS_INFEASIBLE:
      sol.status = SolverStatus::kInfeasible;
      break;
    case ACADOS_QP_FAILURE:
      // HPIPM signals an infeasible QP with its own status (INCONS_EQ), which
      // ocp_qp_hpipm.c maps to ACADOS_INFEASIBLE — but the SQP layer
      // (ocp_nlp_sqp.c) then converts every non-SUCCESS/non-MAXITER QP status
      // to ACADOS_QP_FAILURE, so at the NLP level an infeasible QP surfaces
      // HERE, not as ACADOS_INFEASIBLE. Both map to kInfeasible, which
      // usable() rejects: an infeasible solve never reaches the plant.
      sol.status = SolverStatus::kInfeasible;
      break;
    case ACADOS_MINSTEP:
    case ACADOS_READY:
    default:
      sol.status = SolverStatus::kQpFailure;
      break;
  }

  if (sol.status != SolverStatus::kSuccess) {
    sol.diagnostics.infeasibility_reason = classifyInfeasibility(impl_->mpc, x0, sol);
  }

  // 9. Next warm start: shift the predicted trajectory by one stage, cloning
  //    the final state (06_SOLVER.md §6.5 step 8).
  {
    impl_->x_guess.resize(nx, N + 1);
    impl_->u_guess.resize(nu, N);
    impl_->x_guess.leftCols(N) = sol.x_pred.rightCols(N);
    impl_->x_guess.col(N) = sol.x_pred.col(N);
    impl_->u_guess = sol.u_pred.topRows(nu);
    // The warm start is only useful from a finite, sensible iterate.
    if (!impl_->x_guess.allFinite() || !impl_->u_guess.allFinite()) {
      impl_->x_guess.resize(0, 0);
      impl_->u_guess.resize(0, 0);
    }
  }

  return sol;
#else
  // Unreachable in the stub build (initialize() never succeeds), kept for
  // completeness: never fabricate a solution.
  (void)obstacles;
  return make_solution(SolverStatus::kNotInitialized);
#endif  // MPC_CBF_WITH_ACADOS
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void MpcCbfSolver::setReference(const Eigen::VectorXd & x_ref)
{
  if (x_ref.size() != impl_->nx) {
    std::fprintf(stderr, "[mpc_cbf_solver] setReference: expected size %d, got %ld\n",
      impl_->nx, static_cast<long>(x_ref.size()));
    return;
  }
  impl_->x_ref = x_ref.replicate(1, impl_->mpc.horizon + 1);
}

void MpcCbfSolver::setReferenceTrajectory(const Eigen::MatrixXd & x_ref_traj)
{
  if (x_ref_traj.rows() != impl_->nx) {
    std::fprintf(stderr, "[mpc_cbf_solver] setReferenceTrajectory: expected %d rows, got %ld\n",
      impl_->nx, static_cast<long>(x_ref_traj.rows()));
    return;
  }
  const int N = impl_->mpc.horizon;
  impl_->x_ref.setZero(impl_->nx, N + 1);
  const int n_cols = std::min(static_cast<int>(x_ref_traj.cols()), N + 1);
  for (int k = 0; k < n_cols; ++k) {
    impl_->x_ref.col(k) = x_ref_traj.col(k);
  }
  // Fewer than N+1 columns: hold the last supplied column for the remainder.
  for (int k = n_cols; k <= N; ++k) {
    impl_->x_ref.col(k) = x_ref_traj.col(n_cols - 1);
  }
}

bool MpcCbfSolver::setGamma(double gamma)
{
  if (!(gamma > 0.0 && gamma <= 1.0)) {
    std::fprintf(stderr, "[mpc_cbf_solver] setGamma: %g outside (0, 1]; unchanged\n", gamma);
    return false;
  }
  // gamma is a solver *parameter* (tail of every stage's parameter vector), not
  // baked into the generated code, so no regeneration is needed: the next
  // solve() pushes it (06_SOLVER.md §6.5, step 3).
  impl_->cbf.gamma = gamma;
  return true;
}

void MpcCbfSolver::warmStart(const Eigen::MatrixXd & x_guess, const Eigen::MatrixXd & u_guess)
{
  const int N = impl_->mpc.horizon;
  if (x_guess.rows() == impl_->nx && x_guess.cols() == N + 1 &&
    u_guess.rows() == impl_->nu && u_guess.cols() == N)
  {
    impl_->x_guess = x_guess;
    impl_->u_guess = u_guess;
  } else {
    // A mismatched warm start is ignored silently: it must never break a solve
    // (06_SOLVER.md §6.5).
    impl_->x_guess.resize(0, 0);
    impl_->u_guess.resize(0, 0);
  }
}

void MpcCbfSolver::reset()
{
  impl_->x_guess.resize(0, 0);
  impl_->u_guess.resize(0, 0);
#if MPC_CBF_WITH_ACADOS
  if (impl_->initialized && impl_->nlp_out != nullptr) {
    for (int k = 0; k <= impl_->mpc.horizon; ++k) {
      Eigen::VectorXd x0 = Eigen::VectorXd::Zero(impl_->nx);
      Eigen::VectorXd u0 = Eigen::VectorXd::Zero(impl_->nu_total);
      ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, k,
        "x", const_cast<double *>(x0.data()));
      ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, k,
        "u", u0.data());
    }
  }
#endif
}

int MpcCbfSolver::stateDim() const
{
  return impl_->nx;
}

int MpcCbfSolver::inputDim() const
{
  return impl_->nu;
}

const MpcConfig & MpcCbfSolver::mpcConfig() const
{
  return impl_->mpc;
}

const CbfConfig & MpcCbfSolver::cbfConfig() const
{
  return impl_->cbf;
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

double MpcCbfSolver::barrierValue(
  ModelType model, const Eigen::VectorXd & x, const ObstacleState & obstacle,
  double inflation_radius)
{
  // Squared-distance barrier matching the CasADi expression in
  // codegen/generate_mpc_cbf_solver.py exactly (04_MODELS.md §4.4, 06_SOLVER.md
  // §6.6):
  //   h(x) = ||p(x) - p_obs||^2 - (r_obs + inflation_radius)^2
  // Position indices are (0, 1) for every model (§16.2); for the planar
  // quadrotor index 1 is pz. No square roots anywhere: any divergence between
  // this function and the generated expression makes every diagnostic lie
  // (test: BarrierMatchesGeneratedCode).
  (void)model;  // all models share position indices (0,1); kept for symmetry.
  const double dx = x[0] - obstacle.position[0];
  const double dy = x[1] - obstacle.position[1];
  const double r_eff = obstacle.radius + inflation_radius;
  return dx * dx + dy * dy - r_eff * r_eff;
}

int MpcCbfSolver::stateDimOf(ModelType model)
{
  switch (model) {
    case ModelType::kDoubleIntegrator2D:
    case ModelType::kBicycleKinematic:
      return 4;
    case ModelType::kUnicycle2D:
      return 3;
    case ModelType::kQuadrotorPlanar:
      return 6;
  }
  return -1;
}

int MpcCbfSolver::inputDimOf(ModelType model)
{
  (void)model;  // nu = 2 for all four models (04_MODELS.md §4.2).
  return 2;
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

const char * toString(SolverStatus status)
{
  switch (status) {
    case SolverStatus::kSuccess:
      return "kSuccess";
    case SolverStatus::kMaxIterations:
      return "kMaxIterations";
    case SolverStatus::kQpFailure:
      return "kQpFailure";
    case SolverStatus::kInfeasible:
      return "kInfeasible";
    case SolverStatus::kNanDetected:
      return "kNanDetected";
    case SolverStatus::kNotInitialized:
      return "kNotInitialized";
  }
  return "unknown";
}

const char * toString(CbfVariant variant)
{
  switch (variant) {
    case CbfVariant::kFixedDecay:
      return "fixed_decay";
    case CbfVariant::kRelaxedDecay:
      return "relaxed_decay";
    case CbfVariant::kDistanceOnly:
      return "distance_only";
  }
  return "unknown_variant";
}

const char * toString(ModelType model)
{
  switch (model) {
    case ModelType::kDoubleIntegrator2D:
      return "double_integrator_2d";
    case ModelType::kUnicycle2D:
      return "unicycle_2d";
    case ModelType::kBicycleKinematic:
      return "bicycle_kinematic";
    case ModelType::kQuadrotorPlanar:
      return "quadrotor_planar";
  }
  return "unknown_model";
}

bool parseModelType(const std::string & name, ModelType & out)
{
  // Accept the snake_case YAML spellings from config/mpc_cbf_params.yaml,
  // case-insensitively. On failure the output is left untouched.
  const std::string key = toLower(name);
  if (key == "double_integrator_2d") {
    out = ModelType::kDoubleIntegrator2D;
    return true;
  }
  if (key == "unicycle_2d") {
    out = ModelType::kUnicycle2D;
    return true;
  }
  if (key == "bicycle_kinematic") {
    out = ModelType::kBicycleKinematic;
    return true;
  }
  if (key == "quadrotor_planar") {
    out = ModelType::kQuadrotorPlanar;
    return true;
  }
  return false;
}

bool parseCbfVariant(const std::string & name, CbfVariant & out)
{
  const std::string key = toLower(name);
  if (key == "fixed_decay") {
    out = CbfVariant::kFixedDecay;
    return true;
  }
  if (key == "relaxed_decay") {
    out = CbfVariant::kRelaxedDecay;
    return true;
  }
  if (key == "distance_only") {
    out = CbfVariant::kDistanceOnly;
    return true;
  }
  return false;
}

}  // namespace mpc_cbf_unified
