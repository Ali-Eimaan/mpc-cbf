// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// SKELETON — declarations only. Every method is specified by its doc comment;
// no bodies live in this header. See .deepseek/06_SOLVER.md.
//
// Discrete-time MPC with discrete-time control barrier function (DCBF)
// constraints, following:
//   [1] J. Zeng, B. Zhang, K. Sreenath, "Safety-Critical Model Predictive
//       Control with Discrete-Time Control Barrier Function", ACC 2021.
//   [2] J. Zeng, Z. Li, K. Sreenath, "Enhancing Feasibility and Safety of
//       Nonlinear Model Predictive Control with Discrete-Time Control Barrier
//       Functions", CDC 2021.

#ifndef MPC_CBF_UNIFIED__MPC_CBF_SOLVER_HPP_
#define MPC_CBF_UNIFIED__MPC_CBF_SOLVER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

namespace mpc_cbf_unified
{

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Terminal status of one call to MpcCbfSolver::solve().
enum class SolverStatus
{
  kSuccess = 0,        ///< QP/NLP converged within tolerance.
  kMaxIterations,      ///< Iteration budget exhausted; iterate may still be usable.
  kQpFailure,          ///< Underlying QP solver failed (HPIPM/qpOASES error).
  kInfeasible,         ///< Constraints proven inconsistent at the current state.
  kNanDetected,        ///< Non-finite value in the iterate or in the data.
  kNotInitialized      ///< initialize() not called, or built without acados.
};

/// Prediction model used by the solver. Selects which acados-generated solver
/// is loaded and fixes (nx, nu). See .deepseek/06_SOLVER.md §6.2.
enum class ModelType
{
  kDoubleIntegrator2D = 0,  ///< nx=4  x=[px,py,vx,vy],      nu=2 u=[ax,ay]
  kUnicycle2D,              ///< nx=3  x=[px,py,theta],      nu=2 u=[v,omega]
  kBicycleKinematic,        ///< nx=4  x=[px,py,theta,v],    nu=2 u=[a,delta]
  kQuadrotorPlanar          ///< nx=6  x=[px,pz,vx,vz,phi,phidot], nu=2 u=[T,tau]
};

/// Which DCBF formulation the solver enforces over the horizon.
enum class CbfVariant
{
  kFixedDecay = 0,   ///< [1]  h(x_{k+1}) - h(x_k) >= -gamma * h(x_k)
  kRelaxedDecay,     ///< [2]  h(x_{k+1}) - h(x_k) >= -omega_k * gamma * h(x_k)
  kDistanceOnly      ///< MPC-DC baseline: h(x_k) >= 0 only, no decay condition.
};

// ---------------------------------------------------------------------------
// Plain data types
// ---------------------------------------------------------------------------

/// One circular/spherical obstacle in the world frame. Only the first
/// `kMaxObstacles` entries handed to solve() are passed to the solver; the
/// remainder are dropped after the distance-based pruning described in §6.4.
struct ObstacleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};   ///< [m]; z ignored for 2-D models.
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};   ///< [m/s]; constant-velocity prediction.
  double radius{0.0};                                  ///< [m] obstacle radius.
  bool is_dynamic{false};                              ///< If false, velocity is ignored.
};

/// Tuning of the DCBF constraints. Mirrors config/mpc_cbf_params.yaml.
struct CbfConfig
{
  CbfVariant variant{CbfVariant::kFixedDecay};
  double gamma{0.3};            ///< Linear class-K gain, must lie in (0, 1].
  int cbf_horizon{0};           ///< #DCBF constraints; 0 or <0 means "= mpc horizon".
  double ego_radius{0.0};       ///< [m] inflates every obstacle radius.
  double safety_margin{0.0};    ///< [m] extra inflation on top of ego_radius.

  // kRelaxedDecay only (CDC 2021 decision-variable decay rate).
  double omega_min{0.0};        ///< Lower bound on omega_k; >= 0.
  double omega_max{1.0};        ///< Upper bound; safety needs omega_max * gamma <= 1.
  double omega_weight{1.0e3};   ///< Penalty on (omega_k - 1)^2 in the stage cost.
  bool omega_decay_per_step{false};  ///< If true use gamma^k profile from [2] §III.C.
};

/// Tuning of the MPC part. Mirrors config/mpc_cbf_params.yaml.
struct MpcConfig
{
  ModelType model{ModelType::kDoubleIntegrator2D};
  int horizon{8};                     ///< Prediction horizon N (steps).
  double dt{0.1};                     ///< [s] discretisation step.

  Eigen::VectorXd Q;                  ///< size nx, stage state weights (diagonal).
  Eigen::VectorXd R;                  ///< size nu, stage input weights (diagonal).
  Eigen::VectorXd Qf;                 ///< size nx, terminal state weights (diagonal).

  Eigen::VectorXd x_min;              ///< size nx; use -inf entries where unbounded.
  Eigen::VectorXd x_max;              ///< size nx.
  Eigen::VectorXd u_min;              ///< size nu.
  Eigen::VectorXd u_max;              ///< size nu.

  int max_sqp_iterations{20};         ///< SQP iteration cap (1 => RTI).
  double kkt_tolerance{1.0e-6};
  bool use_rti{false};                ///< Real-time-iteration scheme for the ROS node.
};

/// Per-solve introspection. The point of this struct is answering "which
/// constraint went active/infeasible, and when" without a debugger attached.
struct SolverDiagnostics
{
  double solve_time_ms{0.0};
  int sqp_iterations{0};
  double kkt_residual{0.0};
  double cost{0.0};

  /// h(x_{k|t}) for k = 0..N, flattened obstacle-major: index k*n_obs + j.
  std::vector<double> cbf_values;
  /// Slack on each DCBF constraint (0 when strictly satisfied), same layout.
  std::vector<double> cbf_slack;
  /// Realised omega_k, empty unless CbfVariant::kRelaxedDecay.
  std::vector<double> omega;

  /// Horizon index of the first DCBF constraint that is active (within
  /// `active_tolerance`), or -1 if none. Set even on success.
  int first_active_cbf_step{-1};
  /// Obstacle index owning `first_active_cbf_step`, or -1.
  int first_active_obstacle{-1};
  /// Human-readable cause, filled only when status != kSuccess. See §6.7.
  std::string infeasibility_reason;
};

/// Result of one solve. `u0` is the only value the controller should apply.
struct MpcCbfSolution
{
  SolverStatus status{SolverStatus::kNotInitialized};
  Eigen::VectorXd u0;        ///< size nu, first optimal input.
  Eigen::MatrixXd x_pred;    ///< nx x (N+1) predicted states, column k = x_{k|t}.
  Eigen::MatrixXd u_pred;    ///< nu x N predicted inputs.
  SolverDiagnostics diagnostics;

  /// True iff the solution may be applied to the plant (kSuccess, or
  /// kMaxIterations with a finite iterate).
  bool usable() const;
};

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Thin, allocation-free-on-the-hot-path wrapper around one acados OCP solver
/// instance configured as an MPC-CBF problem.
///
/// Threading: not thread-safe. One instance per control thread.
/// Allocation: all memory is acquired in initialize(); solve() must not
/// allocate (see the assertion in test_mpc_cbf_feasibility.cpp).
class MpcCbfSolver
{
public:
  /// Hard cap on obstacles baked into the generated solver; obstacle
  /// parameters beyond this count are pruned by distance in solve().
  static constexpr int kMaxObstacles = 8;

  MpcCbfSolver(const MpcConfig & mpc_config, const CbfConfig & cbf_config);
  ~MpcCbfSolver();

  MpcCbfSolver(const MpcCbfSolver &) = delete;
  MpcCbfSolver & operator=(const MpcCbfSolver &) = delete;
  MpcCbfSolver(MpcCbfSolver &&) noexcept;
  MpcCbfSolver & operator=(MpcCbfSolver &&) noexcept;

  /// Allocates the acados solver, validates the configs, and pre-sizes every
  /// buffer. Returns false (and logs to stderr) on an invalid configuration:
  /// gamma outside (0,1], horizon < 1, weight vector of the wrong length, or
  /// omega_max * gamma > 1 under kRelaxedDecay.
  bool initialize();

  /// True once initialize() has succeeded.
  bool isInitialized() const;

  /// Solve the MPC-CBF problem for the current state.
  /// @param x0        size nx measured/estimated state.
  /// @param obstacles world-frame obstacles; may exceed kMaxObstacles.
  /// @return solution whose `status` must be checked before use.
  /// @pre isInitialized(); x0.size() == stateDim().
  MpcCbfSolution solve(const Eigen::VectorXd & x0, const std::vector<ObstacleState> & obstacles);

  /// Constant set-point tracked by every stage of the horizon. Overrides any
  /// previously set trajectory reference.
  void setReference(const Eigen::VectorXd & x_ref);

  /// Time-varying reference; nx x (N+1). Column k is the reference for stage k.
  /// Falls back to the last column if fewer than N+1 columns are supplied.
  void setReferenceTrajectory(const Eigen::MatrixXd & x_ref_traj);

  /// Update the class-K gain between solves without re-generating code.
  /// No-op and returns false if gamma is outside (0, 1].
  bool setGamma(double gamma);

  /// Seed the next solve. Shapes must match x_pred/u_pred; ignored otherwise.
  void warmStart(const Eigen::MatrixXd & x_guess, const Eigen::MatrixXd & u_guess);

  /// Drop the warm start and any internal iterate, keeping the configuration.
  void reset();

  int stateDim() const;
  int inputDim() const;
  const MpcConfig & mpcConfig() const;
  const CbfConfig & cbfConfig() const;

  /// h_j(x) for obstacle j evaluated on a single state, using the same
  /// definition the generated solver enforces (§6.6). Exposed for tests,
  /// notebooks (via the C API) and the node's diagnostics publisher.
  /// Positive means safe.
  static double barrierValue(
    ModelType model, const Eigen::VectorXd & x, const ObstacleState & obstacle,
    double inflation_radius);

  /// Number of states/inputs implied by a model, for callers that need the
  /// dimensions before constructing a solver.
  static int stateDimOf(ModelType model);
  static int inputDimOf(ModelType model);

private:
  struct Impl;                    ///< Holds the acados handles; defined in the .cpp.
  std::unique_ptr<Impl> impl_;
};

/// Name of a status, for logs and diagnostic messages ("SUCCESS", "INFEASIBLE", ...).
const char * toString(SolverStatus status);
const char * toString(CbfVariant variant);
const char * toString(ModelType model);

/// Parse a model/variant name coming from a ROS parameter or YAML file.
/// Returns false and leaves the output untouched when the name is unknown.
bool parseModelType(const std::string & name, ModelType & out);
bool parseCbfVariant(const std::string & name, CbfVariant & out);

}  // namespace mpc_cbf_unified

#endif  // MPC_CBF_UNIFIED__MPC_CBF_SOLVER_HPP_
