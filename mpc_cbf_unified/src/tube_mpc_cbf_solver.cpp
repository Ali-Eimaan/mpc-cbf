// Copyright (c) 2026 Ali-Eimaan. MIT License.
//
// SKELETON — no implementation. Implement after Milestone 1 is green; the
// algorithm is specified in IMPLEMENTATION_GUIDE.md §4 and §5.
//
// This translation unit also hosts the Polytope/Zonotope/RPI implementations
// declared in disturbance_sets.hpp, so that the set machinery and its only
// consumer stay in one compilation unit for the first version. Split it out
// once it exceeds ~600 lines.

#include "mpc_cbf_unified/tube_mpc_cbf_solver.hpp"

#include <stdexcept>

#include "mpc_cbf_unified/disturbance_sets.hpp"

namespace mpc_cbf_unified
{

// ===========================================================================
// Polytope
// ===========================================================================

Polytope::Polytope(const Eigen::MatrixXd & A, const Eigen::VectorXd & b)
{
  // TODO(deepseek): validate A.rows() == b.size(); normalise each row of [A|b]
  // by ||a_i||_2 so that support/containment tolerances have a geometric
  // meaning; drop zero rows with b_i >= 0 and mark empty when b_i < 0.
  (void)A;
  (void)b;
  throw std::logic_error("Polytope::Polytope not implemented");
}

Polytope Polytope::box(const Eigen::VectorXd & half_widths)
{
  // TODO(deepseek): A = [I; -I], b = [h; h].
  (void)half_widths;
  throw std::logic_error("Polytope::box(half_widths) not implemented");
}

Polytope Polytope::box(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
{
  // TODO(deepseek): A = [I; -I], b = [upper; -lower]; reject lower > upper.
  (void)lower;
  (void)upper;
  throw std::logic_error("Polytope::box(lower, upper) not implemented");
}

Polytope Polytope::fromVertices(const Eigen::MatrixXd & points)
{
  // TODO(deepseek): 2-D convex hull (monotone chain) -> edge normals; 3-D via
  // incremental hull. Throw std::invalid_argument for dimension > 3.
  (void)points;
  throw std::logic_error("Polytope::fromVertices not implemented");
}

int Polytope::dimension() const
{
  throw std::logic_error("Polytope::dimension not implemented");
}

int Polytope::numHalfspaces() const
{
  throw std::logic_error("Polytope::numHalfspaces not implemented");
}

const Eigen::MatrixXd & Polytope::A() const
{
  throw std::logic_error("Polytope::A not implemented");
}

const Eigen::VectorXd & Polytope::b() const
{
  throw std::logic_error("Polytope::b not implemented");
}

bool Polytope::isEmpty() const
{
  // TODO(deepseek): Chebyshev-centre LP; empty iff the max inscribed radius is
  // negative. Cache the result.
  throw std::logic_error("Polytope::isEmpty not implemented");
}

double Polytope::support(const Eigen::VectorXd & direction) const
{
  // TODO(deepseek): max d^T x s.t. A x <= b, solved with the dense simplex in
  // the anonymous namespace of this file (write it once, ~120 lines, no
  // external LP dependency). Return +inf when unbounded.
  (void)direction;
  throw std::logic_error("Polytope::support not implemented");
}

bool Polytope::contains(const Eigen::VectorXd & x, double tol) const
{
  (void)x;
  (void)tol;
  throw std::logic_error("Polytope::contains not implemented");
}

Polytope Polytope::minkowskiSum(const Polytope & other) const
{
  // TODO(deepseek): for the union of both normal directions a_i, set
  // b_i = h_this(a_i) + h_other(a_i); then removeRedundantHalfspaces().
  (void)other;
  throw std::logic_error("Polytope::minkowskiSum not implemented");
}

Polytope Polytope::pontryaginDifference(const Polytope & other) const
{
  // TODO(deepseek): same A, b_i <- b_i - h_other(a_i). Exact. This is the
  // constraint-tightening workhorse: X (-) Omega and U (-) K Omega.
  (void)other;
  throw std::logic_error("Polytope::pontryaginDifference not implemented");
}

Polytope Polytope::linearMap(const Eigen::MatrixXd & M) const
{
  // TODO(deepseek): invertible M -> (A M^{-1}, b). Otherwise map the vertices
  // and rebuild via fromVertices (dimension <= 3 only).
  (void)M;
  throw std::logic_error("Polytope::linearMap not implemented");
}

Polytope Polytope::intersect(const Polytope & other) const
{
  (void)other;
  throw std::logic_error("Polytope::intersect not implemented");
}

int Polytope::removeRedundantHalfspaces(double tol)
{
  // TODO(deepseek): row i is redundant iff max{a_i^T x : A_{-i} x <= b_{-i}}
  // <= b_i + tol. One LP per row; fine at these sizes.
  (void)tol;
  throw std::logic_error("Polytope::removeRedundantHalfspaces not implemented");
}

Eigen::MatrixXd Polytope::vertices() const
{
  // TODO(deepseek): enumerate intersections of n half-space subsets, keep the
  // feasible ones, deduplicate, order counter-clockwise in 2-D (plotting
  // depends on the ordering).
  throw std::logic_error("Polytope::vertices not implemented");
}

double Polytope::maxNorm() const
{
  // TODO(deepseek): max over vertices of ||v||_2 in low dimension; otherwise
  // maximise the support function over a direction grid (documented as an
  // under-approximation in that branch — say so in the log).
  throw std::logic_error("Polytope::maxNorm not implemented");
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Polytope::boundingBox() const
{
  // TODO(deepseek): support(+e_i) and -support(-e_i) for each axis.
  throw std::logic_error("Polytope::boundingBox not implemented");
}

Polytope Polytope::scaled(double factor) const
{
  // TODO(deepseek): (A, factor * b) for factor >= 0.
  (void)factor;
  throw std::logic_error("Polytope::scaled not implemented");
}

std::string Polytope::toYaml() const
{
  throw std::logic_error("Polytope::toYaml not implemented");
}

Polytope Polytope::fromYaml(const std::string & yaml)
{
  (void)yaml;
  throw std::logic_error("Polytope::fromYaml not implemented");
}

// ===========================================================================
// Zonotope
// ===========================================================================

Zonotope::Zonotope(const Eigen::VectorXd & center, const Eigen::MatrixXd & generators)
{
  // TODO(deepseek): validate generators.rows() == center.size().
  (void)center;
  (void)generators;
  throw std::logic_error("Zonotope::Zonotope not implemented");
}

Zonotope Zonotope::box(const Eigen::VectorXd & half_widths)
{
  (void)half_widths;
  throw std::logic_error("Zonotope::box not implemented");
}

int Zonotope::dimension() const
{
  throw std::logic_error("Zonotope::dimension not implemented");
}

int Zonotope::numGenerators() const
{
  throw std::logic_error("Zonotope::numGenerators not implemented");
}

const Eigen::VectorXd & Zonotope::center() const
{
  throw std::logic_error("Zonotope::center not implemented");
}

const Eigen::MatrixXd & Zonotope::generators() const
{
  throw std::logic_error("Zonotope::generators not implemented");
}

double Zonotope::support(const Eigen::VectorXd & direction) const
{
  // TODO(deepseek): d^T c + ||G^T d||_1. Closed form, no LP.
  (void)direction;
  throw std::logic_error("Zonotope::support not implemented");
}

Zonotope Zonotope::minkowskiSum(const Zonotope & other) const
{
  (void)other;
  throw std::logic_error("Zonotope::minkowskiSum not implemented");
}

Zonotope Zonotope::linearMap(const Eigen::MatrixXd & M) const
{
  (void)M;
  throw std::logic_error("Zonotope::linearMap not implemented");
}

Zonotope Zonotope::reduceOrder(int max_generators) const
{
  // TODO(deepseek): Girard's method — sort generators by ||g||_1 - ||g||_inf,
  // keep the largest (max_generators - n), replace the rest by the diagonal
  // box of their absolute row sums. Must be an over-approximation; assert it
  // in the unit test by sampling.
  (void)max_generators;
  throw std::logic_error("Zonotope::reduceOrder not implemented");
}

Polytope Zonotope::toPolytope() const
{
  (void)0;
  throw std::logic_error("Zonotope::toPolytope not implemented");
}

bool Zonotope::contains(const Eigen::VectorXd & x, double tol) const
{
  // TODO(deepseek): feasibility LP min 0 s.t. G z = x - c, ||z||_inf <= 1.
  (void)x;
  (void)tol;
  throw std::logic_error("Zonotope::contains not implemented");
}

double Zonotope::maxNorm() const
{
  throw std::logic_error("Zonotope::maxNorm not implemented");
}

// ===========================================================================
// RPI computation
// ===========================================================================

RpiResult computeRpiSet(
  const Eigen::MatrixXd & A_cl, const Zonotope & W, double epsilon, int max_iterations)
{
  // TODO(deepseek): Rakovic et al. (2005), Algorithm 1.
  //  1. Reject a non-Schur A_cl (spectral radius >= 1) -> converged = false.
  //  2. For s = 1, 2, ...: find the smallest alpha in [0,1) with
  //     h_W(A_cl^{s T} a_i) <= alpha * h_W(a_i) for every direction a_i, i.e.
  //     alpha_s = max_i h_W(A_cl^{s T} a_i) / h_W(a_i).
  //  3. Compute M(s) = max over axes of the sum of supports of A_cl^j W,
  //     j = 0..s-1, and stop once alpha/(1 - alpha) * M(s) <= epsilon.
  //  4. F_s = (+)_{j=0}^{s-1} A_cl^j W as a zonotope (exact sums, reduce order
  //     to rpi_max_generators every few steps), then Omega = F_s / (1 - alpha).
  //  5. Fill RpiResult; also convert to Polytope for the tightening step.
  (void)A_cl;
  (void)W;
  (void)epsilon;
  (void)max_iterations;
  throw std::logic_error("computeRpiSet not implemented");
}

bool isRobustPositivelyInvariant(
  const Eigen::MatrixXd & A_cl, const Polytope & Omega, const Polytope & W, double tol)
{
  // TODO(deepseek): for every row a_i of Omega, check
  //   h_Omega(A_cl^T a_i) + h_W(a_i) <= b_i + tol.
  (void)A_cl;
  (void)Omega;
  (void)W;
  (void)tol;
  throw std::logic_error("isRobustPositivelyInvariant not implemented");
}

Eigen::MatrixXd discreteLqrGain(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, int max_iterations, double tol)
{
  // TODO(deepseek): iterate P <- Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A
  // until ||P - P_prev||_inf < tol, then K = -(R + B^T P B)^{-1} B^T P A.
  // Sign convention: the returned K makes A + B K Schur (note the minus).
  (void)A;
  (void)B;
  (void)Q;
  (void)R;
  (void)max_iterations;
  (void)tol;
  throw std::logic_error("discreteLqrGain not implemented");
}

double lipschitzBoundOnTube(
  const Eigen::VectorXd & z, const Polytope & Omega,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & barrier_gradient)
{
  // TODO(deepseek): max ||grad h(z + v)||_2 over the vertices v of Omega. For
  // the quadratic barrier this is exact enough (the gradient is affine, so its
  // norm is maximised at a vertex).
  (void)z;
  (void)Omega;
  (void)barrier_gradient;
  throw std::logic_error("lipschitzBoundOnTube not implemented");
}

// ===========================================================================
// TubeMpcCbfSolver
// ===========================================================================

struct TubeMpcCbfSolver::Impl
{
  MpcConfig mpc;
  CbfConfig cbf;
  TubeConfig tube;
  bool initialized{false};

  Eigen::MatrixXd A_lin;      ///< Linearised (or exact, for the double
  Eigen::MatrixXd B_lin;      ///< integrator) discrete-time model used for K/RPI.
  Eigen::MatrixXd K;          ///< Ancillary gain actually in use.
  Polytope rpi;               ///< Omega.
  Polytope tightened_x;       ///< X (-) Omega.
  Polytope tightened_u;       ///< U (-) K Omega.
  Eigen::MatrixXd z_previous; ///< Last nominal trajectory, for the z_0 policy.
  bool has_previous{false};

  // TODO(deepseek): acados handles for the nominal solver (tube variant).
};

bool TubeMpcCbfSolution::usable() const
{
  throw std::logic_error("TubeMpcCbfSolution::usable not implemented");
}

TubeMpcCbfSolver::TubeMpcCbfSolver(
  const MpcConfig & mpc_config, const CbfConfig & cbf_config, const TubeConfig & tube_config)
: impl_(std::make_unique<Impl>())
{
  (void)mpc_config;
  (void)cbf_config;
  (void)tube_config;
  throw std::logic_error("TubeMpcCbfSolver::TubeMpcCbfSolver not implemented");
}

TubeMpcCbfSolver::~TubeMpcCbfSolver() = default;
TubeMpcCbfSolver::TubeMpcCbfSolver(TubeMpcCbfSolver &&) noexcept = default;
TubeMpcCbfSolver & TubeMpcCbfSolver::operator=(TubeMpcCbfSolver &&) noexcept = default;

bool TubeMpcCbfSolver::initialize()
{
  // TODO(deepseek), in this order (§4.3):
  //  1. Validate the MPC/CBF configs exactly as MpcCbfSolver::initialize does.
  //  2. Build (A_lin, B_lin): exact for kDoubleIntegrator2D; for the nonlinear
  //     models linearise about the current reference (document the assumption
  //     in the log — the RPI set is only valid near that point).
  //  3. K: discreteLqrGain(A, B, diag(lqr_Q), diag(lqr_R)) when
  //     compute_gain_from_lqr, else tube.K. Reject if A + B K is not Schur.
  //  4. Omega = computeRpiSet(A + B K, W, rpi_epsilon, rpi_max_iterations).
  //     Reject when !converged.
  //  5. tightened_x = X (-) Omega; tightened_u = U (-) (K Omega). Reject when
  //     either is empty — that means the disturbance is too large for the
  //     actuator bounds, and it must fail loudly at startup, never at runtime.
  //  6. verifyInvariance(); log alpha, the iteration count and Omega's
  //     bounding box at INFO — the RPI numbers go straight into the
  //     tube_robustness plot caption.
  //  7. Allocate the acados nominal solver with the tightened bounds.
  throw std::logic_error("TubeMpcCbfSolver::initialize not implemented");
}

bool TubeMpcCbfSolver::isInitialized() const
{
  throw std::logic_error("TubeMpcCbfSolver::isInitialized not implemented");
}

TubeMpcCbfSolution TubeMpcCbfSolver::solve(
  const Eigen::VectorXd & x0, const std::vector<ObstacleState> & obstacles)
{
  // TODO(deepseek) (§4.5):
  //  1. Choose z_0. Default policy: z_0 = x0 on the first solve and whenever
  //     x0 - z_prev(1) leaves Omega; otherwise z_0 = z_prev(1) (the shifted
  //     nominal state). This is what makes the tube recursively feasible.
  //  2. Compute the per-stage tightening for every obstacle via
  //     tighteningFor() and push it as a solver parameter, so the generated
  //     constraint reads h(z_{k+1}) - h(z_k) >= -gamma * (h(z_k) - c_k).
  //  3. Solve the nominal problem, read z_pred/v_pred.
  //  4. u_applied = v_0 + K (x0 - z_0), clipped to [u_min, u_max]; record the
  //     clip in diagnostics (a saturating ancillary law voids the guarantee).
  //  5. Fill robust_cbf_values and tightening for the plots.
  (void)x0;
  (void)obstacles;
  throw std::logic_error("TubeMpcCbfSolver::solve not implemented");
}

void TubeMpcCbfSolver::setReference(const Eigen::VectorXd & x_ref)
{
  (void)x_ref;
  throw std::logic_error("TubeMpcCbfSolver::setReference not implemented");
}

void TubeMpcCbfSolver::setReferenceTrajectory(const Eigen::MatrixXd & x_ref_traj)
{
  (void)x_ref_traj;
  throw std::logic_error("TubeMpcCbfSolver::setReferenceTrajectory not implemented");
}

void TubeMpcCbfSolver::reset()
{
  throw std::logic_error("TubeMpcCbfSolver::reset not implemented");
}

const Polytope & TubeMpcCbfSolver::rpiSet() const
{
  throw std::logic_error("TubeMpcCbfSolver::rpiSet not implemented");
}

const Eigen::MatrixXd & TubeMpcCbfSolver::ancillaryGain() const
{
  throw std::logic_error("TubeMpcCbfSolver::ancillaryGain not implemented");
}

const Polytope & TubeMpcCbfSolver::tightenedStateSet() const
{
  throw std::logic_error("TubeMpcCbfSolver::tightenedStateSet not implemented");
}

const Polytope & TubeMpcCbfSolver::tightenedInputSet() const
{
  throw std::logic_error("TubeMpcCbfSolver::tightenedInputSet not implemented");
}

double TubeMpcCbfSolver::tighteningFor(
  const Eigen::VectorXd & z, const ObstacleState & obstacle) const
{
  // TODO(deepseek) (§4.4):
  //  kSupportFunction: for h(x) = ||P x - p_obs||^2 - r^2 the exact margin is
  //    sup_{e in Omega} [ -2 (P z - p_obs)^T P e - ||P e||^2 ]. The quadratic
  //    term is non-positive, so dropping it *over*-estimates the margin and is
  //    therefore sound:  c = h_Omega(-grad h(z)) with grad h(z) = 2 P^T (P z - p_obs).
  //    Implement the support-only form; do not "improve" it by adding the
  //    quadratic back with a plus sign — that direction is unsound.
  //  kLipschitz:      c = lipschitz_h * Omega.maxNorm().
  //  kNone:           c = 0.
  (void)z;
  (void)obstacle;
  throw std::logic_error("TubeMpcCbfSolver::tighteningFor not implemented");
}

bool TubeMpcCbfSolver::verifyInvariance(double tol) const
{
  (void)tol;
  throw std::logic_error("TubeMpcCbfSolver::verifyInvariance not implemented");
}

const char * toString(TightenMode mode)
{
  (void)mode;
  throw std::logic_error("toString(TightenMode) not implemented");
}

bool parseTightenMode(const std::string & name, TightenMode & out)
{
  // TODO(deepseek): "support_function", "lipschitz", "none".
  (void)name;
  (void)out;
  throw std::logic_error("parseTightenMode not implemented");
}

}  // namespace mpc_cbf_unified
