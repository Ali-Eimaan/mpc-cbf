// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Convex set utilities for the tube-MPC-CBF pipeline: H-representation
// polytopes, zonotopes, Minkowski sums, support functions, and an outer
// approximation of the minimal robust positively invariant (mRPI) set.
//
// Reference for the mRPI approximation:
//   S. V. Rakovic, E. C. Kerrigan, K. I. Kouramas, D. Q. Mayne, "Invariant
//   approximations of the minimal robust positively invariant set",
//   IEEE TAC 50(3), 2005.

#ifndef MPC_CBF_UNIFIED__DISTURBANCE_SETS_HPP_
#define MPC_CBF_UNIFIED__DISTURBANCE_SETS_HPP_

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mpc_cbf_unified
{

// ---------------------------------------------------------------------------
// Polytope: { x in R^n : A x <= b }
// ---------------------------------------------------------------------------

/// Bounded convex polytope in half-space (H) representation.
/// Invariant: A.rows() == b.size(); rows of A are normalised to unit 2-norm by
/// every operation that constructs a Polytope.
class Polytope
{
public:
  Polytope() = default;
  Polytope(const Eigen::MatrixXd & A, const Eigen::VectorXd & b);

  /// Axis-aligned box [-half_widths, +half_widths] centred at the origin.
  static Polytope box(const Eigen::VectorXd & half_widths);
  /// Axis-aligned box with explicit bounds; lower <= upper element-wise.
  static Polytope box(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper);
  /// Convex hull of the given points (columns of `points`). n <= 3 only —
  /// higher dimensions must go through Zonotope. Throws std::invalid_argument
  /// above 3-D so a silent wrong answer is impossible.
  static Polytope fromVertices(const Eigen::MatrixXd & points);

  int dimension() const;
  int numHalfspaces() const;
  const Eigen::MatrixXd & A() const;
  const Eigen::VectorXd & b() const;
  bool isEmpty() const;

  /// Support function h_P(d) = max{ d^T x : x in P }. Solved as a small LP.
  /// Returns +inf if the set is unbounded in direction d.
  double support(const Eigen::VectorXd & direction) const;

  /// True iff A x <= b + tol element-wise.
  bool contains(const Eigen::VectorXd & x, double tol = 1e-9) const;

  /// Minkowski sum via support functions in the directions of both operands'
  /// half-spaces. Outer approximation in general; exact when the normal cones
  /// align (e.g. box (+) box).
  Polytope minkowskiSum(const Polytope & other) const;

  /// Pontryagin (erosion) difference P (-) Q: { x : x + q in P for all q in Q }.
  /// Exact for H-representation: b_i <- b_i - h_Q(a_i).
  Polytope pontryaginDifference(const Polytope & other) const;

  /// Image under a linear map M P = { M x : x in P }. Requires M invertible for
  /// the exact H-representation update; otherwise routes through vertices.
  Polytope linearMap(const Eigen::MatrixXd & M) const;

  /// Intersection: stack the half-spaces, then remove redundant rows.
  Polytope intersect(const Polytope & other) const;

  /// Drop half-spaces that do not touch the boundary (LP-based redundancy
  /// removal). Returns the number of rows removed.
  int removeRedundantHalfspaces(double tol = 1e-9);

  /// Vertex enumeration; n <= 3 only, used for plotting and tests.
  Eigen::MatrixXd vertices() const;

  /// Largest 2-norm attained on the set, i.e. max{ ||x|| : x in P }. Used for
  /// the Lipschitz-based CBF tightening in tube_mpc_cbf_solver.
  double maxNorm() const;

  /// Element-wise interval hull, as (lower, upper).
  std::pair<Eigen::VectorXd, Eigen::VectorXd> boundingBox() const;

  /// Uniform scaling of the set about the origin.
  Polytope scaled(double factor) const;

  /// Serialise to / from the YAML shape documented in config/tube_mpc_params.yaml.
  std::string toYaml() const;
  static Polytope fromYaml(const std::string & yaml);

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

// ---------------------------------------------------------------------------
// Zonotope: { c + G z : ||z||_inf <= 1 }
// ---------------------------------------------------------------------------

/// Centrally symmetric set in generator representation. Minkowski sums and
/// linear maps are exact and cheap here, which is why the RPI iteration runs
/// in this representation and converts to Polytope only at the end.
class Zonotope
{
public:
  Zonotope() = default;
  Zonotope(const Eigen::VectorXd & center, const Eigen::MatrixXd & generators);

  /// Box as a zonotope: c = 0, G = diag(half_widths).
  static Zonotope box(const Eigen::VectorXd & half_widths);

  int dimension() const;
  int numGenerators() const;
  const Eigen::VectorXd & center() const;
  const Eigen::MatrixXd & generators() const;

  /// h_Z(d) = d^T c + ||G^T d||_1. Exact and closed-form.
  double support(const Eigen::VectorXd & direction) const;

  /// Exact: centres add, generator matrices concatenate.
  Zonotope minkowskiSum(const Zonotope & other) const;
  /// Exact: M c, M G.
  Zonotope linearMap(const Eigen::MatrixXd & M) const;

  /// Box-reduction of the generator count to at most `max_generators`
  /// (Girard's method: keep the longest generators, over-approximate the rest
  /// by an interval hull). Outer approximation — never shrinks the set.
  Zonotope reduceOrder(int max_generators) const;

  /// Convert to H-representation. Exact for n <= 3; for higher n it emits the
  /// half-spaces spanned by generator subsets, which is exact but exponential —
  /// call reduceOrder() first.
  Polytope toPolytope() const;

  bool contains(const Eigen::VectorXd & x, double tol = 1e-9) const;
  double maxNorm() const;

private:
  Eigen::VectorXd center_;
  Eigen::MatrixXd generators_;
};

// ---------------------------------------------------------------------------
// Invariant set computation
// ---------------------------------------------------------------------------

/// Outcome of the mRPI approximation.
struct RpiResult
{
  Polytope set;             ///< Outer approximation of F_inf.
  Zonotope zonotope_set;    ///< Same set before conversion (cheaper for sums).
  int iterations{0};        ///< s in Rakovic's algorithm.
  double alpha{0.0};        ///< The alpha satisfying A^s W subseteq alpha W.
  bool converged{false};    ///< False if max_iterations was hit first.
  bool exact{true};         ///< True if F_s was never order-reduced, i.e. the
                            ///< geometric-series identity certifying RPI of
                            ///< Omega applies exactly (see computeRpiSet).
};

/// Rakovic et al. outer approximation of the minimal RPI set for
///     e_{k+1} = A_cl e_k + w_k,   w_k in W,
/// i.e. a set Omega with A_cl Omega (+) W subseteq Omega.
/// @param A_cl          closed-loop matrix A + B K; must be Schur stable.
/// @param W             disturbance set, must contain the origin.
/// @param epsilon       target Hausdorff accuracy of the approximation.
/// @param max_iterations cap on s; returns converged = false if reached.
RpiResult computeRpiSet(
  const Eigen::MatrixXd & A_cl, const Zonotope & W, double epsilon = 1e-3,
  int max_iterations = 100);

/// Verify Omega is RPI: checks A_cl Omega (+) W subseteq Omega by comparing
/// support functions in every half-space direction of Omega.
bool isRobustPositivelyInvariant(
  const Eigen::MatrixXd & A_cl, const Polytope & Omega, const Polytope & W, double tol = 1e-6);

/// LQR gain for (A, B, Q, R) via the discrete-time algebraic Riccati equation,
/// solved by iterating the Riccati recursion to a fixed point. Returns the
/// gain K such that u = K x is stabilising (note the sign: A + B K is Schur).
Eigen::MatrixXd discreteLqrGain(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, int max_iterations = 1000, double tol = 1e-10);

/// Estimate a Lipschitz constant of h over the tube: sup{ ||dh/dx(x)|| : x in
/// z (+) Omega }, sampled on the vertices of Omega. Used for the tightened CBF
/// condition when h is nonlinear.
double lipschitzBoundOnTube(
  const Eigen::VectorXd & z, const Polytope & Omega,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & barrier_gradient);

}  // namespace mpc_cbf_unified

#endif  // MPC_CBF_UNIFIED__DISTURBANCE_SETS_HPP_
