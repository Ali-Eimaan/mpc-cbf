// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Implementation of the set machinery in disturbance_sets.hpp (Polytope,
// Zonotope, RPI approximation) and of the robust tube-MPC-CBF
// solver declared in tube_mpc_cbf_solver.hpp.
//
// The set machinery and its only consumer stay in one translation unit for
// the first version; split the set part out once it exceeds ~600 lines.

#include "mpc_cbf_unified/tube_mpc_cbf_solver.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mpc_cbf_unified/disturbance_sets.hpp"

#if MPC_CBF_WITH_ACADOS
// Generated per configuration (codegen/generate_tube_solver.py --all).
// Same SYSTEM-include arrangement as mpc_cbf_solver.cpp.
#include "acados_c/ocp_nlp_interface.h"
#include "acados/utils/types.h"
#include "acados_solver_tube_mpc_cbf_double_integrator_2d_N8_lipschitz.h"
#include "acados_solver_tube_mpc_cbf_double_integrator_2d_N8_none.h"
#include "acados_solver_tube_mpc_cbf_double_integrator_2d_N8_support_function.h"
#endif

namespace mpc_cbf_unified
{

namespace
{
constexpr double kLpFeasTol = 1e-11;          // phase-1 feasibility threshold
constexpr double kLpPivotTol = 1e-12;         // ratio-test denominator threshold
constexpr double kLpReducedCostTol = 1e-11;   // entering-column threshold
constexpr double kSupportUnbounded = std::numeric_limits<double>::infinity();
constexpr int kMaxPolytopeRows = 4096;        // toPolytope() facet-direction cap
constexpr int kRpiGeneratorCap = 256;         // computeRpiSet() zonotope cap
constexpr double kChebyshevBound = 1e9;       // artificial bound for isEmpty()
constexpr double kZeroTol = 1e-14;            // treat-as-zero threshold

// ---------------------------------------------------------------------------
// Small dense LP (no external dependency).
// ---------------------------------------------------------------------------

double spectralRadius(const Eigen::MatrixXd & M)
{
  if (M.rows() == 0) {return 0.0;}
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  double rho = 0.0;
  for (int i = 0; i < es.eigenvalues().size(); ++i) {
    rho = std::max(rho, std::abs(es.eigenvalues()(i)));
  }
  return rho;
}

struct LpResult
{
  enum class Status { kOptimal, kInfeasible, kUnbounded };
  Status status{Status::kInfeasible};
  double value{0.0};
  Eigen::VectorXd x;
};

/// Minimise c^T x subject to A x <= b, x free. Dense two-phase simplex with
/// Bland's rule for anti-cycling; x is split into x+ - x- so the standard
/// non-negative form applies. The LP sizes in this file (a few hundred rows,
/// a handful of columns) make the doubling irrelevant.
///
/// Fast path: when b >= 0 the origin is feasible, so the initial basis is the
/// slack basis and phase 1 (artificials) is skipped. This matters because the
/// tube certificate evaluates support functions on a several-hundred-row
/// polytope many times.
LpResult solveLp(
  const Eigen::MatrixXd & A, const Eigen::VectorXd & b, const Eigen::VectorXd & c)
{
  int m = static_cast<int>(A.rows());
  const int n = static_cast<int>(A.cols());
  if (b.size() != m) {
    throw std::invalid_argument("solveLp: A.rows() != b.size()");
  }
  if (c.size() != n) {
    throw std::invalid_argument("solveLp: c.size() != A.cols()");
  }
  LpResult out;
  if (m == 0) {
    // Unconstrained: unbounded below unless the objective is identically zero.
    if (c.cwiseAbs().maxCoeff() < kLpReducedCostTol) {
      out.status = LpResult::Status::kOptimal;
      out.value = 0.0;
      out.x = Eigen::VectorXd::Zero(n);
    } else {
      out.status = LpResult::Status::kUnbounded;
    }
    return out;
  }

  // Column layout: [x+ (n) | x- (n) | slack (m) | artificial (m)].
  const int n_cols = 2 * n + 2 * m;
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(m, n_cols);
  T.leftCols(n) = A;
  T.middleCols(n, n) = -A;
  T.middleCols(2 * n, m) = Eigen::MatrixXd::Identity(m, m);
  T.rightCols(m) = Eigen::MatrixXd::Identity(m, m);
  Eigen::VectorXd rhs = b;
  Eigen::VectorXi basis(m);
  const bool origin_feasible = b.minCoeff() >= 0.0;
  if (origin_feasible) {
    // Initial basis: slacks (x = 0).
    for (int i = 0; i < m; ++i) {
      basis(i) = 2 * n + i;
    }
  } else {
    // Initial basis: artificials; flip rows with negative RHS first.
    for (int i = 0; i < m; ++i) {
      if (rhs(i) < 0.0) {
        T.row(i) = -T.row(i);
        rhs(i) = -rhs(i);
      }
      basis(i) = 2 * n + m + i;
    }
  }

  // Simplex driver. Returns kOptimal when no improving pivot exists, kUnbounded
  // when an improving ray is found (phase 2 only — in phase 1 that situation is
  // unreachable because the artificial sum is bounded below by zero).
  const int iteration_cap = 400 * (m + n_cols) + 1000;
  enum class SimplexOutcome { kOptimal, kUnbounded, kCapHit };
  auto runSimplex = [&](const Eigen::VectorXd & cost, bool allow_artificial) -> SimplexOutcome {
      for (int iter = 0; iter < iteration_cap; ++iter) {
      // Basic-cost vector (size m, indexed by TABLEAU ROW — not by column) for
      // the reduced costs c - T' c_B.
        Eigen::VectorXd cb = Eigen::VectorXd::Zero(m);
        for (int i = 0; i < m; ++i) {
          cb(i) = cost(basis(i));
        }
        const Eigen::VectorXd reduced = cost - T.transpose() * cb;
      // Entering column: smallest index with a negative reduced cost (Bland).
        int enter = -1;
        for (int j = 0; j < n_cols; ++j) {
          if (!allow_artificial && j >= 2 * n + m) {continue;}
          if (reduced(j) < -kLpReducedCostTol) {
            enter = j;
            break;
          }
        }
        if (enter < 0) {return SimplexOutcome::kOptimal;}
      // Ratio test; ties broken by row index (Bland).
        int leave = -1;
        double best_ratio = std::numeric_limits<double>::infinity();
        for (int i = 0; i < m; ++i) {
          if (T(i, enter) <= kLpPivotTol) {continue;}
          const double ratio = rhs(i) / T(i, enter);
          if (ratio < best_ratio - 1e-15) {
            best_ratio = ratio;
            leave = i;
          }
        }
        if (leave < 0) {return SimplexOutcome::kUnbounded;}
        const double pivot = T(leave, enter);
        T.row(leave) /= pivot;
        rhs(leave) /= pivot;
        for (int i = 0; i < m; ++i) {
          if (i == leave) {continue;}
          const double f = T(i, enter);
          if (std::abs(f) < kLpPivotTol) {continue;}
          T.row(i) -= f * T.row(leave);
          rhs(i) -= f * rhs(leave);
        }
        basis(leave) = enter;
      }
      fprintf(
      stderr,
      "[tube_mpc_cbf_solver] solveLp: iteration cap (%d) hit — this is a bug, "
      "Bland's rule guarantees termination\n",
      iteration_cap);
      return SimplexOutcome::kCapHit;
    };

  // Phase 1: drive the artificials to zero (only when the slack basis is not
  // feasible, i.e. b has negative entries).
  double phase1_value = 0.0;
  if (!origin_feasible) {
    Eigen::VectorXd phase1_cost = Eigen::VectorXd::Zero(n_cols);
    phase1_cost.tail(m) = Eigen::VectorXd::Ones(m);
    const SimplexOutcome outcome = runSimplex(phase1_cost, true);
    (void)outcome;  // kUnbounded is unreachable here (objective bounded below).
    for (int i = 0; i < m; ++i) {
      if (basis(i) >= 2 * n + m) {phase1_value += rhs(i);}
    }
    if (phase1_value > kLpFeasTol) {
      out.status = LpResult::Status::kInfeasible;
      return out;
    }
    // Remove basic artificials (at ~0) by pivoting them out; rows that have no
    // other usable column are redundant and are dropped.
    std::vector<int> keep_rows;
    keep_rows.reserve(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) {
      if (basis(i) < 2 * n + m) {
        keep_rows.push_back(i);
        continue;
      }
      int j_pivot = -1;
      for (int j = 0; j < 2 * n + m && j_pivot < 0; ++j) {
        bool in_basis = false;
        for (int k = 0; k < m && !in_basis; ++k) {
          in_basis = (basis(k) == j);
        }
        if (!in_basis && std::abs(T(i, j)) > kLpPivotTol) {j_pivot = j;}
      }
      if (j_pivot >= 0) {
        const double pivot = T(i, j_pivot);
        T.row(i) /= pivot;
        rhs(i) /= pivot;
        for (int k = 0; k < m; ++k) {
          if (k == i) {continue;}
          const double f = T(k, j_pivot);
          if (std::abs(f) < kLpPivotTol) {continue;}
          T.row(k) -= f * T.row(i);
          rhs(k) -= f * rhs(i);
        }
        basis(i) = j_pivot;
        keep_rows.push_back(i);
      }
      // else: redundant row — dropped.
    }
    if (static_cast<int>(keep_rows.size()) < m) {
      Eigen::MatrixXd T2(static_cast<int>(keep_rows.size()), n_cols);
      Eigen::VectorXd rhs2(static_cast<int>(keep_rows.size()));
      Eigen::VectorXi basis2(static_cast<int>(keep_rows.size()));
      for (size_t r = 0; r < keep_rows.size(); ++r) {
        const int rr = static_cast<int>(r);
        T2.row(rr) = T.row(keep_rows[r]);
        rhs2(rr) = rhs(keep_rows[r]);
        basis2(rr) = basis(keep_rows[r]);
      }
      T = T2;
      rhs = rhs2;
      basis = basis2;
      m = static_cast<int>(keep_rows.size());
    }
  }

  // Phase 2: original objective; artificial columns may not re-enter.
  Eigen::VectorXd phase2_cost = Eigen::VectorXd::Zero(n_cols);
  phase2_cost.head(n) = c;
  phase2_cost.segment(n, n) = -c;
  const SimplexOutcome outcome2 = runSimplex(phase2_cost, false);
  if (outcome2 == SimplexOutcome::kUnbounded) {
    out.status = LpResult::Status::kUnbounded;
    return out;
  }
  if (outcome2 == SimplexOutcome::kCapHit) {
    out.status = LpResult::Status::kInfeasible;
    return out;
  }
  Eigen::VectorXd x_full = Eigen::VectorXd::Zero(n_cols);
  double value = 0.0;
  for (int i = 0; i < m; ++i) {
    x_full(basis(i)) = rhs(i);
    value += phase2_cost(basis(i)) * rhs(i);
  }
  out.status = LpResult::Status::kOptimal;
  out.value = value;
  out.x = x_full.head(n) - x_full.segment(n, n);
  return out;
}

// ---------------------------------------------------------------------------
// Small combinatorial helpers for vertex/hull enumeration.
// ---------------------------------------------------------------------------

/// Visit every k-subset of {0, ..., m-1}.
template<typename Fn>
void forEachCombination(int k, int m, Fn && visit)
{
  std::vector<int> idx(static_cast<size_t>(k));
  std::function<void(int, int)> rec = [&](int at, int start) {
      if (at == k) {
        visit(idx);
        return;
      }
      for (int j = start; j < m; ++j) {
        idx[static_cast<size_t>(at)] = j;
        rec(at + 1, j + 1);
      }
    };
  rec(0, 0);
}

/// For an (n-1) x n matrix G (rows = vectors), the vector v (up to scale)
/// orthogonal to every row: v_i = (-1)^i det(G with column i removed). This is
/// the generalised cross product; it is nonzero exactly when rank(G) = n - 1.
Eigen::VectorXd crossNormal(const Eigen::MatrixXd & G)
{
  const int n = static_cast<int>(G.cols());
  if (G.rows() != n - 1) {
    throw std::invalid_argument("crossNormal: expected an (n-1) x n matrix");
  }
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) {
    Eigen::MatrixXd minor(n - 1, n - 1);
    int col = 0;
    for (int j = 0; j < n; ++j) {
      if (j == i) {continue;}
      minor.col(col) = G.col(j);
      ++col;
    }
    v(i) = (i % 2 == 0) ? minor.determinant() : -minor.determinant();
  }
  return v;
}

/// Canonical direction key for deduplicating facet normals: the unit vector,
/// sign-flipped so the first nonzero component is positive, quantised to the
/// 1e-9 grid. Two normals closer than ~5e-10 map to the same key.
std::vector<long long> directionKey(const Eigen::VectorXd & v)
{
  Eigen::VectorXd u = v;
  const double nrm = u.norm();
  if (nrm < kZeroTol) {return {};}
  u /= nrm;
  for (int i = 0; i < u.size(); ++i) {
    if (std::abs(u(i)) > 1e-12) {
      if (u(i) < 0.0) {u = -u;}
      break;
    }
  }
  std::vector<long long> key(static_cast<size_t>(u.size()));
  for (int i = 0; i < u.size(); ++i) {
    key[static_cast<size_t>(i)] = llround(u(i) / 1e-9);
  }
  return key;
}

/// Deterministic Fibonacci-sphere directions for the maxNorm() grid fallback.
Eigen::MatrixXd fibonacciSphereDirections(int n, int count)
{
  Eigen::MatrixXd D(n, count);
  constexpr double kPi = 3.14159265358979323846;
  const double golden = kPi * (3.0 - std::sqrt(5.0));
  for (int k = 0; k < count; ++k) {
    const double z = 1.0 - 2.0 * (static_cast<double>(k) + 0.5) / static_cast<double>(count);
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double theta = golden * static_cast<double>(k);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(n);
    if (n == 2) {
      d(0) = std::cos(theta);
      d(1) = std::sin(theta);
    } else {
      d(0) = r * std::cos(theta);
      d(1) = r * std::sin(theta);
      d(2) = z;
      for (int i = 3; i < n; ++i) {
        d(i) = 0.0;
      }
      if (n > 3) {
        // Embed the 2-sphere directions in R^n deterministically.
        d.tail(n - 3).setZero();
      }
    }
    if (d.norm() > kZeroTol) {D.col(k) = d.normalized();}
  }
  return D;
}

/// Tiny YAML float-list parser for Polytope::toYaml/fromYaml.
std::vector<double> parseYamlFloatList(const std::string & text)
{
  std::vector<double> out;
  std::string token;
  for (const char ch : text) {
    if (ch == '[' || ch == ']' || ch == ',' || ch == '\n' || ch == ' ') {
      if (!token.empty()) {
        out.push_back(std::stod(token));
        token.clear();
      }
      continue;
    }
    token.push_back(ch);
  }
  if (!token.empty()) {out.push_back(std::stod(token));}
  return out;
}

std::string formatDouble(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return std::string(buf);
}

}  // namespace

// ===========================================================================
// Polytope
// ===========================================================================

// ===========================================================================
// Polytope
// ===========================================================================

Polytope::Polytope(const Eigen::MatrixXd & A, const Eigen::VectorXd & b)
{
  if (A.rows() != b.size()) {
    throw std::invalid_argument("Polytope::Polytope: A.rows() != b.size()");
  }
  if (A.cols() == 0 && A.rows() == 0) {return;}    // default-constructed
  const int n = static_cast<int>(A.cols());
  bool empty = false;
  std::vector<Eigen::VectorXd> rows;
  std::vector<double> offsets;
  for (int i = 0; i < A.rows(); ++i) {
    const double nrm = A.row(i).norm();
    if (nrm > kZeroTol) {
      rows.push_back(A.row(i).transpose() / nrm);
      offsets.push_back(b(i) / nrm);
    } else if (b(i) < 0.0) {
      // 0 <= b_i < 0: the polytope is empty. Represent it with a single
      // unsatisfiable row so isEmpty()/contains()/support() stay well defined.
      empty = true;
      break;
    }
    // else: 0 <= 0 is always satisfied — drop the row.
  }
  if (empty) {
    A_ = Eigen::MatrixXd::Zero(1, n);
    b_ = Eigen::VectorXd::Constant(1, -1.0);
    return;
  }
  A_ = Eigen::MatrixXd(static_cast<int>(rows.size()), n);
  b_ = Eigen::VectorXd(static_cast<int>(rows.size()));
  for (size_t i = 0; i < rows.size(); ++i) {
    A_.row(static_cast<int>(i)) = rows[i].transpose();
    b_(static_cast<int>(i)) = offsets[i];
  }
}

Polytope Polytope::box(const Eigen::VectorXd & half_widths)
{
  const int n = static_cast<int>(half_widths.size());
  Eigen::MatrixXd A(2 * n, n);
  Eigen::VectorXd b(2 * n);
  for (int i = 0; i < n; ++i) {
    A.row(i) = Eigen::VectorXd::Unit(n, i).transpose();
    A.row(n + i) = -Eigen::VectorXd::Unit(n, i).transpose();
    b(i) = half_widths(i);
    b(n + i) = half_widths(i);
  }
  return Polytope(A, b);
}

Polytope Polytope::box(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
{
  if (lower.size() != upper.size()) {
    throw std::invalid_argument("Polytope::box(lower, upper): size mismatch");
  }
  if ((upper - lower).minCoeff() < 0.0) {
    throw std::invalid_argument("Polytope::box(lower, upper): lower > upper");
  }
  const int n = static_cast<int>(lower.size());
  Eigen::MatrixXd A(2 * n, n);
  Eigen::VectorXd b(2 * n);
  for (int i = 0; i < n; ++i) {
    A.row(i) = Eigen::VectorXd::Unit(n, i).transpose();
    A.row(n + i) = -Eigen::VectorXd::Unit(n, i).transpose();
    b(i) = upper(i);
    b(n + i) = -lower(i);
  }
  return Polytope(A, b);
}

Polytope Polytope::fromVertices(const Eigen::MatrixXd & points)
{
  const int n = static_cast<int>(points.rows());
  const int m = static_cast<int>(points.cols());
  if (n > 3) {
    throw std::invalid_argument(
      "Polytope::fromVertices: dimension > 3 is not supported; use Zonotope");
  }
  if (m == 0) {
    return Polytope();  // empty point set -> default (uninitialised) polytope
  }
  if (n == 1) {
    const double lo = points.row(0).minCoeff();
    const double hi = points.row(0).maxCoeff();
    Eigen::VectorXd l(1), u(1);
    l(0) = lo;
    u(0) = hi;
    return Polytope::box(l, u);
  }
  if (n == 2) {
    // Andrew's monotone chain; output is counter-clockwise.
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(static_cast<size_t>(m));
    for (int i = 0; i < m; ++i) {
      pts.emplace_back(points(0, i), points(1, i));
    }
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
        return (a(0) < b(0)) || (a(0) == b(0) && a(1) < b(1));
    });
    auto cross = [](const Eigen::Vector2d & o, const Eigen::Vector2d & a,
      const Eigen::Vector2d & b) {
        return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
      };
    std::vector<Eigen::Vector2d> hull;
    for (const auto & p : pts) {
      while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0.0) {
        hull.pop_back();
      }
      hull.push_back(p);
    }
    const size_t lower_size = hull.size();
    for (int i = m - 2; i >= 0; --i) {
      const auto & p = pts[static_cast<size_t>(i)];
      while (hull.size() > lower_size &&
        cross(hull[hull.size() - 2], hull.back(), p) <= 0.0)
      {
        hull.pop_back();
      }
      hull.push_back(p);
    }
    if (hull.size() > 1) {hull.pop_back();}    // repeated first point
    if (hull.size() < 3) {
      fprintf(
        stderr,
        "[tube_mpc_cbf_solver] Polytope::fromVertices: fewer than 3 distinct "
        "2-D points — returning the empty polytope\n");
      return Polytope();
    }
    Eigen::MatrixXd A(static_cast<int>(hull.size()), 2);
    Eigen::VectorXd b(static_cast<int>(hull.size()));
    for (size_t e = 0; e < hull.size(); ++e) {
      const Eigen::Vector2d & p = hull[e];
      const Eigen::Vector2d & q = hull[(e + 1) % hull.size()];
      const Eigen::Vector2d d = q - p;
      // CCW: interior is to the left, so the outward normal is (d.y, -d.x).
      Eigen::Vector2d nrm(d(1), -d(0));
      nrm.normalize();
      A.row(static_cast<int>(e)) = nrm.transpose();
      b(static_cast<int>(e)) = nrm.dot(p);
    }
    return Polytope(A, b);
  }
  // n == 3: enumerate all triples; a triple is a face iff all points lie on one
  // side of its plane.
  std::vector<Eigen::Vector3d> normals;
  std::vector<double> offsets;
  forEachCombination(3, m, [&](const std::vector<int> & idx) {
      const Eigen::Vector3d p0 = points.col(idx[0]);
      const Eigen::Vector3d p1 = points.col(idx[1]);
      const Eigen::Vector3d p2 = points.col(idx[2]);
      Eigen::Vector3d nrm = (p1 - p0).cross(p2 - p0);
      if (nrm.norm() < kZeroTol) {return;}  // collinear triple
      nrm.normalize();
      const double off = nrm.dot(p0);
      double max_pos = 0.0;
      double max_neg = 0.0;
      for (int i = 0; i < m; ++i) {
        const double s = nrm.dot(points.col(i)) - off;
        max_pos = std::max(max_pos, s);
        max_neg = std::max(max_neg, -s);
      }
      constexpr double kFaceTol = 1e-9;
      if (max_pos <= kFaceTol) {
        normals.push_back(nrm);
        offsets.push_back(off);
      } else if (max_neg <= kFaceTol) {
        normals.push_back(-nrm);
        offsets.push_back(-off);
      }
  });
  if (normals.empty()) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] Polytope::fromVertices: no 3-D faces found — "
      "returning the empty polytope\n");
    return Polytope();
  }
  // Deduplicate near-identical faces.
  std::map<std::vector<long long>, Eigen::Vector3d> unique;
  std::map<std::vector<long long>, double> unique_b;
  for (size_t i = 0; i < normals.size(); ++i) {
    const auto key = directionKey(normals[i]);
    if (key.empty()) {continue;}
    if (unique.find(key) == unique.end()) {
      unique[key] = normals[i];
      unique_b[key] = offsets[i];
    }
  }
  Eigen::MatrixXd A(static_cast<int>(unique.size()), 3);
  Eigen::VectorXd b(static_cast<int>(unique.size()));
  int row = 0;
  for (const auto & kv : unique) {
    A.row(row) = kv.second.transpose();
    b(row) = unique_b[kv.first];
    ++row;
  }
  return Polytope(A, b);
}

int Polytope::dimension() const
{
  return static_cast<int>(A_.cols());
}

int Polytope::numHalfspaces() const
{
  return static_cast<int>(A_.rows());
}

const Eigen::MatrixXd & Polytope::A() const
{
  return A_;
}

const Eigen::VectorXd & Polytope::b() const
{
  return b_;
}

bool Polytope::isEmpty() const
{
  // Chebyshev centre LP: max r s.t. a_i^T x + r <= b_i for every row
  // (rows are unit-normalised, so r is the inscribed radius). Empty iff the
  // max radius is strictly negative.
  if (numHalfspaces() == 0) {return false;}
  const int n = dimension();
  Eigen::MatrixXd A(numHalfspaces() + 1, n + 1);
  Eigen::VectorXd b(numHalfspaces() + 1);
  for (int i = 0; i < numHalfspaces(); ++i) {
    A.row(i).head(n) = A_.row(i);
    A(i, n) = 1.0;
    b(i) = b_(i);
  }
  A.row(numHalfspaces()).setZero();
  A(numHalfspaces(), n) = 1.0;
  b(numHalfspaces()) = kChebyshevBound;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n + 1);
  c(n) = -1.0;  // minimise -r
  const LpResult r = solveLp(A, b, c);
  if (r.status != LpResult::Status::kOptimal) {return true;}    // unbounded?? — defensive
  return (-r.value) <= -1e-9;
}

double Polytope::support(const Eigen::VectorXd & direction) const
{
  if (dimension() == 0) {return 0.0;}
  // max d'x = -min -d'x.
  const LpResult r = solveLp(A_, b_, -direction);
  switch (r.status) {
      case LpResult::Status::kOptimal:
        return -r.value;
      case LpResult::Status::kInfeasible:
        return -kSupportUnbounded; // support of the empty set
      case LpResult::Status::kUnbounded:
        return kSupportUnbounded;
  }
  return 0.0;  // unreachable
}

bool Polytope::contains(const Eigen::VectorXd & x, double tol) const
{
  if (x.size() != dimension()) {return false;}
  // Rows are unit-normalised, so an element-wise check has geometric meaning.
  const Eigen::VectorXd excess = A_ * x - b_;
  for (int i = 0; i < excess.size(); ++i) {
    if (excess(i) > tol) {return false;}
  }
  return true;
}

Polytope Polytope::minkowskiSum(const Polytope & other) const
{
  if (dimension() != other.dimension()) {
    throw std::invalid_argument("Polytope::minkowskiSum: dimension mismatch");
  }
  const int n = dimension();
  const int m1 = numHalfspaces();
  const int m2 = other.numHalfspaces();
  Eigen::MatrixXd A(m1 + m2, n);
  Eigen::VectorXd b(m1 + m2);
  for (int i = 0; i < m1; ++i) {
    const Eigen::VectorXd a = A_.row(i).transpose();
    A.row(i) = a.transpose();
    b(i) = support(a) + other.support(a);
  }
  for (int j = 0; j < m2; ++j) {
    const Eigen::VectorXd a = other.A().row(j).transpose();
    A.row(m1 + j) = a.transpose();
    b(m1 + j) = support(a) + other.support(a);
  }
  Polytope result(A, b);
  result.removeRedundantHalfspaces();
  return result;
}

Polytope Polytope::pontryaginDifference(const Polytope & other) const
{
  if (dimension() != other.dimension()) {
    throw std::invalid_argument("Polytope::pontryaginDifference: dimension mismatch");
  }
  Eigen::VectorXd b = b_;
  for (int i = 0; i < numHalfspaces(); ++i) {
    b(i) -= other.support(A_.row(i).transpose());
  }
  return Polytope(A_, b);
}

Polytope Polytope::linearMap(const Eigen::MatrixXd & M) const
{
  if (M.rows() != dimension() || M.cols() != dimension()) {
    throw std::invalid_argument("Polytope::linearMap: M must be square of dimension()");
  }
  Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
  if (lu.isInvertible()) {
    return Polytope(A_ * lu.inverse(), b_);
  }
  const int n = dimension();
  if (n > 3) {
    throw std::invalid_argument(
      "Polytope::linearMap: singular M needs vertex enumeration, which is "
      "limited to dimension <= 3");
  }
  const Eigen::MatrixXd verts = vertices();
  if (verts.cols() == 0) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] Polytope::linearMap: vertex enumeration failed — "
      "returning the empty polytope\n");
    return Polytope();
  }
  return Polytope::fromVertices(M * verts);
}

Polytope Polytope::intersect(const Polytope & other) const
{
  if (dimension() != other.dimension()) {
    throw std::invalid_argument("Polytope::intersect: dimension mismatch");
  }
  Eigen::MatrixXd A(numHalfspaces() + other.numHalfspaces(), dimension());
  Eigen::VectorXd b(numHalfspaces() + other.numHalfspaces());
  A.topRows(numHalfspaces()) = A_;
  b.head(numHalfspaces()) = b_;
  A.bottomRows(other.numHalfspaces()) = other.A();
  b.tail(other.numHalfspaces()) = other.b();
  Polytope result(A, b);
  result.removeRedundantHalfspaces();
  return result;
}

int Polytope::removeRedundantHalfspaces(double tol)
{
  const int n = dimension();
  const int m = numHalfspaces();
  if (m == 0 || m == 1) {return 0;}
  // Work on an "active" row list and drop rows as they are found redundant, so
  // that a pair of identical half-spaces keeps exactly one of its members (a
  // single pass over the original list would classify both as redundant and
  // throw the constraint away entirely — unsound).
  std::vector<int> active;
  active.reserve(static_cast<size_t>(m));
  for (int i = 0; i < m; ++i) {
    active.push_back(i);
  }
  int removed = 0;
  size_t at = 0;
  while (at < active.size()) {
    const int row = active[at];
    // Row is redundant iff max{ a_i^T x : A_active\{row} x <= b } <= b_i + tol.
    Eigen::MatrixXd A_minus(static_cast<int>(active.size()) - 1, n);
    Eigen::VectorXd b_minus(static_cast<int>(active.size()) - 1);
    int r = 0;
    for (size_t k = 0; k < active.size(); ++k) {
      if (k == at) {continue;}
      A_minus.row(r) = A_.row(active[k]);
      b_minus(r) = b_(active[k]);
      ++r;
    }
    const LpResult lp = solveLp(A_minus, b_minus, -A_.row(row).transpose());
    if (lp.status == LpResult::Status::kOptimal && -lp.value <= b_(row) + tol) {
      active.erase(active.begin() + static_cast<std::ptrdiff_t>(at));
      ++removed;
      // Do not advance: the row that just replaced `at` must be re-tested.
    } else {
      ++at;  // unbounded, infeasible, or essential
    }
  }
  if (removed == 0) {return 0;}
  Eigen::MatrixXd A2(static_cast<int>(active.size()), n);
  Eigen::VectorXd b2(static_cast<int>(active.size()));
  for (size_t i = 0; i < active.size(); ++i) {
    A2.row(static_cast<int>(i)) = A_.row(active[i]);
    b2(static_cast<int>(i)) = b_(active[i]);
  }
  A_ = A2;
  b_ = b2;
  return removed;
}

Eigen::MatrixXd Polytope::vertices() const
{
  const int n = dimension();
  const int m = numHalfspaces();
  if (n == 0) {return Eigen::MatrixXd::Zero(0, 0);}
  if (n > 3) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] Polytope::vertices: vertex enumeration is limited "
      "to dimension <= 3 — returning no vertices\n");
    return Eigen::MatrixXd::Zero(0, n);
  }
  std::vector<Eigen::VectorXd> pts;
  forEachCombination(n, m, [&](const std::vector<int> & idx) {
      Eigen::MatrixXd M(n, n);
      Eigen::VectorXd rhs(n);
      for (int k = 0; k < n; ++k) {
        M.row(k) = A_.row(idx[static_cast<size_t>(k)]);
        rhs(k) = b_(idx[static_cast<size_t>(k)]);
      }
      Eigen::FullPivLU<Eigen::MatrixXd> lu(M);
      if (!lu.isInvertible()) {return;}
      const Eigen::VectorXd x = lu.solve(rhs);
      if ((A_ * x - b_).maxCoeff() <= 1e-9) {pts.push_back(x);}
  });
  if (pts.empty()) {return Eigen::MatrixXd::Zero(0, n);}
  // Deduplicate.
  std::vector<Eigen::VectorXd> unique;
  for (const auto & p : pts) {
    bool dup = false;
    for (const auto & q : unique) {
      if ((p - q).norm() <= 1e-9) {dup = true; break;}
    }
    if (!dup) {unique.push_back(p);}
  }
  // Order counter-clockwise in 2-D (plotting depends on it).
  if (n == 2 && unique.size() > 2) {
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto & p : unique) {
      centroid += p;
    }
    centroid /= static_cast<double>(unique.size());
    std::sort(unique.begin(), unique.end(),
      [&](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
        return std::atan2(a(1) - centroid(1), a(0) - centroid(0)) <
               std::atan2(b(1) - centroid(1), b(0) - centroid(0));
    });
  }
  Eigen::MatrixXd out(n, static_cast<int>(unique.size()));
  for (size_t i = 0; i < unique.size(); ++i) {
    out.col(i) = unique[i];
  }
  return out;
}

double Polytope::maxNorm() const
{
  const int n = dimension();
  if (n <= 3) {
    const Eigen::MatrixXd verts = vertices();
    double best = 0.0;
    for (int i = 0; i < verts.cols(); ++i) {
      best = std::max(best, verts.col(i).norm());
    }
    return best;
  }
  // Dimension >= 4: maximise the support function over a direction grid. This
  // is an UNDER-approximation of the true max norm — it is logged loudly
  // because kLipschitz tightening must never rely on it (the tube uses the
  // zonotope's closed-form over-approximation instead).
  fprintf(
    stderr,
    "[tube_mpc_cbf_solver] Polytope::maxNorm: dimension %d > 3 — grid search "
    "is an under-approximation; do not use it for kLipschitz tightening\n",
    n);
  double best = 0.0;
  const Eigen::MatrixXd D = fibonacciSphereDirections(n, 256);
  for (int i = 0; i < D.cols(); ++i) {
    const double s = support(D.col(i));
    if (std::isfinite(s)) {best = std::max(best, s);}
  }
  for (int i = 0; i < n; ++i) {
    const double s = support(Eigen::VectorXd::Unit(n, i));
    if (std::isfinite(s)) {best = std::max(best, s);}
  }
  return best;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Polytope::boundingBox() const
{
  const int n = dimension();
  Eigen::VectorXd lo(n), hi(n);
  for (int i = 0; i < n; ++i) {
    const Eigen::VectorXd e = Eigen::VectorXd::Unit(n, i);
    hi(i) = support(e);
    lo(i) = -support(-e);
  }
  return {lo, hi};
}

Polytope Polytope::scaled(double factor) const
{
  if (factor < 0.0) {
    throw std::invalid_argument("Polytope::scaled: factor must be >= 0");
  }
  return Polytope(A_, factor * b_);
}

std::string Polytope::toYaml() const
{
  const int n = dimension();
  std::string out = "dimension: " + std::to_string(n) + "\n";
  out += "A: [";
  for (int i = 0; i < numHalfspaces(); ++i) {
    for (int j = 0; j < n; ++j) {
      if (i > 0 || j > 0) {out += ", ";}
      out += formatDouble(A_(i, j));
    }
  }
  out += "]\nb: [";
  for (int i = 0; i < numHalfspaces(); ++i) {
    if (i > 0) {out += ", ";}
    out += formatDouble(b_(i));
  }
  out += "]\n";
  return out;
}

Polytope Polytope::fromYaml(const std::string & yaml)
{
  int n = 0;
  std::vector<double> A_flat;
  std::vector<double> b;
  // Minimal line-based parse of the shape produced by toYaml().
  std::string::size_type start = 0;
  while (start < yaml.size()) {
    std::string::size_type nl = yaml.find('\n', start);
    if (nl == std::string::npos) {nl = yaml.size();}
    const std::string line = yaml.substr(start, nl - start);
    start = nl + 1;
    std::string trimmed = line;
    while (!trimmed.empty() && (trimmed.front() == ' ' || trimmed.front() == '\t')) {
      trimmed.erase(trimmed.begin());
    }
    const std::string dimension_key = "dimension:";
    const std::string a_key = "A:";
    const std::string b_key = "b:";
    if (trimmed.rfind(dimension_key, 0) == 0) {
      n = std::stoi(trimmed.substr(dimension_key.size()));
    } else if (trimmed.rfind(a_key, 0) == 0) {
      A_flat = parseYamlFloatList(trimmed.substr(a_key.size()));
    } else if (trimmed.rfind(b_key, 0) == 0) {
      b = parseYamlFloatList(trimmed.substr(b_key.size()));
    }
  }
  if (n == 0 || A_flat.size() != static_cast<size_t>(n) * b.size()) {
    throw std::invalid_argument("Polytope::fromYaml: malformed YAML");
  }
  Eigen::MatrixXd A(static_cast<int>(b.size()), n);
  for (size_t i = 0; i < b.size(); ++i) {
    for (int j = 0; j < n; ++j) {
      A(static_cast<int>(i), j) = A_flat[i * static_cast<size_t>(n) + static_cast<size_t>(j)];
    }
  }
  Eigen::VectorXd bv(static_cast<int>(b.size()));
  for (size_t i = 0; i < b.size(); ++i) {
    bv(static_cast<int>(i)) = b[i];
  }
  return Polytope(A, bv);
}

// ===========================================================================
// Zonotope
// ===========================================================================

Zonotope::Zonotope(const Eigen::VectorXd & center, const Eigen::MatrixXd & generators)
{
  if (generators.rows() != center.size()) {
    throw std::invalid_argument("Zonotope::Zonotope: generators.rows() != center.size()");
  }
  center_ = center;
  generators_ = generators;
}

Zonotope Zonotope::box(const Eigen::VectorXd & half_widths)
{
  const int n = static_cast<int>(half_widths.size());
  return Zonotope(Eigen::VectorXd::Zero(n), half_widths.asDiagonal());
}

int Zonotope::dimension() const
{
  return static_cast<int>(center_.size());
}

int Zonotope::numGenerators() const
{
  return static_cast<int>(generators_.cols());
}

const Eigen::VectorXd & Zonotope::center() const
{
  return center_;
}

const Eigen::MatrixXd & Zonotope::generators() const
{
  return generators_;
}

double Zonotope::support(const Eigen::VectorXd & direction) const
{
  // h_Z(d) = d^T c + ||G^T d||_1. Closed form, no LP.
  return direction.dot(center_) + (generators_.transpose() * direction).cwiseAbs().sum();
}

Zonotope Zonotope::minkowskiSum(const Zonotope & other) const
{
  if (dimension() != other.dimension()) {
    throw std::invalid_argument("Zonotope::minkowskiSum: dimension mismatch");
  }
  Eigen::MatrixXd G(center_.size(), numGenerators() + other.numGenerators());
  G.leftCols(numGenerators()) = generators_;
  G.rightCols(other.numGenerators()) = other.generators();
  return Zonotope(center_ + other.center(), G);
}

Zonotope Zonotope::linearMap(const Eigen::MatrixXd & M) const
{
  if (M.cols() != dimension() || M.rows() != dimension()) {
    throw std::invalid_argument("Zonotope::linearMap: M must be square of dimension()");
  }
  return Zonotope(M * center_, M * generators_);
}

Zonotope Zonotope::reduceOrder(int max_generators) const
{
  const int n = dimension();
  const int m = numGenerators();
  if (m <= max_generators) {return *this;}
  if (max_generators < n) {
    throw std::invalid_argument(
      "Zonotope::reduceOrder: max_generators must be >= dimension");
  }
  const int keep = max_generators - n;
  // Girard's method: score ||g||_1 - ||g||_inf, keep the longest generators,
  // replace the rest by the diagonal interval hull of their absolute row sums.
  std::vector<std::pair<double, int>> scored;
  scored.reserve(static_cast<size_t>(m));
  for (int j = 0; j < m; ++j) {
    const double score = generators_.col(j).lpNorm<1>() -
      generators_.col(j).lpNorm<Eigen::Infinity>();
    scored.emplace_back(score, j);
  }
  std::sort(scored.begin(), scored.end(),
    [](const std::pair<double, int> & a, const std::pair<double, int> & b) {
      return a.first > b.first;
    });
  Eigen::VectorXd dropped_sum = Eigen::VectorXd::Zero(n);
  std::vector<int> kept;
  kept.reserve(static_cast<size_t>(keep));
  for (int r = 0; r < m; ++r) {
    const int j = scored[static_cast<size_t>(r)].second;
    if (r < keep) {
      kept.push_back(j);
    } else {
      dropped_sum += generators_.col(j).cwiseAbs();
    }
  }
  Eigen::MatrixXd G(n, keep + n);
  for (size_t r = 0; r < kept.size(); ++r) {
    G.col(static_cast<int>(r)) = generators_.col(kept[r]);
  }
  G.rightCols(n) = dropped_sum.asDiagonal();
  return Zonotope(center_, G);
}

Polytope Zonotope::toPolytope() const
{
  const int n = dimension();
  const int m = numGenerators();
  if (n == 0) {return Polytope();}
  if (m == 0) {
    // Single point: over-approximate with a tiny box (documented).
    const Eigen::VectorXd c = center();
    return Polytope::box(c.array() - 1e-9, c.array() + 1e-9);
  }
  if (n == 1) {
    double h = 0.0;
    for (int j = 0; j < m; ++j) {
      h += std::abs(generators_(0, j));
    }
    Eigen::VectorXd l(1), u(1);
    l(0) = center_(0) - h;
    u(0) = center_(0) + h;
    return Polytope::box(l, u);
  }
  // The facet count is at most 2*C(m, n-1) (each facet normal is the normal of
  // an (n-1)-generator subset). Enumerating C(m, n-1) combinations only to hit
  // the row cap below and discard them is the slow path — m is bounded by the
  // generator cap and C(256, 3) is ~2.7M combinations. Reduce first when the
  // bound already exceeds the cap, then enumerate (the reduced recursion
  // terminates with at most 2n facets).
  {
    const int k = n - 1;
    const int kk = std::min(k, m - k);
    double comb = 1.0;
    for (int i = 1; i <= kk; ++i) {
      comb *= static_cast<double>(m - kk + i) / static_cast<double>(i);
      if (comb > static_cast<double>(kMaxPolytopeRows)) {
        if (m > 16) {return reduceOrder(16).toPolytope();}
        return reduceOrder(n).toPolytope();
      }
    }
  }
  // Facet directions: the normal of every (n-1)-generator subset. Exact for
  // zonotopes: every facet normal arises this way.
  std::map<std::vector<long long>, Eigen::VectorXd> dirs;
  forEachCombination(n - 1, m, [&](const std::vector<int> & idx) {
      Eigen::MatrixXd G(n - 1, n);
      for (int k = 0; k < n - 1; ++k) {
        G.row(k) = generators_.col(idx[static_cast<size_t>(k)]).transpose();
      }
      const Eigen::VectorXd v = crossNormal(G);
      if (v.norm() < kZeroTol) {return;}  // rank < n-1 subset
      const auto key = directionKey(v);
      if (key.empty()) {return;}
      if (dirs.find(key) == dirs.end()) {dirs[key] = v.normalized();}
  });
  if (static_cast<int>(dirs.size()) > kMaxPolytopeRows) {
    // Facet count exponential in m (up to 2*C(m, n-1)): fall back to an
    // over-approximation with a bounded facet count.
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] Zonotope::toPolytope: %d facet directions from "
      "%d generators exceed the %d-row cap\n",
      static_cast<int>(dirs.size()), m, kMaxPolytopeRows);
    if (m > 16) {return reduceOrder(16).toPolytope();}
    // m <= 16 but still too many facets (dimension >= 5): the interval hull
    // has exactly 2n facets and terminates the recursion.
    return reduceOrder(n).toPolytope();
  }
  Eigen::MatrixXd A(2 * static_cast<int>(dirs.size()), n);
  Eigen::VectorXd b(2 * static_cast<int>(dirs.size()));
  int row = 0;
  for (const auto & kv : dirs) {
    const Eigen::VectorXd a = kv.second;
    A.row(row) = a.transpose();
    b(row) = support(a);
    ++row;
    A.row(row) = -a.transpose();
    b(row) = support(-a);
    ++row;
  }
  return Polytope(A, b);
}

bool Zonotope::contains(const Eigen::VectorXd & x, double tol) const
{
  const int n = dimension();
  const int m = numGenerators();
  if (x.size() != n) {return false;}
  // Feasibility LP: find z (size m) with G z = x - c and ||z||_inf <= 1.
  // The equality G z = x - c contributes n rows per sign (G is n x m, not
  // square in general); the box constraint contributes 2 m rows.
  Eigen::MatrixXd A(2 * n + 2 * m, m);
  Eigen::VectorXd b(2 * n + 2 * m);
  A.topRows(n) = generators_;
  b.head(n) = x - center_;
  A.middleRows(n, n) = -generators_;
  b.segment(n, n) = center_ - x;
  A.middleRows(2 * n, m) = Eigen::MatrixXd::Identity(m, m);
  b.segment(2 * n, m) = Eigen::VectorXd::Ones(m);
  A.bottomRows(m) = -Eigen::MatrixXd::Identity(m, m);
  b.tail(m) = Eigen::VectorXd::Ones(m);
  b.head(2 * n) += Eigen::VectorXd::Constant(2 * n, tol);
  const Eigen::VectorXd c = Eigen::VectorXd::Zero(m);
  return solveLp(A, b, c).status == LpResult::Status::kOptimal;
}

double Zonotope::maxNorm() const
{
  // Over-approximation: ||c + G z||_2 <= ||c||_2 + sum_j ||g_j||_2 for
  // ||z||_inf <= 1 (each generator's worst case contributes its 2-norm).
  double s = center_.norm();
  for (int j = 0; j < numGenerators(); ++j) {
    s += generators_.col(j).norm();
  }
  return s;
}

// ===========================================================================
// RPI computation
// ===========================================================================

/// True when W is an axis-aligned box centred on the origin (its support
/// function is then h_W(d) = sum_j w_j |d_j| and the alpha stopping rule in
/// computeRpiSet certifies A_cl^s W subseteq alpha W exactly — see below).
bool isAxisAlignedBox(const Zonotope & W)
{
  const int n = W.dimension();
  if (W.center().norm() > 1e-12) {return false;}
  if (W.numGenerators() != n) {return false;}
  for (int j = 0; j < n; ++j) {
    int nz = 0;
    for (int i = 0; i < n; ++i) {
      if (std::abs(W.generators()(i, j)) > 1e-12) {++nz;}
    }
    if (nz != 1) {return false;}
  }
  return true;
}

/// LP-free RPI certificate: checks A_cl Omega_z (+) W subseteq Omega_p by
/// comparing closed-form zonotope supports in every half-space direction of
/// the polytope Omega_p. This is the fast path of computeRpiSet and of
/// TubeMpcCbfSolver::verifyInvariance; it is sound exactly when the returned
/// RpiResult::exact is true (F_s unreduced) and W is an axis-aligned box —
/// callers must guard on those conditions before trusting it.
bool zonotopeRpiCertificate(
  const Eigen::MatrixXd & A_cl, const Zonotope & Omega_z, const Polytope & Omega_p,
  const Zonotope & W, double tol)
{
  if (Omega_p.numHalfspaces() == 0) {return true;}
  for (int i = 0; i < Omega_p.numHalfspaces(); ++i) {
    const Eigen::VectorXd a = Omega_p.A().row(i).transpose();
    const double h_map = Omega_z.support(A_cl.transpose() * a);
    const double h_w = W.support(a);
    if (h_map + h_w > Omega_p.b()(i) + tol) {
      fprintf(
        stderr,
        "[tube_mpc_cbf_solver] zonotopeRpiCertificate: violated at row %d "
        "(h_map %.3e + h_W %.3e > b %.3e)\n",
        i, h_map, h_w, Omega_p.b()(i));
      return false;
    }
  }
  return true;
}

RpiResult computeRpiSet(
  const Eigen::MatrixXd & A_cl, const Zonotope & W, double epsilon, int max_iterations)
{
  const int n = static_cast<int>(A_cl.rows());
  if (n == 0 || A_cl.cols() != n || W.dimension() != n) {
    throw std::invalid_argument("computeRpiSet: dimension mismatch");
  }
  RpiResult out;
  if (epsilon <= 0.0 || max_iterations < 1) {
    throw std::invalid_argument("computeRpiSet: epsilon > 0 and max_iterations >= 1");
  }
  if (spectralRadius(A_cl) >= 1.0 - 1e-12) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] computeRpiSet: A_cl is not Schur stable "
      "(spectral radius %.6f >= 1) — no finite RPI set\n",
      spectralRadius(A_cl));
    return out;  // converged = false, empty set
  }
  // F_s = (+)_{j=0}^{s-1} A_cl^j W, maintained incrementally as a zonotope.
  Zonotope F(Eigen::VectorXd::Zero(n), Eigen::MatrixXd::Zero(n, 0));
  Eigen::MatrixXd Aprev = Eigen::MatrixXd::Identity(n, n);  // A_cl^{s-1}
  Eigen::VectorXd Msum = Eigen::VectorXd::Zero(n);
  double alpha = 0.0;
  bool reduced = false;  // F_s has been order-reduced at least once
  for (int s = 1; s <= max_iterations; ++s) {
    F = F.minkowskiSum(W.linearMap(Aprev));
    if (F.numGenerators() > kRpiGeneratorCap) {
      F = F.reduceOrder(kRpiGeneratorCap);
      reduced = true;
    }
    for (int i = 0; i < n; ++i) {
      const Eigen::VectorXd a = Aprev.transpose() * Eigen::VectorXd::Unit(n, i);
      Msum(i) += W.support(a) + W.support(-a);
    }
    const Eigen::MatrixXd Apow = Aprev * A_cl;  // A_cl^s
    alpha = 0.0;
    bool alpha_finite = true;
    for (int i = 0; i < n; ++i) {
      const Eigen::VectorXd ei = Eigen::VectorXd::Unit(n, i);
      const double hw_e = W.support(ei);
      const double hw_d = W.support(Apow.transpose() * ei);
      if (hw_e > kZeroTol) {
        alpha = std::max(alpha, hw_d / hw_e);
      } else if (hw_d > kZeroTol) {
        alpha_finite = false;  // disturbance leaks into a degenerate axis
      }
    }
    if (!alpha_finite || alpha >= 1.0) {
      Aprev = Apow;
      continue;
    }
    const double M = Msum.maxCoeff();
    if (alpha / (1.0 - alpha) * M <= epsilon) {
      // Raković et al. (2005): once the stopping rule fires, Omega = F_s /
      // (1 - alpha) is an RPI candidate whose Hausdorff distance to the
      // true mRPI is bounded by epsilon. The exact certificate is re-checked
      // in TubeMpcCbfSolver::initialize (verifyInvariance) as a final gate
      // so no per-iteration certificate is needed here.
      const double scale = 1.0 / (1.0 - alpha);
      out.zonotope_set = Zonotope(F.center() * scale, F.generators() * scale);
      out.set = out.zonotope_set.toPolytope();
      out.iterations = s;
      out.alpha = alpha;
      out.exact = !reduced;
      out.converged = true;
      break;
    }
    Aprev = Apow;
  }
  if (!out.converged) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] computeRpiSet: did not converge within "
      "%d iterations (last alpha=%.6f)\n",
      max_iterations, alpha);
  }
  return out;
}

bool isRobustPositivelyInvariant(
  const Eigen::MatrixXd & A_cl, const Polytope & Omega, const Polytope & W, double tol)
{
  if (Omega.numHalfspaces() == 0) {return true;}
  const int n = Omega.dimension();
  if (A_cl.rows() != n || A_cl.cols() != n || W.dimension() != n) {
    throw std::invalid_argument("isRobustPositivelyInvariant: dimension mismatch");
  }
  for (int i = 0; i < Omega.numHalfspaces(); ++i) {
    const Eigen::VectorXd a = Omega.A().row(i).transpose();
    const double h_map = Omega.support(A_cl.transpose() * a);
    const double h_w = W.support(a);
    if (h_map + h_w > Omega.b()(i) + tol) {
      fprintf(
        stderr,
        "[tube_mpc_cbf_solver] isRobustPositivelyInvariant: violated at row %d "
        "(h_map %.3e + h_W %.3e > b %.3e)\n",
        i, h_map, h_w, Omega.b()(i));
      return false;
    }
  }
  return true;
}

Eigen::MatrixXd discreteLqrGain(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & R, int max_iterations, double tol)
{
  const int n = static_cast<int>(A.rows());
  const int m = static_cast<int>(B.cols());
  if (A.cols() != n || B.rows() != n || Q.rows() != n || Q.cols() != n ||
    R.rows() != m || R.cols() != m)
  {
    throw std::invalid_argument("discreteLqrGain: dimension mismatch");
  }
  Eigen::MatrixXd P = Q;
  bool converged = false;
  Eigen::MatrixXd BtPB;
  for (int it = 0; it < max_iterations; ++it) {
    BtPB = R + B.transpose() * P * B;
    Eigen::LLT<Eigen::MatrixXd> llt(BtPB);
    if (llt.info() != Eigen::Success) {
      fprintf(
        stderr,
        "[tube_mpc_cbf_solver] discreteLqrGain: R + B^T P B is not positive "
        "definite at iteration %d — stopping\n",
        it);
      return Eigen::MatrixXd::Zero(m, n);
    }
    const Eigen::MatrixXd AtPA = A.transpose() * P * A;
    const Eigen::MatrixXd AtPB = A.transpose() * P * B;
    const Eigen::MatrixXd S = llt.solve(B.transpose() * P * A);
    const Eigen::MatrixXd P_new = Q + AtPA - AtPB * S;
    if ((P_new - P).cwiseAbs().maxCoeff() < tol) {
      P = P_new;
      converged = true;
      break;
    }
    P = P_new;
  }
  if (!converged) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] discreteLqrGain: Riccati iteration did not "
      "converge within %d iterations (max |dP| = %.3e) — using the last "
      "iterate\n",
      max_iterations, tol);
  }
  BtPB = R + B.transpose() * P * B;
  Eigen::LLT<Eigen::MatrixXd> llt(BtPB);
  if (llt.info() != Eigen::Success) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] discreteLqrGain: final R + B^T P B is not "
      "positive definite\n");
    return Eigen::MatrixXd::Zero(m, n);
  }
  const Eigen::MatrixXd K = -llt.solve(B.transpose() * P * A);
  if (spectralRadius(A + B * K) >= 1.0 - 1e-12) {
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] discreteLqrGain: WARNING A + B K is not Schur "
      "stable (spectral radius %.6f) — the caller must reject this gain\n",
      spectralRadius(A + B * K));
  }
  return K;
}

double lipschitzBoundOnTube(
  const Eigen::VectorXd & z, const Polytope & Omega,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & barrier_gradient)
{
  if (Omega.dimension() > 3) {
    // Vertex sampling is impossible above 3-D; return +inf so the tube solver
    // refuses kLipschitz instead of under-approximating the bound.
    fprintf(
      stderr,
      "[tube_mpc_cbf_solver] lipschitzBoundOnTube: dimension %d > 3 — "
      "returning +inf (kLipschitz tightening is unavailable)\n",
      Omega.dimension());
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::MatrixXd verts = Omega.vertices();
  double best = 0.0;
  for (int i = 0; i < verts.cols(); ++i) {
    best = std::max(best, barrier_gradient(z + verts.col(i)).norm());
  }
  return best;
}

// ===========================================================================
// TubeMpcCbfSolver
// ===========================================================================

namespace
{

// Mirror of the helpers in mpc_cbf_solver.cpp (anonymous namespace, so each
// TU keeps its own copy — the shared parts are duplicated deliberately rather
// than exported: 08_TUBE.md §8.2 wants the obstacle loop identical, and a
// shared header would drift faster than two greppable copies).

  std::string toLower(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
  });
    return s;
  }

  bool reject(const std::string & field, const std::string & value)
  {
    std::fprintf(stderr, "[tube_mpc_cbf_solver] invalid %s: %s\n", field.c_str(), value.c_str());
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

  const char * tubeModelKey(ModelType model)
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

#if MPC_CBF_WITH_ACADOS
  const char * tightKey(TightenMode mode)
  {
    switch (mode) {
        case TightenMode::kSupportFunction:
          return "support_function";
        case TightenMode::kLipschitz:
          return "lipschitz";
        case TightenMode::kNone:
          return "none";
    }
    return "unknown_tighten";
  }
#endif  // MPC_CBF_WITH_ACADOS

// Large finite upper bound matching CONSTRAINT_UB in the code generator; never
// std::numeric_limits<double>::infinity() (16_CONVENTIONS.md §16.4). Must
// exceed the squared-distance barrier at the far-away dummy obstacles
// h ~ 2e12 (position 1e6, §6.2), or the initial iterate is infeasible and the
// QP dies with HPIPM_MINSTEP.
  constexpr double kTubeConstraintUb = 1.0e13;
  constexpr double kTubeActiveTolerance = 1.0e-3;

// ---------------------------------------------------------------------------
// Discrete-time model for K and the RPI set (04_MODELS.md §4.3).
// ---------------------------------------------------------------------------

  Eigen::VectorXd continuousDynamics(
    ModelType model, const Eigen::VectorXd & x, const Eigen::VectorXd & u)
  {
    switch (model) {
        case ModelType::kUnicycle2D: {
          Eigen::VectorXd xd(3);
          xd(0) = u(0) * std::cos(x(2));
          xd(1) = u(0) * std::sin(x(2));
          xd(2) = u(1);
          return xd;
        }
        case ModelType::kBicycleKinematic: {
          constexpr double kWheelbase = 0.35;
          Eigen::VectorXd xd(4);
          xd(0) = x(3) * std::cos(x(2));
          xd(1) = x(3) * std::sin(x(2));
          xd(2) = x(3) * std::tan(u(1)) / kWheelbase;
          xd(3) = u(0);
          return xd;
        }
        case ModelType::kQuadrotorPlanar: {
          constexpr double kMass = 1.0;
          constexpr double kInertia = 0.01;
          constexpr double kGravity = 9.81;
          Eigen::VectorXd xd(6);
          xd(0) = x(2);
          xd(1) = x(3);
          xd(2) = -u(0) * std::sin(x(4)) / kMass;
          xd(3) = u(0) * std::cos(x(4)) / kMass - kGravity;
          xd(4) = x(5);
          xd(5) = u(1) / kInertia;
          return xd;
        }
        case ModelType::kDoubleIntegrator2D:
        default:
          throw std::invalid_argument("continuousDynamics: unexpected model");
    }
  }

/// Explicit RK4 step, one step per dt — matches `discretise(spec, dt, "rk4")`
/// in codegen/models.py.
  Eigen::VectorXd rk4Step(
    ModelType model, double dt, const Eigen::VectorXd & x, const Eigen::VectorXd & u)
  {
    const Eigen::VectorXd k1 = continuousDynamics(model, x, u);
    const Eigen::VectorXd k2 = continuousDynamics(model, x + 0.5 * dt * k1, u);
    const Eigen::VectorXd k3 = continuousDynamics(model, x + 0.5 * dt * k2, u);
    const Eigen::VectorXd k4 = continuousDynamics(model, x + dt * k3, u);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }

/// Central finite-difference Jacobian of the RK4 map about (x_op, u_op).
/// Matches `linearised_discrete()` in codegen/models.py up to the ~1e-6
/// truncation error of the differencing.
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearisedAbout(
    ModelType model, double dt, const Eigen::VectorXd & x_op, const Eigen::VectorXd & u_op)
  {
    const int nx = static_cast<int>(x_op.size());
    const int nu = static_cast<int>(u_op.size());
    Eigen::MatrixXd A(nx, nx);
    Eigen::MatrixXd B(nx, nu);
    constexpr double kStep = 1.0e-6;
    for (int j = 0; j < nx; ++j) {
      Eigen::VectorXd xp = x_op;
      Eigen::VectorXd xm = x_op;
      xp(j) += kStep;
      xm(j) -= kStep;
      A.col(j) = (rk4Step(model, dt, xp, u_op) - rk4Step(model, dt, xm, u_op)) / (2.0 * kStep);
    }
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd up = u_op;
      Eigen::VectorXd um = u_op;
      up(j) += kStep;
      um(j) -= kStep;
      B.col(j) = (rk4Step(model, dt, x_op, up) - rk4Step(model, dt, x_op, um)) / (2.0 * kStep);
    }
    return {A, B};
  }

// ---------------------------------------------------------------------------
// acados capsule access (mirrors mpc_cbf_solver.cpp).
// ---------------------------------------------------------------------------
#if MPC_CBF_WITH_ACADOS
  struct TubeAcadosSolverApi
  {
    void * (*create_capsule)() = nullptr;
    int (*create)(void *) = nullptr;
    int (*solve)(void *) = nullptr;
    int (*update_params)(void *, int, double *, int) = nullptr;
    // acados' generated free functions return int; declaring them void here
    // would make the cast below change the return type too, which is UB when
    // called through (-Wcast-function-type). The result is a status code we
    // have no recovery for during teardown, so it is discarded at the call site.
    int (*free_solver)(void *) = nullptr;
    int (*free_capsule)(void *) = nullptr;
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
    api.free_solver = reinterpret_cast<int (*)(void *)>(NAME ## _acados_free); \
    api.free_capsule = reinterpret_cast<int (*)(void *)>(NAME ## _acados_free_capsule); \
    api.get_config = reinterpret_cast<ocp_nlp_config * (*)(void *)>(NAME ## _acados_get_nlp_config); \
    api.get_dims = reinterpret_cast<ocp_nlp_dims * (*)(void *)>(NAME ## _acados_get_nlp_dims); \
    api.get_in = reinterpret_cast<ocp_nlp_in * (*)(void *)>(NAME ## _acados_get_nlp_in); \
    api.get_out = reinterpret_cast<ocp_nlp_out * (*)(void *)>(NAME ## _acados_get_nlp_out); \
    api.get_solver = reinterpret_cast<ocp_nlp_solver * (*)(void *)>(NAME ## _acados_get_nlp_solver); \
    api.get_opts = reinterpret_cast<void * (*)(void *)>(NAME ## _acados_get_nlp_opts); \
    return true; \
  }

  bool loadTubeAcadosApi(const std::string & solver_name, TubeAcadosSolverApi & api)
  {
  // The configurations actually generated by codegen --all (§5.7). A request
  // for any other (model, horizon, tighten mode) fails initialize() with a
  // clear message instead of linking a solver that was never generated.
    MPC_CBF_BIND_SOLVER(tube_mpc_cbf_double_integrator_2d_N8_support_function)
    MPC_CBF_BIND_SOLVER(tube_mpc_cbf_double_integrator_2d_N8_lipschitz)
    MPC_CBF_BIND_SOLVER(tube_mpc_cbf_double_integrator_2d_N8_none)
    return false;
  }
#endif  // MPC_CBF_WITH_ACADOS

}  // namespace

struct TubeMpcCbfSolver::Impl
{
  MpcConfig mpc;
  CbfConfig cbf;
  TubeConfig tube;
  bool initialized{false};

  int nx{0};
  int nu{0};
  int np{0};          ///< Parameter vector length per stage (65 for n_obs=8, §5.3).
  int cbf_horizon{0}; ///< Resolved: in [1, mpc.horizon].

  Eigen::MatrixXd A_lin;      ///< Linearised (or exact, for the double
  Eigen::MatrixXd B_lin;      ///< integrator) discrete-time model used for K/RPI.
  Eigen::MatrixXd K;          ///< Ancillary gain actually in use.
  Polytope rpi;               ///< Omega.
  Zonotope rpi_zonotope;      ///< Omega in generator form (exact maxNorm for
                              ///< the kLipschitz tightening — the polytope's
                              ///< maxNorm under-approximates in >3-D).
  bool rpi_exact{true};       ///< RpiResult::exact: F_s was never reduced, so
                              ///< the geometric-series RPI identity applies
                              ///< (see zonotopeRpiCertificate).
  Polytope tightened_x;       ///< X (-) Omega.
  Polytope tightened_u;       ///< U (-) K Omega.
  Eigen::MatrixXd z_previous; ///< Last nominal trajectory, for the z_0 policy.
  bool has_previous{false};
  int tube_resets{0};         ///< Diagnostic: z_0 policy resets (08_TUBE.md §8.5).

  Eigen::MatrixXd x_ref;      ///< nx x (N+1) reference trajectory.
  Eigen::MatrixXd z_guess;    ///< Warm start, nx x (N+1); empty when unset.
  Eigen::MatrixXd v_guess;    ///< Warm start, nu x N; empty when unset.
  std::vector<double> parameter_buffer;  ///< (N+1)*np; reused every solve.
  std::vector<std::pair<double, int>> prune_order;
  Eigen::VectorXd yref;       ///< nx + nu stage reference, reused every stage.
  std::vector<int> idx0;      ///< Full state index set for the initial equality.
  std::array<ObstacleState, MpcCbfSolver::kMaxObstacles> kept_obstacles;
  double inflation{0.0};      ///< ego_radius + safety_margin, applied once (§6.4).

#if MPC_CBF_WITH_ACADOS
  void * capsule{nullptr};
  TubeAcadosSolverApi api;
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  ocp_nlp_solver * nlp_solver{nullptr};

  // Solution pool: identical rationale to MpcCbfSolver::Impl — the hot
  // path must not allocate.
  std::array<TubeMpcCbfSolution, 12> solution_pool;
  size_t pool_head{0};
#endif
};

// ---------------------------------------------------------------------------
// TubeMpcCbfSolution
// ---------------------------------------------------------------------------

bool TubeMpcCbfSolution::usable() const
{
  if (status == SolverStatus::kSuccess) {
    return true;
  }
  if (status == SolverStatus::kMaxIterations) {
    return u_applied.allFinite() && z_pred.allFinite();
  }
  return false;
}

// ---------------------------------------------------------------------------
// Construction / lifetime
// ---------------------------------------------------------------------------

TubeMpcCbfSolver::TubeMpcCbfSolver(
  const MpcConfig & mpc_config, const CbfConfig & cbf_config, const TubeConfig & tube_config)
: impl_(std::make_unique<Impl>())
{
  impl_->mpc = mpc_config;
  impl_->cbf = cbf_config;
  impl_->tube = tube_config;
  impl_->nx = MpcCbfSolver::stateDimOf(mpc_config.model);
  impl_->nu = MpcCbfSolver::inputDimOf(mpc_config.model);
}

TubeMpcCbfSolver::~TubeMpcCbfSolver()
{
#if MPC_CBF_WITH_ACADOS
  if (impl_ && impl_->capsule != nullptr) {
    if (impl_->api.free_solver != nullptr) {
      static_cast<void>(impl_->api.free_solver(impl_->capsule));
    }
    if (impl_->api.free_capsule != nullptr) {
      static_cast<void>(impl_->api.free_capsule(impl_->capsule));
    }
    impl_->capsule = nullptr;
  }
#endif
}

TubeMpcCbfSolver::TubeMpcCbfSolver(TubeMpcCbfSolver &&) noexcept = default;
TubeMpcCbfSolver & TubeMpcCbfSolver::operator=(TubeMpcCbfSolver &&) noexcept = default;

bool TubeMpcCbfSolver::initialize()
{
  if (impl_->initialized) {
    return true;
  }
  const auto & mpc = impl_->mpc;
  const auto & cbf = impl_->cbf;
  const auto & tube = impl_->tube;
  const int nx = impl_->nx;
  const int nu = impl_->nu;
  const int N = mpc.horizon;

  // --- 1. configuration validation, exactly as MpcCbfSolver::initialize -----
  // (06_SOLVER.md §6.3; the 1e-9 slack on omega_max * gamma is load-bearing —
  // see the comment in mpc_cbf_solver.cpp).
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
  // kNone is the deliberately-unsafe ablation (08_TUBE.md §8.4): refuse it
  // unless the caller set allow_unsafe_ablation explicitly, so it cannot be
  // reached from a launch file by accident.
  if (tube.tighten_mode == TightenMode::kNone && !tube.allow_unsafe_ablation) {
    return reject("tube.tighten_mode (kNone requires allow_unsafe_ablation)",
      toString(tube.tighten_mode));
  }
  if (tube.disturbance_set.dimension() != nx) {
    return reject("tube.disturbance_set dimension (expected " + std::to_string(nx) + ")",
      tube.disturbance_set.dimension());
  }
  if (!(tube.rpi_epsilon > 0.0) || tube.rpi_max_iterations < 1) {
    return reject("tube.rpi_epsilon / rpi_max_iterations (must be > 0 / >= 1)",
      tube.rpi_epsilon);
  }
  if (tube.compute_gain_from_lqr) {
    if (static_cast<int>(tube.lqr_Q.size()) != nx) {
      return reject("tube.lqr_Q.size() (expected " + std::to_string(nx) + ")",
        static_cast<int>(tube.lqr_Q.size()));
    }
    if (static_cast<int>(tube.lqr_R.size()) != nu) {
      return reject("tube.lqr_R.size() (expected " + std::to_string(nu) + ")",
        static_cast<int>(tube.lqr_R.size()));
    }
  } else {
    if (tube.K.rows() != nu || tube.K.cols() != nx) {
      return reject("tube.K size (expected " + std::to_string(nu) + " x " +
          std::to_string(nx) + ")", std::to_string(tube.K.rows()) + "/" +
        std::to_string(tube.K.cols()));
    }
  }

  impl_->cbf_horizon = (cbf.cbf_horizon <= 0) ? N : std::min(cbf.cbf_horizon, N);
  if (impl_->cbf_horizon < 1) {
    impl_->cbf_horizon = 1;
  }
  impl_->np = 7 * MpcCbfSolver::kMaxObstacles + 1 + MpcCbfSolver::kMaxObstacles;  // 65 (§5.3)
  impl_->inflation = cbf.ego_radius + cbf.safety_margin;

  // Scratch, sized once here (solve() must not allocate, 06_SOLVER.md §6.3).
  impl_->x_ref.setZero(nx, N + 1);
  impl_->z_guess.resize(0, 0);
  impl_->v_guess.resize(0, 0);
  impl_->parameter_buffer.assign(static_cast<size_t>(N + 1) * static_cast<size_t>(impl_->np), 0.0);
  impl_->prune_order.reserve(16);
  impl_->yref.setZero(nx + nu);
  impl_->idx0.resize(static_cast<size_t>(nx));

  // --- 2. (A_lin, B_lin) ----------------------------------------------------
  // Exact ZOH for the double integrator; otherwise linearise the RK4 map about
  // the reference. The certificate is exact in the first case and LOCAL in the
  // second (08_TUBE.md §8.3, §8.7) — say so in the log.
  if (mpc.model == ModelType::kDoubleIntegrator2D) {
    const int dim = nx / 2;
    impl_->A_lin = Eigen::MatrixXd::Zero(nx, nx);
    impl_->B_lin = Eigen::MatrixXd::Zero(nx, nu);
    impl_->A_lin.topLeftCorner(dim, dim).setIdentity();
    impl_->A_lin.topRightCorner(dim, dim) =
      mpc.dt * Eigen::MatrixXd::Identity(dim, dim);
    impl_->A_lin.bottomRightCorner(dim, dim).setIdentity();
    impl_->B_lin.topRows(dim) =
      0.5 * mpc.dt * mpc.dt * Eigen::MatrixXd::Identity(dim, dim);
    impl_->B_lin.bottomRows(dim) = mpc.dt * Eigen::MatrixXd::Identity(dim, dim);
  } else {
    const Eigen::VectorXd u_op = Eigen::VectorXd::Zero(nu);
    const auto ab = linearisedAbout(mpc.model, mpc.dt, impl_->x_ref.col(0), u_op);
    impl_->A_lin = ab.first;
    impl_->B_lin = ab.second;
    std::fprintf(stderr,
      "[tube_mpc_cbf_solver] WARNING: model %s — RPI certificate is LOCAL to "
      "the linearisation point; recompute Omega when the reference moves "
      "(08_TUBE.md §8.7)\n",
      tubeModelKey(mpc.model));
  }

  // --- 3. K -----------------------------------------------------------------
  if (tube.compute_gain_from_lqr) {
    const Eigen::MatrixXd Q_d =
      Eigen::Map<const Eigen::VectorXd>(tube.lqr_Q.data(), nx).asDiagonal();
    const Eigen::MatrixXd R_d =
      Eigen::Map<const Eigen::VectorXd>(tube.lqr_R.data(), nu).asDiagonal();
    impl_->K = discreteLqrGain(impl_->A_lin, impl_->B_lin, Q_d, R_d);
  } else {
    impl_->K = tube.K;
  }
  const double rho = spectralRadius(impl_->A_lin + impl_->B_lin * impl_->K);
  if (rho >= 1.0 - 1.0e-9) {
    return reject("A + B K spectral radius (must be < 1 - 1e-9)", rho);
  }

  // --- 4. Omega = mRPI outer approximation ----------------------------------
  const RpiResult rpi = computeRpiSet(impl_->A_lin + impl_->B_lin * impl_->K,
    tube.disturbance_set, tube.rpi_epsilon, tube.rpi_max_iterations);
  if (!rpi.converged) {
    return reject("RPI convergence (epsilon unreachable within rpi_max_iterations)",
      rpi.iterations);
  }
  impl_->rpi = rpi.set;
  impl_->rpi_zonotope = rpi.zonotope_set;
  impl_->rpi_exact = rpi.exact;

  // --- 5. Tightened state and input sets ------------------------------------
  // X is a box, so X (-) Omega is the per-axis erosion b_i -= h_Omega(a_i)
  // (08_TUBE.md §8.1). omegaSupport() evaluates the same certified set as the
  // barrier margin (§8.4) — the exact zonotope when unreduced, the polytope
  // otherwise — so no LP runs on the (possibly many-facet) polytope here.
  {
    const Polytope X_box = Polytope::box(mpc.x_min, mpc.x_max);
    Eigen::MatrixXd A_x = X_box.A();
    Eigen::VectorXd b_x = X_box.b();
    for (int i = 0; i < A_x.rows(); ++i) {
      b_x(i) -= omegaSupport(A_x.row(i).transpose());
    }
    impl_->tightened_x = Polytope(A_x, b_x);
  }
  if (impl_->tightened_x.isEmpty()) {
    return reject("tightened state set X (-) Omega is empty (the tube does not "
      "fit between the state bounds)", "");
  }
  // U (-) K Omega is exact: U is a box, and the Pontryagin difference of a box
  // with any set is the box whose per-axis bounds are shrunk by the support of
  // K Omega = the support of Omega along K^T e_i (08_TUBE.md §8.1).
  {
    const Polytope U_box = Polytope::box(mpc.u_min, mpc.u_max);
    Eigen::MatrixXd A_u = U_box.A();
    Eigen::VectorXd b_u = U_box.b();
    for (int i = 0; i < nu; ++i) {
      const Eigen::VectorXd e = Eigen::VectorXd::Unit(nu, i);
      b_u(i) -= omegaSupport(impl_->K.transpose() * e);
      b_u(nu + i) -= omegaSupport(-impl_->K.transpose() * e);
    }
    impl_->tightened_u = Polytope(A_u, b_u);
  }
  if (impl_->tightened_u.isEmpty()) {
    return reject("tightened input set U (-) K Omega is empty (the disturbance "
      "exceeds the actuator authority — a physical statement, 08_TUBE.md §8.3)",
      "");
  }

  // --- 6. Certificate -------------------------------------------------------
  if (!verifyInvariance(1.0e-6)) {
    return reject("RPI certificate (isRobustPositivelyInvariant)", "");
  }

  // --- 7. Log the RPI numbers (go straight into the tube_robustness caption).
  {
    const auto bb = impl_->rpi.boundingBox();
    std::string lo_s, hi_s;
    for (int i = 0; i < nx; ++i) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.4f", bb.first(i));
      lo_s += std::string(buf) + (i + 1 < nx ? ", " : "");
      std::snprintf(buf, sizeof(buf), "%.4f", bb.second(i));
      hi_s += std::string(buf) + (i + 1 < nx ? ", " : "");
    }
    std::fprintf(stderr,
      "[tube_mpc_cbf_solver] RPI: s=%d alpha=%.6f Omega box=[%s] x [%s], "
      "||K||=%.4f, tighten=%s\n",
      rpi.iterations, rpi.alpha, lo_s.c_str(), hi_s.c_str(), impl_->K.norm(),
      toString(tube.tighten_mode));
  }

#if !MPC_CBF_WITH_ACADOS
  // Stub build: report honestly. The tube tests use initialize()'s false
  // return to GTEST_SKIP the acados-dependent cases (10_TESTS.md §10.2). The
  // RPI data computed above stays available through the accessors either way.
  static bool warned = false;
  if (!warned) {
    std::fprintf(stderr,
      "[tube_mpc_cbf_solver] built without MPC_CBF_WITH_ACADOS; solver not initialized\n");
    warned = true;
  }
  return false;
#else
  // --- 8. Allocate the acados tube solver with the TIGHTENED bounds. --------
  const std::string solver_name = std::string("tube_mpc_cbf_") + tubeModelKey(mpc.model) +
    "_N" + std::to_string(N) + "_" + tightKey(tube.tighten_mode);
  if (!loadTubeAcadosApi(solver_name, impl_->api)) {
    return reject("solver name (not generated; run codegen/generate_tube_solver.py "
      "--all for this configuration)", solver_name);
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

  // Pre-size every pool slot so make_solution()'s fills never allocate.
  {
    for (auto & sol : impl_->solution_pool) {
      sol.u_applied.setZero(nu);
      sol.v0.setZero(nu);
      sol.z_pred.setZero(nx, N + 1);
      sol.v_pred.setZero(nu, N);
      sol.robust_cbf_values.assign(
        static_cast<size_t>(N + 1) * MpcCbfSolver::kMaxObstacles, 0.0);
      sol.tightening.assign(
        static_cast<size_t>(N + 1) * MpcCbfSolver::kMaxObstacles, 0.0);
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
    // See mpc_cbf_solver.cpp for the LM rationale: 1e-4 leaves HPIPM's
    // Newton system ill-conditioned on degenerate (grazing/tight-gamma) CBF
    // QPs, producing spurious ACADOS_MINSTEP -> QP_FAILURE. 1e-2 regularizes
    // the singular directions without disturbing feasible KKT points beyond
    // the CBF safety margins. Must match the value used by mpc_cbf_solver.cpp
    // and the codegen (05_CODEGEN.md §5.4).
    double lm = 1.0e-2;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "levenberg_marquardt", &lm);
    int warm_start = 1;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "qp_warm_start", &warm_start);
    // See mpc_cbf_solver.cpp: HPIPM's codegen default (50) is too few for the
    // CBF QPs on some seeds; bump the cap so the inner QP can reach the NLP
    // complementarity tolerance instead of stalling at qp_iter_max.
    int qp_max_iter = 500;
    ocp_nlp_solver_opts_set(impl_->nlp_config, opts, "qp_iter_max", &qp_max_iter);
    // Degenerate grazing QPs floor res_comp at ~2e-4 (see mpc_cbf_solver.cpp
    // for the full rationale); loosen tol_comp so SQP returns ACADOS_SUCCESS.
    // All four NLP tolerances are loosened together, to the same values the
    // codegen bakes into the JSON (05_CODEGEN.md §5.4 step 8) — see
    // mpc_cbf_solver.cpp for why tol_stat=tol_eq=tol_ineq=1e-3 is required
    // for consistent SQP termination at grazing points (a tight tol_stat
    // makes HPIPM stall at qp_iter_max and the SQP diverge from a good KKT
    // point; safety is checked on the returned trajectory, not residuals).
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

  // --- cost weights: W = blkdiag(Q, R) --------------------------------------
  {
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(nx + nu, nx + nu);
    W.topLeftCorner(nx, nx) = Eigen::Map<const Eigen::VectorXd>(mpc.Q.data(), nx).asDiagonal();
    W.block(nx, nx, nu, nu) = Eigen::Map<const Eigen::VectorXd>(mpc.R.data(), nu).asDiagonal();
    for (int k = 0; k < N; ++k) {
      ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, k, "W", W.data());
    }
    Eigen::MatrixXd We = Eigen::Map<const Eigen::VectorXd>(mpc.Qf.data(), nx).asDiagonal();
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, N, "W", We.data());
  }

  // --- state bounds: the TIGHTENED ones (X (-) Omega) -----------------------
  // A formerly "unbounded" axis (±1e9) becomes finite after the erosion; it is
  // still skipped when its erosion is exactly zero (degenerate W axis), which
  // keeps the ±1e9 sentinel out of the QP (16_CONVENTIONS.md §16.4).
  {
    std::vector<int> idxbx;
    std::vector<double> lbx, ubx;
    for (int i = 0; i < nx; ++i) {
      const double lo = -impl_->tightened_x.b()(nx + i);
      const double hi = impl_->tightened_x.b()(i);
      if (lo > -kTubeConstraintUb && hi < kTubeConstraintUb) {
        idxbx.push_back(i);
        lbx.push_back(lo);
        ubx.push_back(hi);
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

  // --- input bounds: the TIGHTENED ones (U (-) K Omega) ---------------------
  {
    std::vector<int> idxbu(static_cast<size_t>(nu));
    std::vector<double> lbu(static_cast<size_t>(nu));
    std::vector<double> ubu(static_cast<size_t>(nu));
    for (int i = 0; i < nu; ++i) {
      idxbu[static_cast<size_t>(i)] = i;
      lbu[static_cast<size_t>(i)] = -impl_->tightened_u.b()(nu + i);
      ubu[static_cast<size_t>(i)] = impl_->tightened_u.b()(i);
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
  std::fprintf(stderr, "[tube_mpc_cbf_solver] initialized %s (nx=%d, nu=%d, N=%d, np=%d)\n",
    solver_name.c_str(), nx, nu, N, impl_->np);
  return true;
#endif  // MPC_CBF_WITH_ACADOS
}

bool TubeMpcCbfSolver::isInitialized() const
{
  return impl_->initialized;
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

TubeMpcCbfSolution TubeMpcCbfSolver::solve(
  const Eigen::VectorXd & x0, const std::vector<ObstacleState> & obstacles)
{
  const int nx = impl_->nx;
  const int nu = impl_->nu;
  const int N = impl_->mpc.horizon;
  const int n_obs = MpcCbfSolver::kMaxObstacles;

  // Pooled-solution helper (same rationale as MpcCbfSolver::solve).
  auto make_solution = [&](SolverStatus st) -> TubeMpcCbfSolution {
#if MPC_CBF_WITH_ACADOS
      TubeMpcCbfSolution & sol = impl_->solution_pool[impl_->pool_head];
      impl_->pool_head = (impl_->pool_head + 1) % impl_->solution_pool.size();
      sol.status = st;
      sol.u_applied.setZero(nu);
      sol.v0.setZero(nu);
      sol.z_pred.setZero(nx, N + 1);
      sol.v_pred.setZero(nu, N);
      sol.robust_cbf_values.assign(
        static_cast<size_t>(N + 1) * static_cast<size_t>(n_obs), 0.0);
      sol.tightening.assign(static_cast<size_t>(N + 1) * static_cast<size_t>(n_obs), 0.0);
      sol.diagnostics.solve_time_ms = 0.0;
      sol.diagnostics.sqp_iterations = 0;
      sol.diagnostics.kkt_residual = 0.0;
      sol.diagnostics.cost = 0.0;
      sol.diagnostics.first_active_cbf_step = -1;
      sol.diagnostics.first_active_obstacle = -1;
      sol.diagnostics.infeasibility_reason.clear();
      sol.diagnostics.clip_count = 0;
      sol.diagnostics.tube_resets = 0;
      return std::move(sol);
#else
      TubeMpcCbfSolution sol;
      sol.status = st;
      sol.u_applied.setZero(nu);
      sol.v0.setZero(nu);
      sol.z_pred.setZero(nx, N + 1);
      sol.v_pred.setZero(nu, N);
      sol.robust_cbf_values.assign(
        static_cast<size_t>(N + 1) * static_cast<size_t>(n_obs), 0.0);
      sol.tightening.assign(static_cast<size_t>(N + 1) * static_cast<size_t>(n_obs), 0.0);
      return sol;
#endif
    };

  // 1. Guards (06_SOLVER.md §6.5).
  if (!impl_->initialized) {
    return make_solution(SolverStatus::kNotInitialized);
  }
  if (x0.size() != nx || !x0.allFinite()) {
    TubeMpcCbfSolution sol = make_solution(SolverStatus::kNanDetected);
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
  const auto & tube = impl_->tube;

  // 2. z_0 policy (08_TUBE.md §8.5): anchor to the shifted nominal state when
  //    the true state is still inside the tube, reset to x0 otherwise.
  Eigen::VectorXd z0 = x0;
  if (impl_->has_previous && impl_->rpi.contains(x0 - impl_->z_previous.col(1))) {
    z0 = impl_->z_previous.col(1);
  } else {
    if (impl_->has_previous) {
      ++impl_->tube_resets;
      std::fprintf(stderr,
        "[tube_mpc_cbf_solver] tube reset: x0 - z_prev(1) left Omega "
        "(resets so far: %d)\n",
        impl_->tube_resets);
    }
  }

  // 3. Obstacle pruning — identical to MpcCbfSolver::solve (§6.4).
  {
    const int n_in = static_cast<int>(obstacles.size());
    const int n_keep = std::min(n_in, n_obs);
    const double px = x0[0];
    const double py = x0[1];
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
    for (int j = n_keep; j < n_obs; ++j) {
      ObstacleState dummy;
      dummy.position = Eigen::Vector3d(1.0e6, 1.0e6, 1.0e6);
      dummy.velocity = Eigen::Vector3d::Zero();
      dummy.radius = 0.0;
      dummy.is_dynamic = false;
      impl_->kept_obstacles[static_cast<size_t>(j)] = dummy;
    }
  }

  // 4. Parameters: [obstacle block (7*n_obs)] + [gamma] + [c_0..c_{n_obs-1}]
  //    (§5.3, §8.5). The tightening c_{j,k} depends on the nominal state at
  //    stage k; before the solve it is taken from the shifted warm start (the
  //    previous trajectory), which is what keeps the c's consistent across
  //    steps. Stage 0 uses the fixed z0.
  {
    const bool have_z = (impl_->z_guess.rows() == nx && impl_->z_guess.cols() == N + 1);
    for (int k = 0; k <= N; ++k) {
      double * p = impl_->parameter_buffer.data() +
        static_cast<size_t>(k) * static_cast<size_t>(impl_->np);
      for (int j = 0; j < n_obs; ++j) {
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
      p[7 * n_obs] = cbf.gamma;
      const Eigen::VectorXd z_nominal = (k == 0) ? z0 :
        (have_z ? impl_->z_guess.col(k) : z0);
      for (int j = 0; j < n_obs; ++j) {
        p[7 * n_obs + 1 + j] = tighteningFor(z_nominal,
            impl_->kept_obstacles[static_cast<size_t>(j)]);
      }
      impl_->api.update_params(impl_->capsule, k, p, impl_->np);
    }
  }

  // 5. Reference, initial-state equality (z0), warm start.
  {
    impl_->yref.tail(nu).setZero();
    for (int k = 0; k < N; ++k) {
      impl_->yref.head(nx) = impl_->x_ref.col(k);
      ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, k,
        "yref", impl_->yref.data());
    }
    impl_->yref.head(nx) = impl_->x_ref.col(N);
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in, N,
      "yref", impl_->yref.data());

    for (int i = 0; i < nx; ++i) {
      impl_->idx0[static_cast<size_t>(i)] = i;
    }
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "idxbx", impl_->idx0.data());
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "lbx", const_cast<double *>(z0.data()));
    ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_in,
      impl_->nlp_out, 0, "ubx", const_cast<double *>(z0.data()));

    const bool have_v = (impl_->v_guess.rows() == nu && impl_->v_guess.cols() == N);
    const bool have_z = (impl_->z_guess.rows() == nx && impl_->z_guess.cols() == N + 1);
    if (have_z) {
      for (int k = 0; k <= N; ++k) {
        ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
          k, "x", impl_->z_guess.col(k).data());
      }
    }
    if (have_v) {
      for (int k = 0; k < N; ++k) {
        ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in,
          k, "u", impl_->v_guess.col(k).data());
      }
    }
  }

  // 6. Time the solve; call acados.
  TubeMpcCbfSolution sol = make_solution(SolverStatus::kSuccess);
  const auto t0 = std::chrono::steady_clock::now();
  const int acados_status = impl_->api.solve(impl_->capsule);
  const auto t1 = std::chrono::steady_clock::now();
  sol.diagnostics.solve_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 7. Read back the nominal trajectory.
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, k, "x",
      sol.z_pred.col(k).data());
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, k, "u",
      sol.v_pred.col(k).data());
  }
  sol.v0 = sol.v_pred.col(0).head(nu);

  // 8. u_applied = clip(v_0 + K (x0 - z_0), u_min, u_max). Count clips: a
  //    correctly computed tightening keeps the ancillary term inside U — a
  //    saturation voids the guarantee and must be surfaced (08_TUBE.md §8.5).
  sol.u_applied = sol.v0 + impl_->K * (x0 - z0);
  for (int i = 0; i < nu; ++i) {
    if (sol.u_applied(i) < impl_->mpc.u_min[i]) {
      sol.u_applied(i) = impl_->mpc.u_min[i];
      ++sol.diagnostics.clip_count;
    }
    if (sol.u_applied(i) > impl_->mpc.u_max[i]) {
      sol.u_applied(i) = impl_->mpc.u_max[i];
      ++sol.diagnostics.clip_count;
    }
  }

  // 9. Diagnostics (06_SOLVER.md §6.7 + the tube fields).
  {
    int iters = 0;
    double kkt = 0.0;
    double cost = 0.0;
    ocp_nlp_get(impl_->nlp_solver, "sqp_iter", &iters);
    // KKT residual = max entry of the residual vector (§6.7). v0.6.0 exposes
    // each residual block's inf-norm directly; the statistics table layout is
    // versioned and transposed, so avoid it.
    {
      double res_stat = 0.0, res_eq = 0.0, res_ineq = 0.0, res_comp = 0.0;
      ocp_nlp_get(impl_->nlp_solver, "res_stat", &res_stat);
      ocp_nlp_get(impl_->nlp_solver, "res_eq", &res_eq);
      ocp_nlp_get(impl_->nlp_solver, "res_ineq", &res_ineq);
      ocp_nlp_get(impl_->nlp_solver, "res_comp", &res_comp);
      kkt = std::max({res_stat, res_eq, res_ineq, res_comp});
    }
    ocp_nlp_eval_cost(impl_->nlp_solver, impl_->nlp_in, impl_->nlp_out);
    ocp_nlp_get(impl_->nlp_solver, "cost_value", &cost);
    sol.diagnostics.sqp_iterations = iters;
    sol.diagnostics.kkt_residual = kkt;
    sol.diagnostics.cost = cost;
    sol.diagnostics.tube_resets = impl_->tube_resets;

    const int hk = impl_->cbf_horizon;
    sol.diagnostics.cbf_values.assign(
      static_cast<size_t>(N + 1) * static_cast<size_t>(n_obs), 0.0);
    for (int k = 0; k <= N; ++k) {
      for (int j = 0; j < n_obs; ++j) {
        const double c = tighteningFor(sol.z_pred.col(k),
            impl_->kept_obstacles[static_cast<size_t>(j)]);
        sol.tightening[static_cast<size_t>(k) * static_cast<size_t>(n_obs) +
          static_cast<size_t>(j)] = c;
        const double h_raw = MpcCbfSolver::barrierValue(impl_->mpc.model, sol.z_pred.col(k),
          impl_->kept_obstacles[static_cast<size_t>(j)], impl_->inflation);
        sol.diagnostics.cbf_values[static_cast<size_t>(k) * static_cast<size_t>(n_obs) +
          static_cast<size_t>(j)] = h_raw;
        sol.robust_cbf_values[static_cast<size_t>(k) * static_cast<size_t>(n_obs) +
          static_cast<size_t>(j)] = h_raw - c;
      }
    }

    sol.diagnostics.cbf_slack.assign(
      static_cast<size_t>(hk) * static_cast<size_t>(n_obs), 0.0);
    double min_slack = std::numeric_limits<double>::infinity();
    int arg_k = -1;
    int arg_j = -1;
    for (int k = 0; k < hk; ++k) {
      for (int j = 0; j < n_obs; ++j) {
        const double h_k = sol.diagnostics.cbf_values[static_cast<size_t>(k) * n_obs +
            static_cast<size_t>(j)];
        const double h_k1 = sol.diagnostics.cbf_values[static_cast<size_t>(k + 1) * n_obs +
            static_cast<size_t>(j)];
        const double c = sol.tightening[static_cast<size_t>(k) * n_obs +
            static_cast<size_t>(j)];
        // Tube DCBF row: h_{k+1} - h_k + gamma * (h_k - c_k) >= 0.
        const double slack = h_k1 - h_k + cbf.gamma * (h_k - c);
        sol.diagnostics.cbf_slack[static_cast<size_t>(k) * n_obs +
          static_cast<size_t>(j)] = slack;
        if (slack < min_slack) {
          min_slack = slack;
          arg_k = k;
          arg_j = j;
        }
      }
    }
    if (min_slack < kTubeActiveTolerance) {
      sol.diagnostics.first_active_cbf_step = arg_k;
      sol.diagnostics.first_active_obstacle = arg_j;
    }
  }

  // 10. Map the acados return code (identical mapping to MpcCbfSolver).
  switch (acados_status) {
      case ACADOS_SUCCESS:
        sol.status = SolverStatus::kSuccess;
        break;
      case ACADOS_NAN_DETECTED:
        sol.status = SolverStatus::kNanDetected;
        break;
      case ACADOS_MAXITER:
        sol.status = SolverStatus::kMaxIterations;
        break;
      case ACADOS_INFEASIBLE:
      case ACADOS_QP_FAILURE:
        // The SQP layer masks an infeasible inner QP as ACADOS_QP_FAILURE
        // (ocp_nlp_sqp.c), so both codes mean the QP had no feasible point:
        // kInfeasible (see mpc_cbf_solver.cpp §6.5.1).
        sol.status = SolverStatus::kInfeasible;
        break;
      case ACADOS_MINSTEP:
      case ACADOS_READY:
      default:
        sol.status = SolverStatus::kQpFailure;
        break;
  }
  if (sol.status != SolverStatus::kSuccess) {
    sol.diagnostics.infeasibility_reason =
      "acados returned " + std::to_string(acados_status);
  }

  // 11. Store the nominal trajectory for the z_0 policy, and shift it as the
  //     next warm start (06_SOLVER.md §6.5 step 8).
  impl_->z_previous = sol.z_pred;
  impl_->has_previous = true;
  impl_->z_guess.resize(nx, N + 1);
  impl_->v_guess.resize(nu, N);
  impl_->z_guess.leftCols(N) = sol.z_pred.rightCols(N);
  impl_->z_guess.col(N) = sol.z_pred.col(N);
  impl_->v_guess = sol.v_pred.topRows(nu);
  if (!impl_->z_guess.allFinite() || !impl_->v_guess.allFinite()) {
    impl_->z_guess.resize(0, 0);
    impl_->v_guess.resize(0, 0);
  }

  return sol;
#else
  // Unreachable in the stub build (initialize() never succeeds).
  (void)obstacles;
  return make_solution(SolverStatus::kNotInitialized);
#endif  // MPC_CBF_WITH_ACADOS
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void TubeMpcCbfSolver::setReference(const Eigen::VectorXd & x_ref)
{
  if (x_ref.size() != impl_->nx) {
    std::fprintf(stderr, "[tube_mpc_cbf_solver] setReference: expected size %d, got %ld\n",
      impl_->nx, static_cast<long>(x_ref.size()));
    return;
  }
  impl_->x_ref = x_ref.replicate(1, impl_->mpc.horizon + 1);
}

void TubeMpcCbfSolver::setReferenceTrajectory(const Eigen::MatrixXd & x_ref_traj)
{
  if (x_ref_traj.rows() != impl_->nx) {
    std::fprintf(stderr,
      "[tube_mpc_cbf_solver] setReferenceTrajectory: expected %d rows, got %ld\n",
      impl_->nx, static_cast<long>(x_ref_traj.rows()));
    return;
  }
  const int N = impl_->mpc.horizon;
  impl_->x_ref.setZero(impl_->nx, N + 1);
  const int n_cols = std::min(static_cast<int>(x_ref_traj.cols()), N + 1);
  for (int k = 0; k < n_cols; ++k) {
    impl_->x_ref.col(k) = x_ref_traj.col(k);
  }
  for (int k = n_cols; k <= N; ++k) {
    impl_->x_ref.col(k) = x_ref_traj.col(n_cols - 1);
  }
}

void TubeMpcCbfSolver::reset()
{
  impl_->z_previous.resize(0, 0);
  impl_->has_previous = false;
  impl_->tube_resets = 0;
  impl_->z_guess.resize(0, 0);
  impl_->v_guess.resize(0, 0);
#if MPC_CBF_WITH_ACADOS
  if (impl_->initialized && impl_->nlp_out != nullptr) {
    for (int k = 0; k <= impl_->mpc.horizon; ++k) {
      Eigen::VectorXd z0 = Eigen::VectorXd::Zero(impl_->nx);
      Eigen::VectorXd v0 = Eigen::VectorXd::Zero(impl_->nu);
      ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, k,
        "x", const_cast<double *>(z0.data()));
      ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims, impl_->nlp_out, impl_->nlp_in, k,
        "u", v0.data());
    }
  }
#endif
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

const Polytope & TubeMpcCbfSolver::rpiSet() const
{
  return impl_->rpi;
}

const Eigen::MatrixXd & TubeMpcCbfSolver::ancillaryGain() const
{
  return impl_->K;
}

const Polytope & TubeMpcCbfSolver::tightenedStateSet() const
{
  return impl_->tightened_x;
}

const Polytope & TubeMpcCbfSolver::tightenedInputSet() const
{
  return impl_->tightened_u;
}

double TubeMpcCbfSolver::omegaSupport(const Eigen::VectorXd & direction) const
{
  return impl_->rpi_exact ? impl_->rpi_zonotope.support(direction) :
         impl_->rpi.support(direction);
}

double TubeMpcCbfSolver::tighteningFor(
  const Eigen::VectorXd & z, const ObstacleState & obstacle) const
{
  switch (impl_->tube.tighten_mode) {
      case TightenMode::kSupportFunction: {
      // h(x) = ||P x - p_obs||^2 - r^2, grad h(z) = 2 P^T (P z - p_obs). The
      // margin is h_Omega(-grad h(z)); the dropped quadratic term ||P e||^2 is
      // non-negative, so the margin over-estimates and stays sound (§8.4).
      // omegaSupport() evaluates the certified set (the exact zonotope when
      // unreduced — a closed-form support, not an LP) so the hot path stays
      // LP-free.
        Eigen::Vector2d pz(z(0), z(1));
        const Eigen::Vector2d p_obs(obstacle.position[0], obstacle.position[1]);
        Eigen::VectorXd grad = Eigen::VectorXd::Zero(impl_->nx);
        grad.head(2) = 2.0 * (pz - p_obs);
        const double c = omegaSupport(-grad);
        if (c < 0.0) {
        // 0 in Omega implies h_Omega(d) >= 0 for every d; a negative margin
        // would be a bug in support() or a non-origin-containing W.
          std::fprintf(stderr,
          "[tube_mpc_cbf_solver] tighteningFor: negative margin %.3e — W must "
          "contain the origin\n",
          c);
        }
        return c;
      }
      case TightenMode::kLipschitz:
      // maxNorm of the ZONOTOPE (sum of generator norms) is an over-approx;
      // Polytope::maxNorm on the 4-D conversion hits the grid fallback and
      // under-approximates, which would be unsound here (07_SETS.md §7.2).
        return impl_->tube.lipschitz_h * impl_->rpi_zonotope.maxNorm();
      case TightenMode::kNone:
        return 0.0;
  }
  return 0.0;  // unreachable
}

bool TubeMpcCbfSolver::verifyInvariance(double tol) const
{
  if (impl_->rpi.numHalfspaces() == 0) {
    return false;  // not initialised
  }
  // Fast path (no LP): same conditions under which computeRpiSet accepted the
  // set on the closed-form certificate — axis-aligned box W and an unreduced
  // F_s (the geometric-series identity then makes Omega RPI). Otherwise fall
  // back to the exact per-facet LP certificate.
  if (impl_->rpi_exact && isAxisAlignedBox(impl_->tube.disturbance_set)) {
    return zonotopeRpiCertificate(impl_->A_lin + impl_->B_lin * impl_->K,
      impl_->rpi_zonotope, impl_->rpi, impl_->tube.disturbance_set, tol);
  }
  const Polytope W_p = impl_->tube.disturbance_set.toPolytope();
  return isRobustPositivelyInvariant(
    impl_->A_lin + impl_->B_lin * impl_->K, impl_->rpi, W_p, tol);
}

const char * toString(TightenMode mode)
{
  switch (mode) {
      case TightenMode::kSupportFunction:
        return "support_function";
      case TightenMode::kLipschitz:
        return "lipschitz";
      case TightenMode::kNone:
        return "none";
  }
  return "unknown_tighten_mode";
}

bool parseTightenMode(const std::string & name, TightenMode & out)
{
  const std::string lower = toLower(name);
  if (lower == "support_function") {
    out = TightenMode::kSupportFunction;
    return true;
  }
  if (lower == "lipschitz") {
    out = TightenMode::kLipschitz;
    return true;
  }
  if (lower == "none") {
    out = TightenMode::kNone;
    return true;
  }
  return false;
}

}  // namespace mpc_cbf_unified
