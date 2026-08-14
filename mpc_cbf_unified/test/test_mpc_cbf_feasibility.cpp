// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// MpcCbfSolver tests. The fixture reproduces the
// ACC 2021 2-D example: double integrator, N = 8, dt = 0.1, start (0,0),
// goal (1,1), one static obstacle of radius 0.2 at (0.5, 0.5), u in [-1,1]^2,
// gamma = 0.3. The same numbers appear in the reproduction notebook — keep
// them identical.
//
// Stub build (MPC_CBF_WITH_ACADOS undefined): initialize() returns false, so
// every test that needs a live solver GTEST_SKIPs with a reason. The
// configuration-validation and barrier-sign tests run in both builds.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mpc_cbf_unified/mpc_cbf_solver.hpp"

using mpc_cbf_unified::CbfConfig;
using mpc_cbf_unified::CbfVariant;
using mpc_cbf_unified::ModelType;
using mpc_cbf_unified::MpcCbfSolver;
using mpc_cbf_unified::MpcCbfSolution;
using mpc_cbf_unified::MpcConfig;
using mpc_cbf_unified::ObstacleState;
using mpc_cbf_unified::SolverStatus;

// Fixed RNG seed, printed so a failing run is replayable.
constexpr unsigned int kRngSeed = 0xC0FFEEu;

namespace
{

// Allocation counter for SolveDoesNotAllocate (10_TESTS.md §10.1): override
// the global allocation functions so that a thread-local flag can arm
// counting around the measured solves. Everything else in the binary runs
// with the flag clear and pays nothing.
thread_local bool g_count_allocations = false;
std::atomic<std::size_t> g_allocation_count{0};

}  // namespace

// Global-scope replacements (they must live at global scope, not in a
// namespace, to replace the built-ins). malloc/free keep them ABI-compatible.
void * operator new(std::size_t size)
{
  if (g_count_allocations) {
    g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  }
  if (void * p = std::malloc(size)) {
    return p;
  }
  throw std::bad_alloc();
}

void operator delete(void * p) noexcept
{
  std::free(p);
}

// Sized deallocation (C++14): the compiler warns when a sized delete is
// missing for a replaced operator delete.
void operator delete(void * p, std::size_t) noexcept
{
  std::free(p);
}

void * operator new[](std::size_t size)
{
  if (g_count_allocations) {
    g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  }
  if (void * p = std::malloc(size)) {
    return p;
  }
  throw std::bad_alloc();
}

void operator delete[](void * p) noexcept
{
  std::free(p);
}

void operator delete[](void * p, std::size_t) noexcept
{
  std::free(p);
}

namespace
{

/// Canonical scenario shared by most tests: double integrator, N = 8, dt = 0.1,
/// start (0,0), goal (1,1), one static obstacle of radius 0.2 at (0.5, 0.5).
/// This is the ACC 2021 2-D example.
class MpcCbfFeasibilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::printf("MpcCbfFeasibilityTest RNG seed: 0x%08X\n", kRngSeed);

    // Defaults from config/mpc_cbf_params.yaml (ACC 2021 2-D example).
    mpc_config_.model = ModelType::kDoubleIntegrator2D;
    mpc_config_.horizon = 8;
    mpc_config_.dt = 0.1;
    mpc_config_.Q = (Eigen::VectorXd(4) << 10.0, 10.0, 1.0, 1.0).finished();
    mpc_config_.R = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
    mpc_config_.Qf = (Eigen::VectorXd(4) << 100.0, 100.0, 10.0, 10.0).finished();
    mpc_config_.x_min = (Eigen::VectorXd(4) << -1.0e9, -1.0e9, -2.0, -2.0).finished();
    mpc_config_.x_max = (Eigen::VectorXd(4) << 1.0e9, 1.0e9, 2.0, 2.0).finished();
    mpc_config_.u_min = (Eigen::VectorXd(2) << -1.0, -1.0).finished();
    mpc_config_.u_max = (Eigen::VectorXd(2) << 1.0, 1.0).finished();

    cbf_config_.variant = CbfVariant::kFixedDecay;
    cbf_config_.gamma = 0.3;
    cbf_config_.cbf_horizon = 0;  // full horizon
    cbf_config_.ego_radius = 0.15;
    cbf_config_.safety_margin = 0.05;

    ObstacleState obs;
    obs.position = Eigen::Vector3d(0.5, 0.5, 0.0);
    obs.velocity = Eigen::Vector3d::Zero();
    obs.radius = 0.2;
    obs.is_dynamic = false;
    obstacles_.push_back(obs);

    // Never ASSERT_TRUE here: the stub build's initialize() returns false by
    // design, and the acados-dependent tests GTEST_SKIP on initialized_.
    solver_ = std::make_unique<MpcCbfSolver>(mpc_config_, cbf_config_);
    initialized_ = solver_->initialize();
  }

  // Exact-ZOH double-integrator step (matches codegen's discretise() for the
  // double integrator): px += dt*vx + 0.5*dt^2*ax; vx += dt*ax, per axis.
  Eigen::VectorXd step(const Eigen::VectorXd & x, const Eigen::VectorXd & u) const
  {
    const double dt = mpc_config_.dt;
    Eigen::VectorXd xn(4);
    xn[0] = x[0] + dt * x[2] + 0.5 * dt * dt * u[0];
    xn[1] = x[1] + dt * x[3] + 0.5 * dt * dt * u[1];
    xn[2] = x[2] + dt * u[0];
    xn[3] = x[3] + dt * u[1];
    return xn;
  }

  // Barrier of the fixture obstacle at x, using the full inflation
  // (radius + ego_radius + safety_margin = 0.4).
  double h(const Eigen::VectorXd & x) const
  {
    const double r_eff =
      obstacles_[0].radius + cbf_config_.ego_radius + cbf_config_.safety_margin;
    const double dx = x[0] - obstacles_[0].position[0];
    const double dy = x[1] - obstacles_[0].position[1];
    return dx * dx + dy * dy - r_eff * r_eff;
  }

  Eigen::VectorXd goal() const
  {
    return (Eigen::VectorXd(4) << 1.0, 1.0, 0.0, 0.0).finished();
  }

  Eigen::VectorXd start() const
  {
    return (Eigen::VectorXd(4) << 0.0, 0.0, 0.0, 0.0).finished();
  }

  MpcConfig mpc_config_;
  CbfConfig cbf_config_;
  std::vector<ObstacleState> obstacles_;
  std::unique_ptr<MpcCbfSolver> solver_;
  bool initialized_{false};
};

}  // namespace

// ---------------------------------------------------------------------------
// Configuration validation
// ---------------------------------------------------------------------------

TEST_F(MpcCbfFeasibilityTest, RejectsGammaOutsideUnitInterval)
{
  // gamma = 0.0, -0.1 and 1.5 must all make initialize() return false.
  for (double gamma : {0.0, -0.1, 1.5}) {
    CbfConfig bad = cbf_config_;
    bad.gamma = gamma;
    MpcCbfSolver s(mpc_config_, bad);
    EXPECT_FALSE(s.initialize()) << "gamma = " << gamma;
  }

  // setGamma() must reject them without mutating the config.
  const double before = solver_->cbfConfig().gamma;
  EXPECT_FALSE(solver_->setGamma(0.0));
  EXPECT_FALSE(solver_->setGamma(-0.1));
  EXPECT_FALSE(solver_->setGamma(1.5));
  EXPECT_DOUBLE_EQ(solver_->cbfConfig().gamma, before);
}

TEST_F(MpcCbfFeasibilityTest, RejectsUnsafeOmegaBound)
{
  // kRelaxedDecay with omega_max * gamma > 1 must be rejected — that
  // combination allows h(x_{k+1}) < 0 and voids forward invariance.
  {
    CbfConfig bad = cbf_config_;
    bad.variant = CbfVariant::kRelaxedDecay;
    bad.omega_min = 0.0;
    bad.omega_max = 4.0;  // 4.0 * 0.3 = 1.2 > 1
    MpcCbfSolver s(mpc_config_, bad);
    EXPECT_FALSE(s.initialize());
  }

  // omega_max * gamma == 1.0 exactly must be ACCEPTED: the shipped defaults
  // sit on this boundary, so a strict comparison would reject the
  // repository's own configuration (06_SOLVER.md §6.3).
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable; boundary acceptance needs a live solver";
  }
  {
    CbfConfig ok = cbf_config_;
    ok.variant = CbfVariant::kRelaxedDecay;
    ok.gamma = 0.5;
    ok.omega_min = 0.0;
    ok.omega_max = 2.0;  // 0.5 * 2.0 = 1.0 exactly in IEEE double
    MpcCbfSolver s(mpc_config_, ok);
    EXPECT_TRUE(s.initialize());
  }
}

TEST_F(MpcCbfFeasibilityTest, RejectsMismatchedWeightDimensions)
{
  // Q of length nx-1 = 3 must make initialize() false.
  MpcConfig bad = mpc_config_;
  bad.Q = Eigen::VectorXd::Ones(3);
  MpcCbfSolver s(bad, cbf_config_);
  EXPECT_FALSE(s.initialize());
}

// ---------------------------------------------------------------------------
// Barrier definition
// ---------------------------------------------------------------------------

TEST_F(MpcCbfFeasibilityTest, BarrierValueSignConvention)
{
  // r_eff = 0.2 (obstacle) + 0.15 (ego) + 0.05 (margin) = 0.4.
  const double r_eff = 0.4;
  const double inflation = cbf_config_.ego_radius + cbf_config_.safety_margin;
  const ObstacleState & obs = obstacles_[0];

  // Strictly outside: distance 0.5 > 0.4.
  Eigen::VectorXd out(4);
  out << 1.0, 0.5, 0.0, 0.0;
  EXPECT_GT(MpcCbfSolver::barrierValue(mpc_config_.model, out, obs, inflation), 0.0);

  // On the boundary: distance == r_eff to 1e-9.
  Eigen::VectorXd on(4);
  on << 0.5 + r_eff, 0.5, 0.0, 0.0;
  EXPECT_NEAR(MpcCbfSolver::barrierValue(mpc_config_.model, on, obs, inflation), 0.0, 1.0e-9);

  // Inside: distance 0.1 < 0.4.
  Eigen::VectorXd in(4);
  in << 0.6, 0.5, 0.0, 0.0;
  EXPECT_LT(MpcCbfSolver::barrierValue(mpc_config_.model, in, obs, inflation), 0.0);
}

TEST_F(MpcCbfFeasibilityTest, BarrierMatchesGeneratedCode)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // 100 random states: |barrierValue(...) - h from diagnostics.cbf_values at
  // stage 0| < 1e-9. This is the guard against the C++ helper and the CasADi
  // expression drifting apart.
  solver_->setReference(goal());
  std::mt19937 rng(kRngSeed);
  std::uniform_real_distribution<double> pos(-1.5, 1.5);
  std::uniform_real_distribution<double> vel(-0.5, 0.5);
  int checked = 0;
  for (int i = 0; i < 1000 && checked < 100; ++i) {
    Eigen::VectorXd x(4);
    x << pos(rng), pos(rng), vel(rng), vel(rng);
    // Keep a safety margin so the state is comfortably feasible. h(x) >= 0.2
    // is NOT sufficient on its own: a draw with the velocity pointing hard at
    // the obstacle (|v| = 0.707 m/s toward the centre from |p-p_obs| = 0.6)
    // leaves h(x1) <= 0.1125 after the best one-step braking, below the
    // stage-0 DCBF requirement (1-gamma)*h(x) = 0.14. Such states exercise
    // infeasibility detection (A1), not barrier parity, so skip them with an
    // exact one-step feasibility check: with u pointing directly away from
    // the obstacle, h(F(x,u)) must reach (1-gamma)*h(x).
    if (h(x) < 0.2) {
      continue;
    }
    Eigen::VectorXd u_best(2);
    u_best[0] = (x[0] - obstacles_[0].position[0] + mpc_config_.dt * x[2]) > 0.0 ? 1.0 : -1.0;
    u_best[1] = (x[1] - obstacles_[0].position[1] + mpc_config_.dt * x[3]) > 0.0 ? 1.0 : -1.0;
    const Eigen::VectorXd x1_best = step(x, u_best);
    if (h(x1_best) < (1.0 - cbf_config_.gamma) * h(x) - 1.0e-9) {
      continue;
    }
    // The 100 draws are independent, so the previous solve's shifted
    // trajectory is a stale warm start here: its stage-0 column is unrelated
    // to x, and a trajectory that cuts through the obstacle region makes the
    // first QP's linearisation ill-conditioned (HPIPM MINSTEP at SQP
    // iteration 1). Drop it — barrier parity is the subject of this test, not
    // warm-start performance (06_SOLVER.md §6.5 step 4).
    solver_->reset();
    MpcCbfSolution sol = solver_->solve(x, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << "state " << i;
    const double h_gen = sol.diagnostics.cbf_values[0];  // stage 0, obstacle 0
    const double h_cpp = MpcCbfSolver::barrierValue(
      mpc_config_.model, x, obstacles_[0],
      cbf_config_.ego_radius + cbf_config_.safety_margin);
    EXPECT_NEAR(h_cpp, h_gen, 1.0e-9) << "state " << i;
    ++checked;
  }
  ASSERT_EQ(checked, 100);
}

// ---------------------------------------------------------------------------
// Safety and feasibility
// ---------------------------------------------------------------------------

TEST_F(MpcCbfFeasibilityTest, SolvesFromSafeInitialState)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  solver_->setReference(goal());
  MpcCbfSolution sol = solver_->solve(start(), obstacles_);
  ASSERT_EQ(sol.status, SolverStatus::kSuccess);
  EXPECT_EQ(sol.x_pred.cols(), mpc_config_.horizon + 1);  // N+1 columns
  ASSERT_EQ(sol.u0.size(), 2);
  EXPECT_GE(sol.u0[0], mpc_config_.u_min[0] - 1.0e-9);
  EXPECT_LE(sol.u0[0], mpc_config_.u_max[0] + 1.0e-9);
  EXPECT_GE(sol.u0[1], mpc_config_.u_min[1] - 1.0e-9);
  EXPECT_LE(sol.u0[1], mpc_config_.u_max[1] + 1.0e-9);
  EXPECT_TRUE(sol.x_pred.allFinite());
  EXPECT_TRUE(sol.u_pred.allFinite());
}

TEST_F(MpcCbfFeasibilityTest, DcbfConstraintHoldsOverHorizon)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // The DCBF condition must hold on EVERY solve of the closed-loop rollout,
  // not merely the first: h(x_{k+1}) - h(x_k) >= -gamma*h(x_k) - 1e-6.
  solver_->setReference(goal());
  Eigen::VectorXd x = start();
  for (int t = 0; t < 20; ++t) {
    MpcCbfSolution sol = solver_->solve(x, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << "rollout step " << t;
    const int N_cbf = mpc_config_.horizon;  // cbf_horizon = 0 => full horizon
    for (int k = 0; k < N_cbf; ++k) {
      const double h_k = h(sol.x_pred.col(k));
      const double h_k1 = h(sol.x_pred.col(k + 1));
      EXPECT_GE(h_k1 - h_k, -cbf_config_.gamma * h_k - 1.0e-6)
        << "rollout step " << t << ", k = " << k;
    }
    x = step(x, sol.u0);
  }
}

TEST_F(MpcCbfFeasibilityTest, ClosedLoopStaysSafeForFullRollout)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // 200-step closed-loop rollout applying u0 to the true dynamics. Assert
  // min h >= -1e-6 and that the goal is reached within 0.05 m. Record min h
  // and the step count — REPRODUCTION_REPORT.md quotes them.
  solver_->setReference(goal());
  Eigen::VectorXd x = start();
  double min_h = 1.0e30;
  int goal_step = -1;
  for (int t = 0; t < 200; ++t) {
    MpcCbfSolution sol = solver_->solve(x, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << "rollout step " << t;
    min_h = std::min(min_h, h(x));
    x = step(x, sol.u0);
    if (goal_step < 0 && (x.head(2) - goal().head(2)).norm() < 0.05) {
      goal_step = t;
    }
  }
  std::printf("ClosedLoopStaysSafeForFullRollout: min h = %.6g, goal at step %d\n",
    min_h, goal_step);
  EXPECT_GE(min_h, -1.0e-6);
  EXPECT_GE(goal_step, 0) << "goal (1,1) not reached within 0.05 m in 200 steps";
}

TEST_F(MpcCbfFeasibilityTest, SmallerGammaAvoidsEarlier)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // gamma = 0.1 must give a larger minimum clearance and a longer path than
  // gamma = 0.9 from the same start. Assert the ordering, not absolute
  // numbers (ACC 2021 Fig. 4 is qualitative).
  auto rollout = [&](double gamma) {
      CbfConfig cfg = cbf_config_;
      cfg.gamma = gamma;
      MpcCbfSolver s(mpc_config_, cfg);
      EXPECT_TRUE(s.initialize());
      s.setReference(goal());
      Eigen::VectorXd x = start();
      Eigen::VectorXd x_prev = x;
      double min_h = 1.0e30;
      double path = 0.0;
      for (int t = 0; t < 200; ++t) {
        MpcCbfSolution sol = s.solve(x, obstacles_);
        if (sol.status != SolverStatus::kSuccess) {
          break;
        }
        min_h = std::min(min_h, h(x));
        x = step(x, sol.u0);
        path += (x.head(2) - x_prev.head(2)).norm();
        x_prev = x;
      }
      return std::make_pair(min_h, path);
    };

  const auto small = rollout(0.1);
  const auto large = rollout(0.9);
  std::printf("SmallerGammaAvoidsEarlier: gamma=0.1 (min h %.6g, path %.6g), "
              "gamma=0.9 (min h %.6g, path %.6g)\n",
    small.first, small.second, large.first, large.second);
  EXPECT_GT(small.first, large.first);    // more clearance
  EXPECT_GT(small.second, large.second);  // longer path
}

TEST_F(MpcCbfFeasibilityTest, DistanceOnlyBaselineFailsWhereCbfSucceeds)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // A5. Same scenario with a short horizon (N = 3, the generated distance_only
  // configuration): kDistanceOnly either reports infeasible or produces
  // min h < 0 over the closed loop; kFixedDecay stays safe. Same cost, same
  // bounds, same horizon — the only difference is the missing decay row
  // (05_CODEGEN.md §5.5).
  MpcConfig m3 = mpc_config_;
  m3.horizon = 3;
  CbfConfig dc = cbf_config_;
  dc.variant = CbfVariant::kDistanceOnly;
  MpcCbfSolver baseline(m3, dc);
  ASSERT_TRUE(baseline.initialize()) << "N=3 distance_only solver must exist (05_CODEGEN.md §5.6)";
  baseline.setReference(goal());

  Eigen::VectorXd x = start();
  bool infeasible = false;
  double min_h_dc = 1.0e30;
  for (int t = 0; t < 200; ++t) {
    MpcCbfSolution sol = baseline.solve(x, obstacles_);
    if (sol.status != SolverStatus::kSuccess) {
      infeasible = true;
      break;
    }
    min_h_dc = std::min(min_h_dc, h(x));
    x = step(x, sol.u0);
  }
  std::printf("DistanceOnlyBaseline: infeasible=%d, min h = %.6g\n",
    static_cast<int>(infeasible), min_h_dc);
  EXPECT_TRUE(infeasible || min_h_dc < 0.0)
    << "MPC-DC stayed feasible and safe — scenario too easy, tune N/obstacle "
       "per 10_TESTS.md §10.1 (A5)";

  // kFixedDecay from the same start must stay safe.
  solver_->setReference(goal());
  Eigen::VectorXd x2 = start();
  double min_h_cbf = 1.0e30;
  for (int t = 0; t < 200; ++t) {
    MpcCbfSolution sol = solver_->solve(x2, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << "CBF rollout step " << t;
    min_h_cbf = std::min(min_h_cbf, h(x2));
    x2 = step(x2, sol.u0);
  }
  EXPECT_GE(min_h_cbf, -1.0e-6);
}

TEST_F(MpcCbfFeasibilityTest, RelaxedDecayRecoversFeasibilityAtTightGamma)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // A6 (CDC 2021 central claim): a state for which kFixedDecay is infeasible
  // must be feasible under kRelaxedDecay, with min h >= 0 in the prediction.
  //
  // Scenario: start at (0.05, 0.5) moving at 0.3 m/s straight at the obstacle
  // centre (0.5, 0.5). h(x0) = 0.0425 is small, so fixed decay gamma = 0.3
  // demands h(x1) >= 0.7*h(x0) = 0.02975 (distance >= sqrt(0.18975) = 0.4356);
  // the best one-step braking reaches only |p1 - p_obs| = 0.425 (h = 0.0206),
  // so kFixedDecay is infeasible. Relaxed decay can push omega up to
  // omega*gamma -> 0.9, lowering the requirement to h(x1) >= 0.1*h(x0) =
  // 0.00425 (distance >= 0.4053), which is reachable — the recovery needs
  // omega ~ 1.7 > 1. (A velocity of 0.5 m/s was tried first: even relaxed
  // then needs h(x1) >= 0.00425 but max reachable h(x1) = 0.00405, i.e. the
  // scenario was infeasible for BOTH variants — 10_TESTS.md §10.1 A6.)
  Eigen::VectorXd x0(4);
  x0 << 0.05, 0.5, 0.3, 0.0;

  {
    CbfConfig fixed = cbf_config_;  // kFixedDecay, gamma = 0.3
    MpcCbfSolver s(mpc_config_, fixed);
    ASSERT_TRUE(s.initialize());
    s.setReference(goal());
    MpcCbfSolution sol = s.solve(x0, obstacles_);
    std::printf("RelaxedDecayRecovers: fixed_decay status = %d (%s), reason: %s\n",
      static_cast<int>(sol.status), mpc_cbf_unified::toString(sol.status),
      sol.diagnostics.infeasibility_reason.c_str());
    EXPECT_EQ(sol.status, SolverStatus::kInfeasible)
      << "fixed_decay must be infeasible from this state (10_TESTS.md §10.1, A6)";
  }

  {
    CbfConfig relaxed = cbf_config_;
    relaxed.variant = CbfVariant::kRelaxedDecay;
    relaxed.omega_min = 0.0;
    relaxed.omega_max = 3.0;  // 3.0 * 0.3 = 0.9 <= 1, and the recovery needs omega > 1
    relaxed.omega_weight = 1.0e3;
    MpcCbfSolver s(mpc_config_, relaxed);
    ASSERT_TRUE(s.initialize());
    s.setReference(goal());
    MpcCbfSolution sol = s.solve(x0, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << sol.diagnostics.infeasibility_reason;

    double min_h_pred = 1.0e30;
    for (int k = 0; k <= mpc_config_.horizon; ++k) {
      min_h_pred = std::min(min_h_pred, h(sol.x_pred.col(k)));
    }
    std::printf("RelaxedDecayRecovers: min h in prediction = %.6g\n", min_h_pred);
    EXPECT_GE(min_h_pred, -1.0e-6);

    // The recovery must not cheat: omega * gamma <= 1 + 1e-9 everywhere (A6).
    for (double omega : sol.diagnostics.omega) {
      EXPECT_LE(omega * cbf_config_.gamma, 1.0 + 1.0e-9) << "omega = " << omega;
    }
  }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

TEST_F(MpcCbfFeasibilityTest, ReportsFirstActiveConstraint)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // Drive the vehicle straight at the obstacle: start at the origin with
  // velocity pointing at (0.5, 0.5). The tightest DCBF row must be reported.
  //
  // The velocity is tuned so the tightest row lands inside the horizon with a
  // robustly conditioned first QP: at v = (0.55, 0.55) the vehicle brakes
  // toward the obstacle each step and the minimum DCBF slack sits at stage 2.
  //
  // A "stage-0 just-active" velocity (v = 0.588) was tried first: the stage-0
  // row then demands h(x1) >= 0.7*h(x0) = 0.238 and the best one-step braking
  // reaches only h(x1) = 2*0.4462^2 - 0.16 = 0.2382, i.e. slack ~ 1.9e-4 <
  // kActiveTolerance. The state is feasible, but the first QP (linearised
  // around the constant-x0 initial guess with u = 0) has a razor-thin
  // feasible set for u0 (the linearised row requires u_x + u_y <= -1.97), and
  // HPIPM collapses with MINSTEP at SQP iteration 1 — a numerical artifact of
  // sitting exactly on the feasibility boundary, not an infeasibility. The
  // diagnostics assertions need a well-conditioned solve, so the scenario
  // keeps a comfortable margin instead (10_TESTS.md §10.1 A7).
  solver_->setReference(goal());
  Eigen::VectorXd x0(4);
  x0 << 0.0, 0.0, 0.55, 0.55;
  MpcCbfSolution sol = solver_->solve(x0, obstacles_);
  ASSERT_EQ(sol.status, SolverStatus::kSuccess) << sol.diagnostics.infeasibility_reason;

  EXPECT_GE(sol.diagnostics.first_active_cbf_step, 0);
  EXPECT_EQ(sol.diagnostics.first_active_obstacle, 0);
  // The reported step must be the argmin of cbf_slack.
  if (sol.diagnostics.first_active_cbf_step >= 0) {
    const auto & slack = sol.diagnostics.cbf_slack;
    const auto it = std::min_element(slack.begin(), slack.end());
    ASSERT_NE(it, slack.end());
    const std::size_t argmin =
      static_cast<std::size_t>(std::distance(slack.begin(), it));
    EXPECT_EQ(static_cast<int>(argmin / MpcCbfSolver::kMaxObstacles),
      sol.diagnostics.first_active_cbf_step);
  }
}

TEST_F(MpcCbfFeasibilityTest, InfeasibilityReasonIsPopulated)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // Start inside the obstacle -> kInfeasible, reason non-empty and naming the
  // obstacle index.
  solver_->setReference(goal());
  Eigen::VectorXd x0(4);
  x0 << 0.5, 0.5, 0.0, 0.0;  // obstacle centre
  MpcCbfSolution sol = solver_->solve(x0, obstacles_);
  EXPECT_EQ(sol.status, SolverStatus::kInfeasible);
  EXPECT_FALSE(sol.diagnostics.infeasibility_reason.empty());
  EXPECT_NE(sol.diagnostics.infeasibility_reason.find("obstacle 0"),
    std::string::npos) << sol.diagnostics.infeasibility_reason;
}

TEST_F(MpcCbfFeasibilityTest, RejectsNonFiniteState)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // NaN in x0 -> kNanDetected, no crash, no acados call.
  solver_->setReference(goal());
  Eigen::VectorXd x0(4);
  x0 << 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0;
  MpcCbfSolution sol = solver_->solve(x0, obstacles_);
  EXPECT_EQ(sol.status, SolverStatus::kNanDetected);
  EXPECT_FALSE(sol.diagnostics.infeasibility_reason.empty());
}

// ---------------------------------------------------------------------------
// Runtime contract
// ---------------------------------------------------------------------------

TEST_F(MpcCbfFeasibilityTest, HandlesMoreObstaclesThanSlots)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // kMaxObstacles + 4 = 12 obstacles: the nearest kMaxObstacles must be the
  // ones reflected in diagnostics.cbf_values, and the solve must succeed.
  std::vector<ObstacleState> many;
  std::mt19937 rng(kRngSeed);
  const double kTwoPi = 2.0 * std::acos(-1.0);
  std::uniform_real_distribution<double> angle(0.0, kTwoPi);
  // 8 obstacles at distance ~1.2 in random directions.
  for (int j = 0; j < 8; ++j) {
    ObstacleState o;
    const double a = angle(rng);
    o.position = Eigen::Vector3d(1.2 * std::cos(a), 1.2 * std::sin(a), 0.0);
    o.radius = 0.1;
    many.push_back(o);
  }
  // 4 far obstacles: distance ~50.
  for (int j = 0; j < 4; ++j) {
    ObstacleState o;
    o.position = Eigen::Vector3d(50.0 + j, -50.0 - j, 0.0);
    o.radius = 0.1;
    many.push_back(o);
  }
  ASSERT_EQ(static_cast<int>(many.size()), MpcCbfSolver::kMaxObstacles + 4);

  solver_->setReference(goal());
  MpcCbfSolution sol = solver_->solve(start(), many);
  ASSERT_EQ(sol.status, SolverStatus::kSuccess) << sol.diagnostics.infeasibility_reason;

  // The 8 nearest by squared distance from x0 must be exactly the kept ones.
  std::vector<std::pair<double, int>> order;
  order.reserve(many.size());
  for (std::size_t j = 0; j < many.size(); ++j) {
    const double dx = many[j].position[0] - 0.0;
    const double dy = many[j].position[1] - 0.0;
    order.emplace_back(dx * dx + dy * dy, static_cast<int>(j));
  }
  std::sort(order.begin(), order.end());
  const double inflation = cbf_config_.ego_radius + cbf_config_.safety_margin;
  for (int j = 0; j < MpcCbfSolver::kMaxObstacles; ++j) {
    const ObstacleState & kept =
      many[static_cast<std::size_t>(order[static_cast<std::size_t>(j)].second)];
    const double h_expected =
      MpcCbfSolver::barrierValue(mpc_config_.model, start(), kept, inflation);
    EXPECT_NEAR(h_expected, sol.diagnostics.cbf_values[static_cast<std::size_t>(j)], 1.0e-9)
      << "slot " << j;
  }
}

TEST_F(MpcCbfFeasibilityTest, WarmStartReducesIterations)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // Solve twice from the same state; the second call (warm started from the
  // first) must use no more SQP iterations than the first.
  solver_->setReference(goal());
  MpcCbfSolution sol1 = solver_->solve(start(), obstacles_);
  ASSERT_EQ(sol1.status, SolverStatus::kSuccess);
  MpcCbfSolution sol2 = solver_->solve(start(), obstacles_);
  ASSERT_EQ(sol2.status, SolverStatus::kSuccess);
  std::printf("WarmStartReducesIterations: %d -> %d SQP iterations\n",
    sol1.diagnostics.sqp_iterations, sol2.diagnostics.sqp_iterations);
  EXPECT_LE(sol2.diagnostics.sqp_iterations, sol1.diagnostics.sqp_iterations);
}

TEST_F(MpcCbfFeasibilityTest, SolveMeetsRealTimeBudget)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // 100 solves with use_rti = true; p95 solve_time_ms < 10 ms (dt = 0.1 s =>
  // 10% of the control period). Loosen only with a comment explaining the
  // hardware (14_CI.md §14.4).
  MpcConfig rti = mpc_config_;
  rti.use_rti = true;
  rti.max_sqp_iterations = 1;
  MpcCbfSolver s(rti, cbf_config_);
  ASSERT_TRUE(s.initialize());
  s.setReference(goal());

  std::vector<double> times;
  times.reserve(100);
  Eigen::VectorXd x = start();
  for (int i = 0; i < 100; ++i) {
    MpcCbfSolution sol = s.solve(x, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess) << "solve " << i;
    times.push_back(sol.diagnostics.solve_time_ms);
    x = step(x, sol.u0);
  }
  std::sort(times.begin(), times.end());
  const double p95 = times[94];  // 0-indexed 95th percentile of 100 samples
  std::printf("SolveMeetsRealTimeBudget: p95 = %.3f ms\n", p95);
  EXPECT_LT(p95, 10.0) << "p95 solve time exceeded 10 ms";
}

TEST_F(MpcCbfFeasibilityTest, SolveDoesNotAllocate)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  // Global operator new counter, warm up with one solve, then zero
  // allocations across the next 10 solves. Guards the real-time claim made
  // in the header.
  solver_->setReference(goal());

  // x0 is built before the counter is armed — Eigen::VectorXd(4) allocates.
  const Eigen::VectorXd x0 = start();

  // Warm-up: allocation is allowed here (first solve sizes internal buffers).
  {
    MpcCbfSolution sol = solver_->solve(x0, obstacles_);
    ASSERT_EQ(sol.status, SolverStatus::kSuccess);
  }

  std::array<SolverStatus, 10> statuses;
  g_allocation_count = 0;
  g_count_allocations = true;
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    // No ASSERT/EXPECT inside the counted region — the gtest machinery
    // allocates. Capture, then check after the counter is disarmed.
    statuses[i] = solver_->solve(x0, obstacles_).status;
  }
  g_count_allocations = false;

  EXPECT_EQ(g_allocation_count, 0u) << "solve() must not allocate after warm-up";
  for (SolverStatus s : statuses) {
    EXPECT_EQ(s, SolverStatus::kSuccess);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
