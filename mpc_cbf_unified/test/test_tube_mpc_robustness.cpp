// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Two layers are tested here: the convex-set machinery
// (cheap, exact, must be bullet-proof because everything above it inherits its
// errors) and the tube solver's robustness claim.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include "mpc_cbf_unified/disturbance_sets.hpp"
#include "mpc_cbf_unified/mpc_cbf_solver.hpp"
#include "mpc_cbf_unified/tube_mpc_cbf_solver.hpp"

using mpc_cbf_unified::CbfConfig;
using mpc_cbf_unified::CbfVariant;
using mpc_cbf_unified::computeRpiSet;
using mpc_cbf_unified::discreteLqrGain;
using mpc_cbf_unified::isRobustPositivelyInvariant;
using mpc_cbf_unified::MpcCbfSolver;
using mpc_cbf_unified::MpcConfig;
using mpc_cbf_unified::ModelType;
using mpc_cbf_unified::ObstacleState;
using mpc_cbf_unified::Polytope;
using mpc_cbf_unified::TightenMode;
using mpc_cbf_unified::TubeConfig;
using mpc_cbf_unified::TubeMpcCbfSolution;
using mpc_cbf_unified::TubeMpcCbfSolver;
using mpc_cbf_unified::Zonotope;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kRngSeed = 0xC0FFEEu;

/// Spectral radius via Eigen (the solver TU keeps its own private copy; tests
/// must not depend on it).
double spectralRadius(const Eigen::MatrixXd & M)
{
  Eigen::EigenSolver<Eigen::MatrixXd> es(M);
  return es.eigenvalues().cwiseAbs().maxCoeff();
}

/// Double-integrator ZOH dynamics shared by the RPI and closed-loop tests
/// (exact ZOH, codegen/models.py _exact_zoh): dt = 0.1.
struct DoubleIntegrator
{
  static constexpr double dt = 0.1;

  static Eigen::MatrixXd A()
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
    A.topLeftCorner(2, 2).setIdentity();
    A.topRightCorner(2, 2) = dt * Eigen::MatrixXd::Identity(2, 2);
    A.bottomRightCorner(2, 2).setIdentity();
    return A;
  }

  static Eigen::MatrixXd B()
  {
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 2);
    B.topRows(2) = 0.5 * dt * dt * Eigen::MatrixXd::Identity(2, 2);
    B.bottomRows(2) = dt * Eigen::MatrixXd::Identity(2, 2);
    return B;
  }
};

/// Fixture disturbance set: W = box(0.005, 0.005, 0.02, 0.02) — matches
/// config/tube_mpc_params.yaml after the M7 fix (a velocity half-width of 0.1
/// makes U (-) K Omega empty).
Eigen::Vector4d fixtureWHalfWidths()
{
  return Eigen::Vector4d(0.005, 0.005, 0.02, 0.02);
}

}  // namespace

// ---------------------------------------------------------------------------
// Polytope / Zonotope primitives
// ---------------------------------------------------------------------------

TEST(PolytopeTest, BoxSupportFunctionIsExact)
{
  const Polytope unit = Polytope::box(Eigen::VectorXd::Ones(3));
  std::mt19937 rng(0x5EED);
  std::uniform_real_distribution<double> unif(-2.0, 2.0);
  for (int trial = 0; trial < 20; ++trial) {
    Eigen::VectorXd d(3);
    for (int i = 0; i < 3; ++i) {
      d(i) = unif(rng);
    }
    EXPECT_NEAR(unit.support(d), d.cwiseAbs().sum(), 1e-9)
      << "direction " << d.transpose();
  }
}

TEST(PolytopeTest, MinkowskiSumOfBoxesIsABox)
{
  const Eigen::Vector3d a(1.0, 0.5, 2.0);
  const Eigen::Vector3d b(0.25, 1.5, 0.75);
  const Polytope S = Polytope::box(a).minkowskiSum(Polytope::box(b));
  const Polytope ref = Polytope::box(a + b);

  // Exact for axis-aligned boxes: support functions add in every axis
  // direction, and the box is symmetric.
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(S.support(Eigen::Vector3d::Unit(i)), (a + b)(i), 1e-9);
    EXPECT_NEAR(S.support(-Eigen::Vector3d::Unit(i)), (a + b)(i), 1e-9);
  }
  // Set equality via vertices (both are 8-vertex boxes).
  const Eigen::MatrixXd vs = S.vertices();
  const Eigen::MatrixXd vr = ref.vertices();
  ASSERT_EQ(vr.cols(), vs.cols());
  for (int i = 0; i < vs.cols(); ++i) {
    EXPECT_TRUE(ref.contains(vs.col(i), 1e-9)) << "S vertex " << i;
  }
  for (int i = 0; i < vr.cols(); ++i) {
    EXPECT_TRUE(S.contains(vr.col(i), 1e-9)) << "ref vertex " << i;
  }
}

TEST(PolytopeTest, PontryaginDifferenceIsInverseOfSumForBoxes)
{
  const Eigen::Vector3d a(0.4, 1.0, 0.7);
  const Eigen::Vector3d b(0.1, 0.3, 0.2);
  const Polytope P =
    Polytope::box(a + b).pontryaginDifference(Polytope::box(b));
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(P.support(Eigen::Vector3d::Unit(i)), a(i), 1e-9);
    EXPECT_NEAR(P.support(-Eigen::Vector3d::Unit(i)), a(i), 1e-9);
  }
}

TEST(PolytopeTest, PontryaginDifferenceCanBeEmpty)
{
  // Eroding away more than half of a box leaves nothing.
  const Polytope P = Polytope::box(Eigen::VectorXd::Constant(2, 0.1))
    .pontryaginDifference(Polytope::box(Eigen::VectorXd::Constant(2, 0.5)));
  EXPECT_TRUE(P.isEmpty());
  // Sanity: the reverse order stays non-empty.
  const Polytope Q = Polytope::box(Eigen::VectorXd::Constant(2, 0.5))
    .pontryaginDifference(Polytope::box(Eigen::VectorXd::Constant(2, 0.1)));
  EXPECT_FALSE(Q.isEmpty());
}

TEST(PolytopeTest, RedundancyRemovalPreservesTheSet)
{
  const Polytope sq = Polytope::box(Eigen::VectorXd::Ones(2));
  const int n = sq.numHalfspaces();
  Eigen::MatrixXd A(2 * n, 2);
  Eigen::VectorXd b(2 * n);
  for (int i = 0; i < n; ++i) {
    A.row(2 * i) = sq.A().row(i);
    A.row(2 * i + 1) = sq.A().row(i);
    b(2 * i) = sq.b()(i);
    b(2 * i + 1) = sq.b()(i);
  }
  Polytope dup(A, b);
  const int removed = dup.removeRedundantHalfspaces();
  EXPECT_EQ(dup.numHalfspaces(), n);
  EXPECT_EQ(removed, n);

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> unif(-1.5, 1.5);
  for (int i = 0; i < 1000; ++i) {
    Eigen::Vector2d x;
    x << unif(rng), unif(rng);
    EXPECT_EQ(dup.contains(x), sq.contains(x)) << "sample " << i;
  }
}

TEST(ZonotopeTest, SupportFunctionMatchesSampledMaximum)
{
  const Eigen::Vector2d c(0.5, -0.25);
  Eigen::MatrixXd G(2, 3);
  G << 1.0, 0.5, -0.25,
    0.25, -0.75, 0.5;
  const Zonotope Z(c, G);
  const Eigen::Vector2d d = Eigen::Vector2d(0.7, -1.3).normalized();

  // Interior samples never exceed the closed-form support.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(-1.0, 1.0);
  for (int i = 0; i < 10000; ++i) {
    Eigen::Vector3d z;  // generator-box parameter (G is 2 x 3)
    z << unif(rng), unif(rng), unif(rng);
    EXPECT_LE(d.dot(c + G * z), Z.support(d) + 1e-12) << "sample " << i;
  }

  // The maximum over the extreme points (the 2^3 corners of the generator
  // box) is attained exactly — this is the "sampled maximum" that matters.
  double corner_max = -std::numeric_limits<double>::infinity();
  for (int bits = 0; bits < 8; ++bits) {
    Eigen::Vector3d z;
    for (int j = 0; j < 3; ++j) {
      z(j) = ((bits >> j) & 1) ? 1.0 : -1.0;
    }
    corner_max = std::max(corner_max, d.dot(c + G * z));
  }
  EXPECT_NEAR(Z.support(d), corner_max, 1e-9);
}

TEST(ZonotopeTest, OrderReductionOverApproximates)
{
  Eigen::MatrixXd G(3, 5);
  G << 1.0, 0.4, -0.6, 0.2, 0.1,
    0.1, 0.9, 0.3, -0.5, 0.2,
    -0.2, 0.3, 0.8, 0.4, -0.1;
  const Zonotope Z(Eigen::Vector3d::Zero(), G);
  const Zonotope reduced = Z.reduceOrder(3 + 2);  // all generators survive the cut
  ASSERT_LE(reduced.numGenerators(), 5);

  // Every sample of the original must be inside the reduction. An
  // under-approximation here silently breaks safety.
  std::mt19937 rng(2024);
  std::uniform_real_distribution<double> unif(-1.0, 1.0);
  for (int i = 0; i < 2000; ++i) {
    Eigen::VectorXd z(5);
    for (int j = 0; j < 5; ++j) {
      z(j) = unif(rng);
    }
    EXPECT_TRUE(reduced.contains(Z.center() + G * z, 1e-9)) << "sample " << i;
  }
  // The extreme points (box corners of the generator parameter) too.
  for (int bits = 0; bits < 32; ++bits) {
    Eigen::VectorXd z(5);
    for (int j = 0; j < 5; ++j) {
      z(j) = ((bits >> j) & 1) ? 1.0 : -1.0;
    }
    EXPECT_TRUE(reduced.contains(Z.center() + G * z, 1e-9)) << "corner " << bits;
  }
}

// ---------------------------------------------------------------------------
// RPI set
// ---------------------------------------------------------------------------

TEST(RpiTest, ConvergesForSchurStableSystem)
{
  const Eigen::Matrix2d Acl = 0.5 * Eigen::Matrix2d::Identity();
  const Zonotope W = Zonotope::box(Eigen::Vector2d::Constant(0.1));
  const auto rpi = computeRpiSet(Acl, W, 1.0e-3, 100);
  ASSERT_TRUE(rpi.converged);
  EXPECT_LT(rpi.alpha, 1.0);
  EXPECT_GT(rpi.iterations, 0);
  // The analytic mRPI of x^+ = 0.5 x + w, w in box(0.1), is
  // box(0.1 / (1 - 0.5)) = box(0.2); the outer approximation must contain it.
  const Polytope analytic = Polytope::box(Eigen::Vector2d::Constant(0.2));
  const Eigen::MatrixXd V = analytic.vertices();
  for (int i = 0; i < V.cols(); ++i) {
    EXPECT_TRUE(rpi.set.contains(V.col(i), 1.0e-6)) << "vertex " << i;
  }
}

TEST(RpiTest, RejectsUnstableClosedLoop)
{
  Eigen::Matrix2d Acl;
  Acl << 1.1, 0.0,
    0.0, 0.8;  // spectral radius 1.1
  const Zonotope W = Zonotope::box(Eigen::Vector2d::Constant(0.1));
  const auto rpi = computeRpiSet(Acl, W, 1.0e-3, 100);
  EXPECT_FALSE(rpi.converged);
  EXPECT_EQ(rpi.set.numHalfspaces(), 0);
}

TEST(RpiTest, ResultIsRobustPositivelyInvariant)
{
  // Slightly coupled Schur matrix so the invariant set is not axis-aligned.
  Eigen::Matrix2d Acl;
  Acl << 0.5, 0.2,
    0.0, 0.6;
  const Zonotope W = Zonotope::box(Eigen::Vector2d::Constant(0.1));
  const auto rpi = computeRpiSet(Acl, W, 1.0e-4, 100);
  ASSERT_TRUE(rpi.converged);
  EXPECT_TRUE(isRobustPositivelyInvariant(Acl, rpi.set, W.toPolytope(), 1.0e-6));

  // Empirical check: 10000 steps of random w in W, started inside Omega, must
  // never leave it. Catches sign errors the support-function check can miss.
  std::mt19937 rng(0xABCD);
  std::uniform_real_distribution<double> unif(-0.1, 0.1);
  Eigen::Vector2d x = Eigen::Vector2d::Zero();  // the origin is inside Omega
  for (int step = 0; step < 10000; ++step) {
    Eigen::Vector2d w;
    w << unif(rng), unif(rng);
    x = Acl * x + w;
    EXPECT_TRUE(rpi.set.contains(x, 1.0e-9)) << "step " << step;
  }
}

TEST(RpiTest, LqrGainIsStabilising)
{
  const Eigen::MatrixXd A = DoubleIntegrator::A();
  const Eigen::MatrixXd B = DoubleIntegrator::B();
  const Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd K = discreteLqrGain(A, B, Q, R);
  ASSERT_EQ(K.rows(), 2);
  ASSERT_EQ(K.cols(), 4);
  EXPECT_LT(spectralRadius(A + B * K), 1.0);
}

TEST(RpiTest, RpiMatchesPythonReference)
{
  // Parity with codegen/generate_tube_solver.py (compute_offline_sets, which
  // solves the DARE with scipy — deliberately independent of the C++ Riccati
  // iteration, so an agreeing answer is real evidence). Reference supports
  // below were generated for this exact fixture (double integrator, dt=0.1,
  // Q=(10,10,1,1), R=I, W = box(0.005,0.005,0.02,0.02), epsilon=1e-3) with:
  //   python3 codegen/generate_tube_solver.py --print-reference-supports
  const Eigen::MatrixXd A = DoubleIntegrator::A();
  const Eigen::MatrixXd B = DoubleIntegrator::B();
  const Eigen::MatrixXd Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd K = discreteLqrGain(A, B, Q, R);
  const Zonotope W = Zonotope::box(fixtureWHalfWidths());
  const auto rpi = computeRpiSet(A + B * K, W, 1.0e-3, 100);
  ASSERT_TRUE(rpi.converged);
  ASSERT_EQ(rpi.iterations, 49);  // s must agree with the Python run

  const std::vector<std::pair<Eigen::Vector4d, double>> reference = {
    {{1, 0, 0, 0}, 0.11553617254755075},
    {{-1, 0, 0, 0}, 0.11553617254755075},
    {{0, 1, 0, 0}, 0.11553617254755075},
    {{0, -1, 0, 0}, 0.11553617254755075},
    {{0, 0, 1, 0}, 0.16415112690836056},
    {{0, 0, -1, 0}, 0.16415112690836056},
    {{0, 0, 0, 1}, 0.16415112690836056},
    {{0, 0, 0, -1}, 0.16415112690836056},
    {{0.18015906894634776, 0.49545953890636091, 0.84969705107236548, -0.0088020782777398169},
      0.18714357740722548},
    {{-0.6982552846994029, 0.43078588822402841, 0.48029668870099584, -0.31012604972696045},
      0.22866193736621085},
    {{-0.36457002449339276, -0.83302788138098094, -0.17877368824424192, -0.37575685552542076},
      0.12412594170099561},
    {{0.44808552619009323, -0.64462966358144669, 0.36492996321642035, 0.50049783215982091},
      0.19217277598477236},
    {{-0.1665052450815698, 0.30615896939339438, 0.8304918544402381, -0.43454110108103167},
      0.24513654868181936},
    {{-0.19975415789781553, -0.25747666692797733, 0.91421293335277665, -0.24087082613032529},
      0.19946892401143768},
    {{-0.11837034554561809, -0.34454727249130523, -0.70144320358195567, -0.61257903201374442},
      0.19372356190686149},
    {{0.07012443862602287, -0.093044387133264669, 0.95233819203839887, -0.28191713874011615},
      0.19364473223958806},
    {{0.626305942974824, 0.49283450746640595, -0.45846512191667099, 0.39327439025531968},
      0.18874864336717284},
    {{0.45944724880437121, -0.016118173921586067, 0.70227634977904141, -0.54355897433172173},
      0.1850172757477227},
    {{0.81272440217020403, 0.56893602271172672, 0.12366266762358365, -0.022324713083600432},
      0.15675707471253086},
    {{-0.35145456198557401, 0.46326695250273248, -0.12052272376591705, -0.80457298900373475},
      0.20573466576001823},
    {{-0.79298505967533983, 0.016375560139683488, 0.29733582872569725, 0.53150535379976382},
      0.21015928499401451},
    {{-0.26644785913273639, -0.087668531818288264, 0.94494368867636802, 0.16852653240100318},
      0.21002078316049486},
    {{0.71648578641604976, 0.62109040709251173, -0.19050032604766784, -0.25417405425645839},
      0.20147270138494308},
    {{0.44797459829633929, -0.5716240511584173, -0.61932052774162405, -0.29834005318815077},
      0.19778479617817143},
    {{0.13461592593031357, 0.93380606863198523, -0.2815554641268932, 0.17496085074178788},
      0.15812966826192906},
    {{0.42089429490574987, -0.41581674321808737, -0.76382800141004747, 0.2578976790070574},
      0.23802148927564717},
    {{-0.13569877474021202, 0.44989539076353141, -0.77156007071194399, 0.42880652652066148},
      0.1795658193278023},
    {{0.08711317903651275, -0.050754698091927453, -0.60875363564681784, 0.78692710319732351},
      0.23930356874937148},
    {{0.40505400870488861, 0.28302088762345323, -0.66120020247942823, 0.56448624380235868},
      0.22030655112479886},
    {{0.8556185979743911, -0.24014403480549523, -0.24683471444599447, 0.38641982492108057},
      0.20635409530759635},
    {{-0.242985835032801, 0.8007227578149636, 0.015347200243730369, 0.54732569146285315},
      0.11822010927484229},
    {{0.95175373847470435, -0.11868565050744652, -0.19369675271458867, -0.20630100739180104},
      0.15695584364536586},
  };
  for (const auto & entry : reference) {
    EXPECT_NEAR(rpi.zonotope_set.support(entry.first), entry.second, 1.0e-6)
      << "direction " << entry.first.transpose();
  }
}

// ---------------------------------------------------------------------------
// Tube solver
// ---------------------------------------------------------------------------

namespace
{

/// Fixture mirroring config/tube_mpc_params.yaml + mpc_cbf_params.yaml:
/// double integrator, N = 8, dt = 0.1, W = box(0.005, 0.005, 0.02, 0.02),
/// obstacle at (0.5, 0.5) with radius 0.2 (r_eff = 0.2 + 0.1 + 0.05 = 0.35).
///
/// SetUp() must never ASSERT — the acados solver is absent in stub builds, so
/// `initialized_` is false there and every acados-dependent test skips itself.
class TubeMpcTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::printf("[tube_mpc_robustness] RNG seed: 0x%08X\n", kRngSeed);

    mpc_config_.model = ModelType::kDoubleIntegrator2D;
    mpc_config_.horizon = 8;
    mpc_config_.dt = 0.1;
    mpc_config_.Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0);
    mpc_config_.R = Eigen::Vector2d(1.0, 1.0);
    mpc_config_.Qf = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0);
    mpc_config_.x_min = Eigen::Vector4d(-1e9, -1e9, -2.0, -2.0);
    mpc_config_.x_max = Eigen::Vector4d(1e9, 1e9, 2.0, 2.0);
    mpc_config_.u_min = Eigen::Vector2d(-1.0, -1.0);
    mpc_config_.u_max = Eigen::Vector2d(1.0, 1.0);
    mpc_config_.max_sqp_iterations = 20;
    mpc_config_.kkt_tolerance = 1.0e-6;

    cbf_config_.variant = CbfVariant::kFixedDecay;
    cbf_config_.gamma = 0.3;
    cbf_config_.cbf_horizon = 0;  // full horizon
    cbf_config_.ego_radius = 0.1;
    cbf_config_.safety_margin = 0.05;

    tube_config_ = makeTubeConfig(TightenMode::kSupportFunction, 1.0);
    solver_ = std::make_unique<TubeMpcCbfSolver>(mpc_config_, cbf_config_, tube_config_);
    initialized_ = solver_->initialize();
  }

  static TubeConfig makeTubeConfig(TightenMode mode, double lipschitz_h)
  {
    TubeConfig tube;
    tube.disturbance_set = Zonotope::box(fixtureWHalfWidths());
    tube.compute_gain_from_lqr = true;
    tube.lqr_Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0);
    tube.lqr_R = Eigen::Vector2d(1.0, 1.0);
    tube.rpi_epsilon = 1.0e-3;
    tube.rpi_max_iterations = 100;
    tube.tighten_mode = mode;
    tube.lipschitz_h = lipschitz_h;
    return tube;
  }

  static ObstacleState obstacle()
  {
    ObstacleState o;
    o.position = Eigen::Vector3d(0.5, 0.5, 0.0);
    o.velocity = Eigen::Vector3d::Zero();
    o.radius = 0.2;
    o.is_dynamic = false;
    return o;
  }

  /// True-state step: x^+ = A x + B u_applied + w with the exact ZOH model.
  static Eigen::Vector4d advance(
    const Eigen::Vector4d & x, const Eigen::Vector2d & u, const Eigen::Vector4d & w)
  {
    return DoubleIntegrator::A() * x + DoubleIntegrator::B() * u + w;
  }

  /// Adversarial disturbance: the W vertex that pushes x hardest toward the
  /// obstacle — position components along -grad h, velocity components also
  /// accelerating toward it (the tie-break that stresses the tube the most).
  static Eigen::Vector4d worstCaseW(const Eigen::Vector4d & x, const ObstacleState & o)
  {
    const Eigen::Vector2d to_obs =
      Eigen::Vector2d(o.position[0] - x(0), o.position[1] - x(1));
    const Eigen::Vector4d half = fixtureWHalfWidths();
    Eigen::Vector4d w;
    w(0) = (to_obs(0) > 0 ? half(0) : -half(0));
    w(1) = (to_obs(1) > 0 ? half(1) : -half(1));
    w(2) = (to_obs(0) > 0 ? half(2) : -half(2));
    w(3) = (to_obs(1) > 0 ? half(3) : -half(3));
    return w;
  }

  MpcConfig mpc_config_;
  CbfConfig cbf_config_;
  TubeConfig tube_config_;
  std::unique_ptr<TubeMpcCbfSolver> solver_;
  bool initialized_{false};
};

}  // namespace

TEST_F(TubeMpcTest, InitializeRejectsOversizedDisturbance)
{
  // W so large that U (-) K Omega is empty must fail at startup, not surface
  // as a runtime infeasibility. (c_u scales linearly with W: the fixture's
  // ~0.36 becomes ~3.6 with velocity half-width 0.2 > u_max = 1.)
  TubeConfig big = tube_config_;
  big.disturbance_set = Zonotope::box(Eigen::Vector4d(0.05, 0.05, 0.2, 0.2));
  TubeMpcCbfSolver bad(mpc_config_, cbf_config_, big);
  EXPECT_FALSE(bad.initialize());
}

TEST_F(TubeMpcTest, TighteningIsNonNegativeAndShrinksWithW)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  TubeConfig half = tube_config_;
  half.disturbance_set = Zonotope::box(Eigen::Vector4d(0.0025, 0.0025, 0.01, 0.01));
  TubeMpcCbfSolver half_solver(mpc_config_, cbf_config_, half);
  ASSERT_TRUE(half_solver.initialize());

  const ObstacleState o = obstacle();
  std::mt19937 rng(kRngSeed);
  std::uniform_real_distribution<double> unif(-0.5, 1.5);
  for (int trial = 0; trial < 200; ++trial) {
    Eigen::Vector4d z;
    z << unif(rng), unif(rng), 0.0, 0.0;
    const double c_full = solver_->tighteningFor(z, o);
    const double c_half = half_solver.tighteningFor(z, o);
    EXPECT_GE(c_full, -1e-12) << "tightening must never be negative";
    EXPECT_GE(c_half, -1e-12) << "tightening must never be negative";
    EXPECT_LE(c_half, c_full + 1e-9) << "smaller W must not increase the margin";
  }
}

TEST_F(TubeMpcTest, SupportTighteningDominatesLipschitz)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  TubeConfig lips = tube_config_;
  lips.tighten_mode = TightenMode::kLipschitz;
  lips.lipschitz_h = 1.0;  // L_h = 1 covers ||grad h|| = 2 d for d <= 0.5
  TubeMpcCbfSolver lips_solver(mpc_config_, cbf_config_, lips);
  ASSERT_TRUE(lips_solver.initialize());

  const ObstacleState o = obstacle();
  constexpr double kReff = 0.35;  // obstacle + ego radius + safety margin
  for (double dist = kReff; dist <= 0.5 + 1e-9; dist += 0.05) {
    for (double ang = 0.0; ang < 2.0 * kPi - 1e-9; ang += kPi / 6.0) {
      Eigen::Vector4d z;
      z << 0.5 + dist * std::cos(ang), 0.5 + dist * std::sin(ang), 0.0, 0.0;
      const double c_sup = solver_->tighteningFor(z, o);
      const double c_lip = lips_solver.tighteningFor(z, o);
      // Both sound; the support one is less conservative (never more).
      EXPECT_LE(c_sup, c_lip + 1e-9) << "z = " << z.transpose();
    }
  }
}

TEST_F(TubeMpcTest, StaysSafeUnderWorstCaseDisturbance)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  solver_->setReference(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0));  // through the obstacle
  const ObstacleState o = obstacle();
  const std::vector<ObstacleState> obs{o};

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  double min_h = std::numeric_limits<double>::infinity();
  int resets = 0;
  for (int step = 0; step < 200; ++step) {
    const TubeMpcCbfSolution sol = solver_->solve(x, obs);
    ASSERT_TRUE(sol.usable()) << "step " << step
                              << " status " << static_cast<int>(sol.status);
    resets = sol.diagnostics.tube_resets;

    const Eigen::Vector4d w = worstCaseW(x, o);
    const double h = MpcCbfSolver::barrierValue(
      ModelType::kDoubleIntegrator2D, x, o,
      cbf_config_.ego_radius + cbf_config_.safety_margin);
    min_h = std::min(min_h, h);
    x = advance(x, sol.u_applied, w);
  }
  EXPECT_GE(min_h, 0.0) << "worst-case disturbance violated the CBF";
  EXPECT_LE(resets, 2) << "the z_0 policy should not be thrashing";
  std::printf("[tube_mpc_robustness] worst-case sweep: min h = %.6f, resets = %d\n",
    min_h, resets);
}

TEST_F(TubeMpcTest, StaysSafeAcrossRandomDisturbanceSeeds)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  solver_->setReference(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0));
  const ObstacleState o = obstacle();
  const std::vector<ObstacleState> obs{o};
  const Eigen::Vector4d half = fixtureWHalfWidths();

  int violations = 0;
  double min_h_overall = std::numeric_limits<double>::infinity();
  for (int seed = 0; seed < 50; ++seed) {
    solver_->reset();
    std::mt19937 rng(kRngSeed ^ static_cast<std::uint32_t>(seed));
    std::uniform_real_distribution<double> unif(-1.0, 1.0);
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    for (int step = 0; step < 200; ++step) {
      const TubeMpcCbfSolution sol = solver_->solve(x, obs);
      if (!sol.usable()) {
        ++violations;
        break;
      }
      Eigen::Vector4d w;
      w << half(0) * unif(rng), half(1) * unif(rng),
        half(2) * unif(rng), half(3) * unif(rng);
      const double h = MpcCbfSolver::barrierValue(
        ModelType::kDoubleIntegrator2D, x, o,
        cbf_config_.ego_radius + cbf_config_.safety_margin);
      if (h < 0.0) {
        ++violations;
      }
      min_h_overall = std::min(min_h_overall, h);
      x = advance(x, sol.u_applied, w);
    }
  }
  EXPECT_EQ(violations, 0);
  std::printf("[tube_mpc_robustness] random-seed sweep: min h = %.6f\n", min_h_overall);
}

TEST_F(TubeMpcTest, NominalCbfViolatesUnderSameDisturbance)
{
  // The A8 ablation: the un-tightened CBF under the same worst-case
  // disturbance must actually violate — otherwise W is too small to prove
  // anything. kNone is gated behind allow_unsafe_ablation.
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  TubeConfig ablation = tube_config_;
  ablation.tighten_mode = TightenMode::kNone;
  ablation.allow_unsafe_ablation = true;
  TubeMpcCbfSolver nominal(mpc_config_, cbf_config_, ablation);
  ASSERT_TRUE(nominal.initialize());
  nominal.setReference(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0));
  const ObstacleState o = obstacle();
  const std::vector<ObstacleState> obs{o};

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  double min_h_nominal = std::numeric_limits<double>::infinity();
  for (int step = 0; step < 200; ++step) {
    const TubeMpcCbfSolution sol = nominal.solve(x, obs);
    ASSERT_TRUE(sol.usable()) << "step " << step;
    x = advance(x, sol.u_applied, worstCaseW(x, o));
    const double h = MpcCbfSolver::barrierValue(
      ModelType::kDoubleIntegrator2D, x, o,
      cbf_config_.ego_radius + cbf_config_.safety_margin);
    min_h_nominal = std::min(min_h_nominal, h);
  }
  EXPECT_LT(min_h_nominal, 0.0)
    << "the ablation stayed safe — the disturbance is too small to be evidence";
  std::printf("[tube_mpc_robustness] ablation: min h = %.6f\n", min_h_nominal);
}

TEST_F(TubeMpcTest, ErrorStaysInsideRpiSet)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  solver_->setReference(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0));
  const ObstacleState o = obstacle();
  const std::vector<ObstacleState> obs{o};
  const Eigen::Vector4d half = fixtureWHalfWidths();
  std::mt19937 rng(kRngSeed);
  std::uniform_real_distribution<double> unif(-1.0, 1.0);

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  for (int step = 0; step < 200; ++step) {
    const TubeMpcCbfSolution sol = solver_->solve(x, obs);
    ASSERT_TRUE(sol.usable()) << "step " << step;
    Eigen::Vector4d w;
    w << half(0) * unif(rng), half(1) * unif(rng),
      half(2) * unif(rng), half(3) * unif(rng);
    // e_{k+1} = x_{k+1} - z_{k+1}; the certificate says this is always in
    // Omega (the z_0 policy keeps e_0 there and Omega is RPI).
    const Eigen::Vector4d e = advance(x, sol.u_applied, w) - sol.z_pred.col(1);
    EXPECT_TRUE(solver_->rpiSet().contains(e, 1e-9)) << "step " << step
                                                     << " e = " << e.transpose();
    x = advance(x, sol.u_applied, w);
  }
}

TEST_F(TubeMpcTest, AncillaryInputRespectsBounds)
{
  if (!initialized_) {
    GTEST_SKIP() << "acados unavailable";
  }
  solver_->setReference(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0));
  const ObstacleState o = obstacle();
  const std::vector<ObstacleState> obs{o};
  const Eigen::Vector4d half = fixtureWHalfWidths();
  std::mt19937 rng(kRngSeed);
  std::uniform_real_distribution<double> unif(-1.0, 1.0);

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  int clip_total = 0;
  for (int step = 0; step < 200; ++step) {
    const TubeMpcCbfSolution sol = solver_->solve(x, obs);
    ASSERT_TRUE(sol.usable()) << "step " << step;
    clip_total += sol.diagnostics.clip_count;
    for (int i = 0; i < 2; ++i) {
      EXPECT_GE(sol.u_applied(i), mpc_config_.u_min[i] - 1e-9);
      EXPECT_LE(sol.u_applied(i), mpc_config_.u_max[i] + 1e-9);
    }
    Eigen::Vector4d w;
    w << half(0) * unif(rng), half(1) * unif(rng),
      half(2) * unif(rng), half(3) * unif(rng);
    x = advance(x, sol.u_applied, w);
  }
  // The clip must never trigger: with a correct tightening, v_0 + K e_0 stays
  // inside U by construction.
  EXPECT_EQ(clip_total, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
