// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS 2 wrapper around MpcCbfSolver / TubeMpcCbfSolver. See .deepseek/09_NODE.md.

#include "mpc_cbf_unified/mpc_cbf_node.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mpc_cbf_unified
{
namespace
{

/// Parse the infeasible_policy parameter. Returns false on an unknown name.
bool parseInfeasiblePolicy(const std::string & name, InfeasiblePolicy & out)
{
  if (name == "hold_last") {
    out = InfeasiblePolicy::kHoldLast;
    return true;
  }
  if (name == "zero") {
    out = InfeasiblePolicy::kZero;
    return true;
  }
  if (name == "brake") {
    out = InfeasiblePolicy::kBrake;
    return true;
  }
  if (name == "previous_horizon") {
    out = InfeasiblePolicy::kPreviousHorizon;
    return true;
  }
  return false;
}

const char * toString(InfeasiblePolicy policy)
{
  switch (policy) {
    case InfeasiblePolicy::kHoldLast:
      return "hold_last";
    case InfeasiblePolicy::kZero:
      return "zero";
    case InfeasiblePolicy::kBrake:
      return "brake";
    case InfeasiblePolicy::kPreviousHorizon:
      return "previous_horizon";
  }
  return "unknown";
}

/// YAML double array -> Eigen vector.
Eigen::VectorXd toVectorXd(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

}  // namespace

MpcCbfNode::MpcCbfNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mpc_cbf_node", options)
{
  // §9.1: constructor order is fixed — declare, load, set up the solver, then
  // the interfaces. A failure in loadParameters or setupSolver is fatal: a
  // controller that starts with a silently-substituted configuration is worse
  // than one that refuses to start.
  declareParameters();
  if (!loadParameters()) {
    RCLCPP_FATAL(get_logger(), "invalid configuration (see messages above); shutting down");
    rclcpp::shutdown();
    return;
  }
  if (!setupSolver()) {
    RCLCPP_FATAL(get_logger(), "solver initialisation failed (see messages above); shutting down");
    rclcpp::shutdown();
    return;
  }
  setupInterfaces();
  RCLCPP_INFO(get_logger(), "mpc_cbf_node ready: model=%s N=%d dt=%.3f tube=%s policy=%s",
    toString(mpc_config_.model), mpc_config_.horizon, mpc_config_.dt,
    use_tube_mpc_ ? "on" : "off", toString(infeasible_policy_));
}

MpcCbfNode::~MpcCbfNode() = default;

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void MpcCbfNode::declareParameters()
{
  // The names and defaults below are the contract between config/, launch/ and
  // this function — they must match the YAML files byte-for-byte (§9.2).
  // Read-only parameters need a different generated solver (or an initialize()
  // that must not run in the control loop); changing them at runtime is
  // rejected by onSetParameters().
  using rcl_interfaces::msg::FloatingPointRange;
  using rcl_interfaces::msg::IntegerRange;
  using rcl_interfaces::msg::ParameterDescriptor;

  auto declare_double = [this](
    const std::string & name, double def, const std::string & description,
    double lo, double hi, bool read_only) {
      ParameterDescriptor d;
      d.description = description;
      d.read_only = read_only;
      FloatingPointRange range;
      range.from_value = lo;
      range.to_value = hi;
      d.floating_point_range.push_back(range);
      this->declare_parameter(name, def, d);
    };
  auto declare_int = [this](
    const std::string & name, int def, const std::string & description,
    int lo, int hi, bool read_only) {
      ParameterDescriptor d;
      d.description = description;
      d.read_only = read_only;
      IntegerRange range;
      range.from_value = lo;
      range.to_value = hi;
      d.integer_range.push_back(range);
      this->declare_parameter(name, def, d);
    };
  auto declare_str = [this](
    const std::string & name, const std::string & def,
    const std::string & description, bool read_only) {
      ParameterDescriptor d;
      d.description = description;
      d.read_only = read_only;
      this->declare_parameter(name, def, d);
    };
  auto declare_arr = [this](
    const std::string & name, const std::vector<double> & def,
    const std::string & description, bool read_only) {
      ParameterDescriptor d;
      d.description = description;
      d.read_only = read_only;
      this->declare_parameter(name, def, d);
    };
  auto declare_bool = [this](
    const std::string & name, bool def, const std::string & description, bool read_only) {
      ParameterDescriptor d;
      d.description = description;
      d.read_only = read_only;
      this->declare_parameter(name, def, d);
    };

  // --- plant / discretisation (read-only: selects the generated solver) ----
  declare_str("model", "double_integrator_2d",
    "Model registry key; selects the generated solver.", true);
  declare_int("horizon", 8, "Prediction horizon N.", 1, 1000, true);
  declare_double("dt", 0.1,
    "Discretisation step [s]; a property of the generated solver.", 1.0e-4, 1.0e3, true);
  declare_double("control_rate_hz", 10.0,
    "Control loop rate; must equal 1/dt unless you know why it should not.", 1.0e-3, 1.0e6, false);

  // --- cost (live-changeable, validated against nx/nu) ----------------------
  declare_arr("Q", {10.0, 10.0, 1.0, 1.0}, "Stage state weights, diagonal, size nx.", false);
  declare_arr("R", {1.0, 1.0}, "Stage input weights, diagonal, size nu.", false);
  declare_arr("Qf", {100.0, 100.0, 10.0, 10.0}, "Terminal state weights, diagonal, size nx.",
      false);

  // --- bounds --------------------------------------------------------------
  declare_arr("x_min", {-1.0e9, -1.0e9, -2.0, -2.0}, "State lower bounds (use +/-1e9, not inf).",
      false);
  declare_arr("x_max", {1.0e9, 1.0e9, 2.0, 2.0}, "State upper bounds (use +/-1e9, not inf).",
      false);
  declare_arr("u_min", {-1.0, -1.0}, "Input lower bounds.", false);
  declare_arr("u_max", {1.0, 1.0}, "Input upper bounds.", false);

  // --- CBF -----------------------------------------------------------------
  declare_str("cbf_variant", "fixed_decay",
    "fixed_decay | relaxed_decay | distance_only; selects the generated solver.", true);
  declare_double("gamma", 0.3, "Class-K gain; (0, 1]. Live-changeable via setGamma.", 1.0e-6, 1.0,
      false);
  declare_int("cbf_horizon", 0, "0 => use `horizon`; a smaller value is the paper's N_CBF.", 0,
      1000, false);
  declare_double("ego_radius", 0.15, "Ego radius [m], inflates every obstacle.", 0.0, 1.0e3, false);
  declare_double("safety_margin", 0.05, "Extra inflation [m] on top of ego_radius.", 0.0, 1.0e3,
      false);

  // relaxed_decay only; safety requires omega_max * gamma <= 1.
  declare_double("omega_min", 0.0, "Lower bound on omega_k (relaxed_decay only).", 0.0, 1.0e3,
      false);
  declare_double("omega_max", 3.0, "Upper bound; safety needs omega_max * gamma <= 1.", 0.0, 1.0e3,
      false);
  declare_double("omega_weight", 1000.0, "Penalty on (omega_k - 1)^2.", 0.0, 1.0e9, false);
  declare_bool("omega_decay_per_step", false, "gamma^k profile from CDC 2021 III.C.", false);

  // --- solver --------------------------------------------------------------
  declare_int("max_sqp_iterations", 20,
    "SQP iteration cap (1 => RTI); read at initialize().", 1, 1000, false);
  declare_double("kkt_tolerance", 1.0e-6, "Solver residual tolerance.", 1.0e-12, 1.0, false);
  declare_bool("use_rti", false, "Real-time-iteration scheme; read at initialize().", false);

  // --- runtime behaviour ---------------------------------------------------
  declare_str("infeasible_policy", "previous_horizon",
    "hold_last | zero | brake | previous_horizon (see §9.4).", false);
  declare_int("max_consecutive_failures", 5,
    "Failures before the diagnostic level goes ERROR.", 1, 100000, false);
  declare_double("odom_timeout_s", 0.5, "Odometry age before the fallback input is used.", 1.0e-3,
      1.0e3, false);
  declare_str("frame_id", "map", "Frame id for published messages.", false);
  declare_bool("use_tube_mpc", false,
    "true switches to TubeMpcCbfSolver (see config/tube_mpc_params.yaml).", true);

  // --- topics (remap-friendly defaults) ------------------------------------
  declare_str("odom_topic", "~/odom", "Odometry subscription.", false);
  declare_str("goal_topic", "~/goal", "Goal subscription.", false);
  declare_str("obstacle_topic", "~/obstacles", "MarkerArray subscription.", false);
  declare_str("cmd_topic", "~/cmd_vel", "TwistStamped command publisher.", false);
  declare_str("predicted_path_topic", "~/predicted_path", "Predicted path publisher.", false);
  declare_str("cbf_values_topic", "~/cbf_values", "Barrier values publisher.", false);

  // --- tube-MPC-CBF (read-only; all baked into initialize()) ---------------
  declare_int("tube.disturbance_set.dimension", 4, "W dimension (== nx).", 1, 100, true);
  declare_arr("tube.disturbance_set.center", {0.0, 0.0, 0.0, 0.0}, "W centre.", true);
  declare_arr("tube.disturbance_set.generators",
    {0.005, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.02},
    "W generators, row-major (dimension rows).", true);
  declare_bool("tube.compute_gain_from_lqr", true, "Overwrite tube.K with the LQR gain.", true);
  declare_arr("tube.lqr_Q", {10.0, 10.0, 1.0, 1.0}, "LQR state weights.", true);
  declare_arr("tube.lqr_R", {1.0, 1.0}, "LQR input weights.", true);
  declare_arr("tube.K", {}, "Ancillary gain, row-major nu x nx; A + B K must be Schur.", true);
  declare_double("tube.rpi_epsilon", 1.0e-3, "mRPI outer-approximation accuracy.", 1.0e-12, 1.0,
      true);
  declare_int("tube.rpi_max_iterations", 100, "RPI iteration cap.", 1, 100000, true);
  declare_int("tube.rpi_max_generators", 64, "Zonotope order-reduction cap.", 1, 100000, true);
  declare_str("tube.tighten_mode", "support_function",
    "support_function | lipschitz | none (`none` is the unsafe ablation).", true);
  declare_double("tube.lipschitz_h", 2.0, "Lipschitz constant, tighten_mode == lipschitz only.",
      0.0, 1.0e9, true);
  declare_double("tube.relinearisation_threshold", 0.0,
    "0 disables Omega re-computation; otherwise the drift [m] that triggers it.", 0.0, 1.0e9, true);
}

bool MpcCbfNode::loadParameters()
{
  const std::string model_name = get_parameter("model").as_string();
  if (!parseModelType(model_name, mpc_config_.model)) {
    RCLCPP_FATAL(get_logger(), "loadParameters: unknown model '%s'", model_name.c_str());
    return false;
  }
  const std::string variant_name = get_parameter("cbf_variant").as_string();
  if (!parseCbfVariant(variant_name, cbf_config_.variant)) {
    RCLCPP_FATAL(get_logger(), "loadParameters: unknown cbf_variant '%s'", variant_name.c_str());
    return false;
  }

  mpc_config_.horizon = static_cast<int>(get_parameter("horizon").as_int());
  mpc_config_.dt = get_parameter("dt").as_double();
  mpc_config_.Q = toVectorXd(get_parameter("Q").as_double_array());
  mpc_config_.R = toVectorXd(get_parameter("R").as_double_array());
  mpc_config_.Qf = toVectorXd(get_parameter("Qf").as_double_array());
  mpc_config_.x_min = toVectorXd(get_parameter("x_min").as_double_array());
  mpc_config_.x_max = toVectorXd(get_parameter("x_max").as_double_array());
  mpc_config_.u_min = toVectorXd(get_parameter("u_min").as_double_array());
  mpc_config_.u_max = toVectorXd(get_parameter("u_max").as_double_array());
  mpc_config_.max_sqp_iterations = static_cast<int>(get_parameter("max_sqp_iterations").as_int());
  mpc_config_.kkt_tolerance = get_parameter("kkt_tolerance").as_double();
  mpc_config_.use_rti = get_parameter("use_rti").as_bool();

  cbf_config_.gamma = get_parameter("gamma").as_double();
  cbf_config_.cbf_horizon = static_cast<int>(get_parameter("cbf_horizon").as_int());
  cbf_config_.ego_radius = get_parameter("ego_radius").as_double();
  cbf_config_.safety_margin = get_parameter("safety_margin").as_double();
  cbf_config_.omega_min = get_parameter("omega_min").as_double();
  cbf_config_.omega_max = get_parameter("omega_max").as_double();
  cbf_config_.omega_weight = get_parameter("omega_weight").as_double();
  cbf_config_.omega_decay_per_step = get_parameter("omega_decay_per_step").as_bool();

  use_tube_mpc_ = get_parameter("use_tube_mpc").as_bool();

  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  max_consecutive_failures_ = static_cast<int>(get_parameter("max_consecutive_failures").as_int());
  odom_timeout_s_ = get_parameter("odom_timeout_s").as_double();
  frame_id_ = get_parameter("frame_id").as_string();

  const std::string policy_name = get_parameter("infeasible_policy").as_string();
  if (!parseInfeasiblePolicy(policy_name, infeasible_policy_)) {
    RCLCPP_FATAL(get_logger(), "loadParameters: unknown infeasible_policy '%s'",
        policy_name.c_str());
    return false;
  }

  odom_topic_ = get_parameter("odom_topic").as_string();
  goal_topic_ = get_parameter("goal_topic").as_string();
  obstacle_topic_ = get_parameter("obstacle_topic").as_string();
  cmd_topic_ = get_parameter("cmd_topic").as_string();
  predicted_path_topic_ = get_parameter("predicted_path_topic").as_string();
  cbf_values_topic_ = get_parameter("cbf_values_topic").as_string();

  if (use_tube_mpc_) {
    const std::string tm_name = get_parameter("tube.tighten_mode").as_string();
    if (!parseTightenMode(tm_name, tube_config_.tighten_mode)) {
      RCLCPP_FATAL(get_logger(), "loadParameters: unknown tube.tighten_mode '%s'", tm_name.c_str());
      return false;
    }
    const int dim = static_cast<int>(get_parameter("tube.disturbance_set.dimension").as_int());
    const std::vector<double> center =
      get_parameter("tube.disturbance_set.center").as_double_array();
    const std::vector<double> gens =
      get_parameter("tube.disturbance_set.generators").as_double_array();
    if (dim < 1 || static_cast<int>(center.size()) != dim ||
      static_cast<int>(gens.size()) != dim * dim)
    {
      RCLCPP_FATAL(get_logger(),
        "loadParameters: tube.disturbance_set dimension mismatch "
        "(dimension=%d, center=%zu, generators=%zu)", dim, center.size(), gens.size());
      return false;
    }
    Eigen::MatrixXd G(dim, dim);
    for (int r = 0; r < dim; ++r) {
      for (int c = 0; c < dim; ++c) {
        G(r, c) = gens[static_cast<size_t>(r * dim + c)];
      }
    }
    tube_config_.disturbance_set = Zonotope(toVectorXd(center), G);

    tube_config_.compute_gain_from_lqr = get_parameter("tube.compute_gain_from_lqr").as_bool();
    tube_config_.lqr_Q = toVectorXd(get_parameter("tube.lqr_Q").as_double_array());
    tube_config_.lqr_R = toVectorXd(get_parameter("tube.lqr_R").as_double_array());
    const std::vector<double> k_flat = get_parameter("tube.K").as_double_array();
    if (!k_flat.empty()) {
      const int nx = MpcCbfSolver::stateDimOf(mpc_config_.model);
      const int nu = MpcCbfSolver::inputDimOf(mpc_config_.model);
      if (static_cast<int>(k_flat.size()) != nu * nx) {
        RCLCPP_FATAL(get_logger(),
          "loadParameters: tube.K has %zu entries, expected nu*nx = %d",
          k_flat.size(), nu * nx);
        return false;
      }
      tube_config_.K = Eigen::MatrixXd(nu, nx);
      for (int r = 0; r < nu; ++r) {
        for (int c = 0; c < nx; ++c) {
          tube_config_.K(r, c) = k_flat[static_cast<size_t>(r * nx + c)];
        }
      }
    }
    tube_config_.rpi_epsilon = get_parameter("tube.rpi_epsilon").as_double();
    tube_config_.rpi_max_iterations =
      static_cast<int>(get_parameter("tube.rpi_max_iterations").as_int());
    tube_config_.rpi_max_generators =
      static_cast<int>(get_parameter("tube.rpi_max_generators").as_int());
    tube_config_.lipschitz_h = get_parameter("tube.lipschitz_h").as_double();
    tube_config_.relinearisation_threshold =
      get_parameter("tube.relinearisation_threshold").as_double();
  }

  // Validate before accepting (§9.2). The solver re-validates in initialize().
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&MpcCbfNode::onSetParameters, this, std::placeholders::_1));
  return true;
}

rcl_interfaces::msg::SetParametersResult MpcCbfNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const int nx = MpcCbfSolver::stateDimOf(mpc_config_.model);
  const int nu = MpcCbfSolver::inputDimOf(mpc_config_.model);

  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "gamma") {
      const double g = p.as_double();
      if (!(g > 0.0) || g > 1.0) {
        result.successful = false;
        result.reason = "gamma must lie in (0, 1]";
        return result;
      }
      cbf_config_.gamma = g;
      if (solver_) {
        solver_->setGamma(g);  // applied live; the generated model holds gamma as a parameter
      }
      // TubeMpcCbfSolver has no runtime gamma setter; the value is accepted
      // into the config and applied on the next initialize().
    } else if (name == "Q" || name == "R" || name == "Qf") {
      const std::vector<double> v = p.as_double_array();
      const int expected = (name == "R") ? nu : nx;
      bool bad = static_cast<int>(v.size()) != expected;
      for (double w : v) {
        if (!std::isfinite(w) || w < 0.0) {
          bad = true;
        }
      }
      if (bad) {
        result.successful = false;
        result.reason = name + " must have " + std::to_string(expected) +
          " non-negative finite entries (the generated cost is fixed at codegen; "
          "the value applies on the next solver initialisation)";
        return result;
      }
      if (name == "Q") {
        mpc_config_.Q = toVectorXd(v);
      } else if (name == "R") {
        mpc_config_.R = toVectorXd(v);
      } else {
        mpc_config_.Qf = toVectorXd(v);
      }
    } else if (name == "safety_margin" || name == "ego_radius") {
      const double x = p.as_double();
      if (!std::isfinite(x) || x < 0.0) {
        result.successful = false;
        result.reason = name + " must be a non-negative finite number";
        return result;
      }
      if (name == "safety_margin") {
        cbf_config_.safety_margin = x;
      } else {
        cbf_config_.ego_radius = x;
      }
    } else if (name == "infeasible_policy") {
      InfeasiblePolicy policy;
      if (!parseInfeasiblePolicy(p.as_string(), policy)) {
        result.successful = false;
        result.reason =
          "infeasible_policy must be one of hold_last | zero | brake | previous_horizon";
        return result;
      }
      infeasible_policy_ = policy;
      RCLCPP_INFO(get_logger(), "infeasible_policy -> %s", toString(policy));
    } else {
      result.successful = false;
      result.reason = "parameter '" + name +
        "' is not changeable at runtime (it selects the generated solver or is "
        "fixed at startup)";
      return result;
    }
  }
  return result;
}

bool MpcCbfNode::setupSolver()
{
  if (use_tube_mpc_) {
    tube_solver_ = std::make_unique<TubeMpcCbfSolver>(mpc_config_, cbf_config_, tube_config_);
    if (!tube_solver_->initialize()) {
      RCLCPP_FATAL(get_logger(), "setupSolver: TubeMpcCbfSolver::initialize() failed");
      return false;
    }
    RCLCPP_INFO(get_logger(), "tube-MPC-CBF initialised: tighten=%s W=%d generators",
      toString(tube_config_.tighten_mode), tube_config_.disturbance_set.numGenerators());
  } else {
    solver_ = std::make_unique<MpcCbfSolver>(mpc_config_, cbf_config_);
    if (!solver_->initialize()) {
      RCLCPP_FATAL(get_logger(),
        "setupSolver: MpcCbfSolver::initialize() failed (built without acados? "
        "set ACADOS_SOURCE_DIR and rebuild — the stub refuses to run a "
        "controller with a silently-substituted solver)");
      return false;
    }
    RCLCPP_INFO(get_logger(), "MPC-CBF initialised: model=%s N=%d variant=%s",
      toString(mpc_config_.model), mpc_config_.horizon, toString(cbf_config_.variant));
  }
  return true;
}

void MpcCbfNode::setupInterfaces()
{
  // §9.3: sensor inputs at SensorDataQoS (depth-5 best effort — a dropped
  // odometry message is cheaper than head-of-line blocking behind a slow
  // republisher); the goal is a low-rate command, reliable is fine.
  auto sensor_qos = rclcpp::SensorDataQoS();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, sensor_qos, std::bind(&MpcCbfNode::odomCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&MpcCbfNode::goalCallback, this, std::placeholders::_1));
  obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    obstacle_topic_, sensor_qos,
    std::bind(&MpcCbfNode::obstacleCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(predicted_path_topic_, 10);
  cbf_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cbf_values_topic_, 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // The control timer runs in its own MutuallyExclusive callback group so a
  // slow solve cannot re-enter itself: ROS 2 timers in the same group never
  // overlap, and neither solver is re-entrant (§9.3).
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_hz_),
    std::bind(&MpcCbfNode::controlLoop, this), control_group_);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void MpcCbfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // The conversion (rotation of the body-frame twist, per-model layout) is the
  // pure function stateFromOdometry; the callback only copies + stamps.
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = stateFromOdometry(*msg);
  last_odom_stamp_ = msg->header.stamp;
}

void MpcCbfNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const int nx = MpcCbfSolver::stateDimOf(mpc_config_.model);
  goal_state_ = Eigen::VectorXd::Zero(nx);
  goal_state_(0) = msg->pose.position.x;
  goal_state_(1) = msg->pose.position.y;
  if (mpc_config_.model == ModelType::kQuadrotorPlanar) {
    // The planar quad flies in (px, pz); z comes from the pose, not y (§4.2).
    goal_state_(1) = msg->pose.position.z;
  }
  if (mpc_config_.model == ModelType::kUnicycle2D ||
    mpc_config_.model == ModelType::kBicycleKinematic)
  {
    goal_state_(2) = tf2::getYaw(msg->pose.orientation);
  } else if (mpc_config_.model == ModelType::kQuadrotorPlanar) {
    goal_state_(4) = 2.0 * std::atan2(msg->pose.orientation.y, msg->pose.orientation.w);
  }
  if (solver_) {
    solver_->setReference(goal_state_);
  }
  RCLCPP_INFO(get_logger(), "goal -> [% .2f, % .2f, ...]", goal_state_(0), goal_state_(1));
}

void MpcCbfNode::obstacleCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  const std::vector<ObstacleState> obs = obstaclesFromMarkers(*msg);
  std::lock_guard<std::mutex> lock(state_mutex_);
  obstacles_ = obs;
}

void MpcCbfNode::controlLoop()
{
  // 1. Copy the shared state under the lock, then release before solving: the
  // solve can take longer than the subscription period and must not block
  // odometry callbacks (they only fill the copy next time).
  Eigen::VectorXd x0;
  std::vector<ObstacleState> obstacles;
  rclcpp::Time odom_stamp;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    x0 = latest_state_;
    odom_stamp = last_odom_stamp_;
    obstacles = obstacles_;
  }

  const int nu = MpcCbfSolver::inputDimOf(mpc_config_.model);

  // 2. Stale check. No message yet => last_odom_stamp_ is epoch 0, so the age
  // is huge and this branch covers both cases.
  const rclcpp::Time now_t = now();
  const double age = (now_t - odom_stamp).seconds();
  if (x0.size() == 0 || age > odom_timeout_s_) {
    const Eigen::VectorXd u_fb =
      (last_input_.size() == nu) ? last_input_ : Eigen::VectorXd::Zero(nu);
    cmd_pub_->publish(twistFromInput(u_fb));
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now_t;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.name = "mpc_cbf";
    st.message = "odometry stale (" + std::to_string(age) + " s)";
    arr.status.push_back(st);
    diagnostics_pub_->publish(arr);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "odometry stale (%.1f s); holding last input", age);
    return;
  }

  SolverStatus status = SolverStatus::kNotInitialized;
  SolverDiagnostics diag;
  Eigen::VectorXd u(nu);
  Eigen::MatrixXd x_pred;
  Eigen::MatrixXd u_pred;

  if (use_tube_mpc_) {
    // 3. Solve (the tube solver applies the ancillary term itself).
    const TubeMpcCbfSolution sol = tube_solver_->solve(x0, obstacles);
    status = sol.status;
    diag = sol.diagnostics;
    if (!sol.usable()) {
      handleSolverFailure(status, diag);
      return;
    }
    u = sol.u_applied;
    x_pred = sol.z_pred;
    u_pred = sol.v_pred;
    // The robust (tightened) barrier values are what the controller actually
    // guarantees; publish those instead of the nominal ones.
    diag.cbf_values = sol.robust_cbf_values;
  } else {
    const MpcCbfSolution sol = solver_->solve(x0, obstacles);
    status = sol.status;
    diag = sol.diagnostics;
    if (!sol.usable()) {
      handleSolverFailure(status, diag);
      return;
    }
    u = sol.u0;
    x_pred = sol.x_pred;
    u_pred = sol.u_pred;
  }

  // 4. Success path.
  consecutive_failures_ = 0;
  fallback_index_ = 0;
  last_input_ = u;
  last_u_pred_ = u_pred;  // the previous_horizon fallback walks these columns
  cmd_pub_->publish(twistFromInput(u));
  publishPredictedPath(x_pred);
  publishCbfValues(diag);
  publishDiagnostics(status, diag);
}

void MpcCbfNode::handleSolverFailure(
  SolverStatus status, const SolverDiagnostics & diagnostics)
{
  ++consecutive_failures_;
  const int nu = MpcCbfSolver::inputDimOf(mpc_config_.model);

  // Apply the configured fallback policy (§9.4). None of the four is a
  // guarantee — previous_horizon is the only one with any relationship to the
  // constraints that were satisfied when the plan was computed.
  Eigen::VectorXd u_fb;
  switch (infeasible_policy_) {
    case InfeasiblePolicy::kHoldLast:
      u_fb = (last_input_.size() == nu) ? last_input_ : Eigen::VectorXd::Zero(nu);
      break;
    case InfeasiblePolicy::kZero:
      u_fb = Eigen::VectorXd::Zero(nu);
      break;
    case InfeasiblePolicy::kBrake:
      // Maximum-deceleration input admissible under u_min/u_max. For a double
      // integrator this is u_min (full braking); for rotational axes it is the
      // maximum opposing moment. Deterministic and always feasible.
      u_fb = mpc_config_.u_min;
      break;
    case InfeasiblePolicy::kPreviousHorizon:
      if (fallback_index_ < static_cast<std::size_t>(last_u_pred_.cols())) {
        u_fb = last_u_pred_.col(static_cast<Eigen::Index>(fallback_index_));
        ++fallback_index_;
      } else {
        u_fb = mpc_config_.u_min;  // exhausted the plan: brake
      }
      break;
  }
  cmd_pub_->publish(twistFromInput(u_fb));

  // Diagnostics: WARN on the first failure, ERROR from max_consecutive_failures
  // onward. Carries the infeasibility_reason and the first active CBF step so
  // a monitor can tell "transient numeric hiccup" from "the plan violates the
  // barrier at step 3". Never throws, never shuts the node down.
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = (consecutive_failures_ >= static_cast<std::size_t>(max_consecutive_failures_)) ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.name = "mpc_cbf";
  st.message = "solver returned " + std::string(toString(status)) + " (" +
    std::to_string(consecutive_failures_) + " consecutive)";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "infeasibility_reason";
  kv.value = diagnostics.infeasibility_reason;
  st.values.push_back(kv);
  kv.key = "first_active_cbf_step";
  kv.value = std::to_string(diagnostics.first_active_cbf_step);
  st.values.push_back(kv);
  kv.key = "first_active_obstacle";
  kv.value = std::to_string(diagnostics.first_active_obstacle);
  st.values.push_back(kv);
  kv.key = "consecutive_failures";
  kv.value = std::to_string(consecutive_failures_);
  st.values.push_back(kv);
  arr.status.push_back(st);
  diagnostics_pub_->publish(arr);

  if (consecutive_failures_ == 1) {
    RCLCPP_WARN(get_logger(), "solver failed (%s): %s",
      toString(status), diagnostics.infeasibility_reason.c_str());
  } else if (consecutive_failures_ >= static_cast<std::size_t>(max_consecutive_failures_)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "solver failing for %zu consecutive cycles (%s); policy=%s",
      consecutive_failures_, toString(status), toString(infeasible_policy_));
  }
}

// ---------------------------------------------------------------------------
// Conversions
// ---------------------------------------------------------------------------

Eigen::VectorXd MpcCbfNode::stateFromOdometry(const nav_msgs::msg::Odometry & msg) const
{
  const double px = msg.pose.pose.position.x;
  const double py = msg.pose.pose.position.y;
  const double pz = msg.pose.pose.position.z;
  const double vx_b = msg.twist.twist.linear.x;
  const double vy_b = msg.twist.twist.linear.y;
  const double vz_b = msg.twist.twist.linear.z;

  switch (mpc_config_.model) {
    case ModelType::kDoubleIntegrator2D: {
      // nav_msgs twist is BODY-frame; the double-integrator plant lives in the
      // odom frame, so rotate the twist by the pose yaw. Written explicitly
      // (no tf2::transform) so the sign convention is reviewable:
      //   v_world = R(yaw) * v_body, R = [c -s; s  c].
        const double yaw = tf2::getYaw(msg.pose.pose.orientation);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double vx_w = c * vx_b - s * vy_b;
        const double vy_w = s * vx_b + c * vy_b;
        Eigen::VectorXd x(4);
        x << px, py, vx_w, vy_w;
        return x;
      }
    case ModelType::kUnicycle2D: {
        Eigen::VectorXd x(3);
        x << px, py, tf2::getYaw(msg.pose.pose.orientation);
        return x;
      }
    case ModelType::kBicycleKinematic: {
      // v is the forward speed, signed by the body-frame x component so a
      // reversing robot has v < 0.
        const double v = std::copysign(std::hypot(vx_b, vy_b), vx_b);
        Eigen::VectorXd x(4);
        x << px, py, tf2::getYaw(msg.pose.pose.orientation), v;
        return x;
      }
    case ModelType::kQuadrotorPlanar: {
      // State layout [px, pz, vx, vz, pitch, pitch_rate]; position axis is
      // (px, pz). The twist is body-frame; pitch is about the y-axis, so
      //   v_world = R(pitch) * v_body, R = [c  s; -s  c].
        const double pitch = 2.0 * std::atan2(
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.w);
        const double c = std::cos(pitch);
        const double s = std::sin(pitch);
        const double vx_w = c * vx_b + s * vz_b;
        const double vz_w = -s * vx_b + c * vz_b;
        Eigen::VectorXd x(6);
        x << px, pz, vx_w, vz_w, pitch, msg.twist.twist.angular.y;
        return x;
      }
  }
  return Eigen::VectorXd();
}

std::vector<ObstacleState> MpcCbfNode::obstaclesFromMarkers(
  const visualization_msgs::msg::MarkerArray & msg) const
{
  std::vector<ObstacleState> out;
  std::size_t ignored = 0;
  for (const auto & m : msg.markers) {
    // SPHERE/CYLINDER only. scale.x is the diameter; the obstacle radius is
    // scale.x / 2 (the solver adds ego_radius + safety_margin on top).
    if (m.type != visualization_msgs::msg::Marker::SPHERE &&
      m.type != visualization_msgs::msg::Marker::CYLINDER)
    {
      ++ignored;
      continue;
    }
    ObstacleState o;
    o.position.x() = m.pose.position.x;
    o.position.y() = m.pose.position.y;
    o.position.z() = m.pose.position.z;
    o.radius = 0.5 * m.scale.x;
    o.is_dynamic = !m.frame_locked;
    // Velocity encoding convention (documented in the launch files): a dynamic
    // obstacle's velocity is stored in the marker's `points[0]` field, with
    // frame_locked == false. Static markers carry no points.
    if (o.is_dynamic && m.points.size() > 0) {
      o.velocity.x() = m.points[0].x;
      o.velocity.y() = m.points[0].y;
      o.velocity.z() = m.points[0].z;
    }
    out.push_back(o);
  }
  if (ignored > 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "obstaclesFromMarkers: ignored %zu non-SPHERE/CYLINDER marker(s)", ignored);
  }
  return out;
}

geometry_msgs::msg::TwistStamped MpcCbfNode::twistFromInput(const Eigen::VectorXd & u) const
{
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = now();
  out.header.frame_id = frame_id_;
  switch (mpc_config_.model) {
    case ModelType::kDoubleIntegrator2D:
      // Accelerations; the plant integrates them into velocities (§9.5, and
      // documented in the launch files).
      out.twist.linear.x = u(0);
      out.twist.linear.y = u(1);
      break;
    case ModelType::kUnicycle2D:
      out.twist.linear.x = u(0);
      out.twist.angular.z = u(1);
      break;
    case ModelType::kBicycleKinematic:
      out.twist.linear.x = u(0);
      out.twist.angular.z = u(1);  // steering angle [rad]
      break;
    case ModelType::kQuadrotorPlanar:
      out.twist.linear.x = u(0);  // thrust [N]
      out.twist.angular.y = u(1);  // pitch torque [N m] (y is the planar axis)
      break;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void MpcCbfNode::publishPredictedPath(const Eigen::MatrixXd & x_pred)
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = frame_id_;
  const int nx = MpcCbfSolver::stateDimOf(mpc_config_.model);
  const int n_steps = static_cast<int>(x_pred.cols());
  path.poses.reserve(static_cast<std::size_t>(n_steps));
  for (int k = 0; k < n_steps; ++k) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = x_pred(0, k);
    // Quadrotor's position axis is (px, pz) — publish z into position.z.
    ps.pose.position.y = (nx >= 2 && mpc_config_.model != ModelType::kQuadrotorPlanar) ?
      x_pred(1, k) : 0.0;
    ps.pose.position.z = (mpc_config_.model == ModelType::kQuadrotorPlanar) ?
      x_pred(1, k) : 0.0;
    if (mpc_config_.model == ModelType::kUnicycle2D ||
      mpc_config_.model == ModelType::kBicycleKinematic)
    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, x_pred(2, k));
      ps.pose.orientation = tf2::toMsg(q);
    } else if (mpc_config_.model == ModelType::kQuadrotorPlanar) {
      tf2::Quaternion q;
      q.setRPY(0.0, x_pred(4, k), 0.0);
      ps.pose.orientation = tf2::toMsg(q);
    }
    path.poses.push_back(ps);
  }
  path_pub_->publish(path);
}

void MpcCbfNode::publishCbfValues(const SolverDiagnostics & diagnostics)
{
  // MultiArrayLayout: dim[0] = stage (0..N), dim[1] = obstacle (0..n_obs).
  std_msgs::msg::Float64MultiArray msg;
  msg.data = diagnostics.cbf_values;
  const std::size_t n_obs = (diagnostics.cbf_values.size() == 0) ?
    0u :
    diagnostics.cbf_values.size() / static_cast<std::size_t>(mpc_config_.horizon + 1);
  std_msgs::msg::MultiArrayDimension stage_dim;
  stage_dim.label = "stage";
  stage_dim.size = static_cast<std::uint32_t>(mpc_config_.horizon + 1);
  stage_dim.stride = static_cast<std::uint32_t>(n_obs);
  std_msgs::msg::MultiArrayDimension obs_dim;
  obs_dim.label = "obstacle";
  obs_dim.size = static_cast<std::uint32_t>(n_obs);
  obs_dim.stride = 1u;
  msg.layout.dim.push_back(stage_dim);
  msg.layout.dim.push_back(obs_dim);
  msg.layout.data_offset = 0;
  cbf_pub_->publish(msg);
}

void MpcCbfNode::publishDiagnostics(
  SolverStatus status, const SolverDiagnostics & diagnostics)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "mpc_cbf";
  st.message = toString(status);

  const double period_ms = 1000.0 / control_rate_hz_;
  // A solve eating more than 80% of the control period is one scheduling
  // hiccup away from starving the loop.
  st.level = (diagnostics.solve_time_ms > 0.8 * period_ms) ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;

  auto add = [&st](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      st.values.push_back(kv);
    };
  add("solve_time_ms", std::to_string(diagnostics.solve_time_ms));
  add("sqp_iterations", std::to_string(diagnostics.sqp_iterations));
  add("kkt_residual", std::to_string(diagnostics.kkt_residual));
  add("cost", std::to_string(diagnostics.cost));
  double h_min = std::numeric_limits<double>::infinity();
  for (double h : diagnostics.cbf_values) {
    h_min = std::min(h_min, h);
  }
  add("min_cbf", std::to_string(h_min));
  double max_slack = 0.0;
  for (double s : diagnostics.cbf_slack) {
    max_slack = std::max(max_slack, s);
  }
  add("max_cbf_slack", std::to_string(max_slack));
  // Realised omega: the first value; empty unless relaxed_decay.
  add("omega", diagnostics.omega.empty() ?
      "1.0" : std::to_string(diagnostics.omega.front()));
  if (use_tube_mpc_) {
    add("clip_count", std::to_string(diagnostics.clip_count));
    add("tube_resets", std::to_string(diagnostics.tube_resets));
  }
  arr.status.push_back(st);
  diagnostics_pub_->publish(arr);
}

}  // namespace mpc_cbf_unified

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mpc_cbf_unified::MpcCbfNode> node;
  try {
    // The constructor runs declare/load/setup; a configuration or solver
    // failure is reported FATAL there and rclcpp::shutdown() is called, so a
    // node that cannot honour its contract never starts spinning.
    node = std::make_shared<mpc_cbf_unified::MpcCbfNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mpc_cbf_node"),
      "node constructor threw: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (!rclcpp::ok()) {
    return 1;  // the constructor already shut down
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
