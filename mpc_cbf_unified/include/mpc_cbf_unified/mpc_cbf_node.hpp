// Copyright (c) 2026, Ali-Eimaan. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS 2 (Lyrical Luth) wrapper around MpcCbfSolver / TubeMpcCbfSolver.

#ifndef MPC_CBF_UNIFIED__MPC_CBF_NODE_HPP_
#define MPC_CBF_UNIFIED__MPC_CBF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mpc_cbf_unified/mpc_cbf_solver.hpp"
#include "mpc_cbf_unified/tube_mpc_cbf_solver.hpp"

namespace mpc_cbf_unified
{

/// What to do when the solver returns a non-usable solution.
enum class InfeasiblePolicy
{
  kHoldLast,        ///< Republish the previous input.
  kZero,            ///< Publish zeros.
  kBrake,           ///< Maximum-deceleration input admissible under u_min/u_max.
  kPreviousHorizon  ///< Walk u_pred from the last usable solve, then brake.
};

/// Fixed-rate MPC-CBF controller node.
///
/// Topics (all names remappable, defaults shown):
///   sub  ~/odom            nav_msgs/Odometry                current state
///   sub  ~/goal            geometry_msgs/PoseStamped        set-point
///   sub  ~/obstacles       visualization_msgs/MarkerArray   SPHERE markers -> ObstacleState
///   pub  ~/cmd_vel         geometry_msgs/TwistStamped       applied input
///   pub  ~/predicted_path  nav_msgs/Path                    x_pred, for RViz
///   pub  ~/cbf_values      std_msgs/Float64MultiArray       h over the horizon
///   pub  /diagnostics      diagnostic_msgs/DiagnosticArray  solver health
///
/// Parameters are declared in config/mpc_cbf_params.yaml and
/// config/tube_mpc_params.yaml.
class MpcCbfNode : public rclcpp::Node
{
public:
  explicit MpcCbfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MpcCbfNode() override;

private:
  // -- setup ---------------------------------------------------------------
  /// Declare every parameter with a descriptor (range, description, read-only
  /// where the solver cannot absorb a runtime change).
  void declareParameters();
  /// Read parameters into mpc_config_/cbf_config_/tube_config_. Returns false
  /// on a value the solver would reject; the node then shuts down rather than
  /// running with a silently-clamped configuration.
  bool loadParameters();
  /// Build the solver selected by the `use_tube_mpc` parameter and call its
  /// initialize(). Returns false on failure.
  bool setupSolver();
  void setupInterfaces();

  // -- callbacks -----------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void obstacleCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  /// Control loop, fired at `control_rate_hz`. Skips (and publishes a stale
  /// warning) when no odometry has arrived within `odom_timeout_s`.
  void controlLoop();

  /// Handle a non-usable solve: apply the fallback policy selected by
  /// `infeasible_policy` (hold_last | zero | brake | previous_horizon) and
  /// raise the diagnostic level.
  void handleSolverFailure(SolverStatus status, const SolverDiagnostics & diagnostics);

  /// Validate live parameter changes before accepting them: only gamma,
  /// Q, R, Qf, safety_margin, ego_radius and infeasible_policy may change at
  /// runtime; everything else is rejected with a reason.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // -- conversions ---------------------------------------------------------
  /// nav_msgs/Odometry -> state vector for the configured ModelType.
  Eigen::VectorXd stateFromOdometry(const nav_msgs::msg::Odometry & msg) const;
  /// SPHERE/CYLINDER markers -> obstacles; ignores other marker types and
  /// treats a marker's scale.x/2 as its radius.
  std::vector<ObstacleState> obstaclesFromMarkers(
    const visualization_msgs::msg::MarkerArray & msg) const;
  /// Input vector -> TwistStamped for the configured ModelType.
  geometry_msgs::msg::TwistStamped twistFromInput(const Eigen::VectorXd & u) const;

  // -- publishing ----------------------------------------------------------
  void publishPredictedPath(const Eigen::MatrixXd & x_pred);
  void publishCbfValues(const SolverDiagnostics & diagnostics);
  void publishDiagnostics(SolverStatus status, const SolverDiagnostics & diagnostics);

  // -- members -------------------------------------------------------------
  MpcConfig mpc_config_;
  CbfConfig cbf_config_;
  TubeConfig tube_config_;
  bool use_tube_mpc_{false};

  std::unique_ptr<MpcCbfSolver> solver_;
  std::unique_ptr<TubeMpcCbfSolver> tube_solver_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cbf_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  /// Keeps the live-parameter callback alive; without the handle it is
  /// deregistered immediately (warn_unused_result).
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /// Guards the three fields below, which are written from subscription
  /// callbacks and read from the control timer.
  std::mutex state_mutex_;
  Eigen::VectorXd latest_state_;
  Eigen::VectorXd goal_state_;
  std::vector<ObstacleState> obstacles_;
  rclcpp::Time last_odom_stamp_;

  Eigen::VectorXd last_input_;
  std::size_t consecutive_failures_{0};
  std::string frame_id_{"map"};

  // -- runtime tuning (live-changeable parameters) --------------------------
  InfeasiblePolicy infeasible_policy_{InfeasiblePolicy::kPreviousHorizon};
  int max_consecutive_failures_{5};
  double odom_timeout_s_{0.5};
  double control_rate_hz_{10.0};

  // -- topic names (read from parameters; remap-friendly) -------------------
  std::string odom_topic_{"~/odom"};
  std::string goal_topic_{"~/goal"};
  std::string obstacle_topic_{"~/obstacles"};
  std::string cmd_topic_{"~/cmd_vel"};
  std::string predicted_path_topic_{"~/predicted_path"};
  std::string cbf_values_topic_{"~/cbf_values"};

  /// The control timer lives in its own callback group so a slow solve cannot
  /// re-enter itself.
  rclcpp::CallbackGroup::SharedPtr control_group_;

  /// Input columns from the last usable solve; the `previous_horizon` fallback
  /// walks through them one per failure, then brakes.
  Eigen::MatrixXd last_u_pred_;
  std::size_t fallback_index_{0};
};

}  // namespace mpc_cbf_unified

#endif  // MPC_CBF_UNIFIED__MPC_CBF_NODE_HPP_
