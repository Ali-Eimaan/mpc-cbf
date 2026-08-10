# §9 · ROS node, configuration, launch and simulators

**Governs:** `mpc_cbf_node.hpp` / `.cpp`, `config/*.yaml`, `launch/*.py`, `scripts/*.py`
**Milestone:** M8
**Done when:** `ros2 launch mpc_cbf_unified 2d_obstacle.launch.py` reaches the goal safely with no
arguments, from a clean clone.

M8 comes after the parity check (M5), deliberately. See
[15_ROADMAP.md §15](15_ROADMAP.md) — a demo of a wrong controller is worse than no demo.

---

## §9.1 Node type and lifecycle

A plain `rclcpp::Node` with a wall timer, **not** a lifecycle node. The sibling `cbf-safety-filter`
uses a lifecycle node because it is a filter inserted into someone else's control path and needs
an explicit activate/deactivate contract. This node owns its loop and has no such requirement;
adding lifecycle here would be ceremony without a consumer.

Constructor order: `declareParameters()` → `loadParameters()` → `setupSolver()` →
`setupInterfaces()`. On failure in either middle step, log `FATAL` with the reason and
`rclcpp::shutdown()`. **A controller that starts with a silently-substituted configuration is
worse than one that refuses to start.**

Wrap the constructor in `main()` with a `try`/`catch (const std::exception&)` and log `FATAL` —
an exception escaping a node constructor produces a stack trace that names nothing useful.

## §9.2 Parameters

Every key in `config/mpc_cbf_params.yaml` and `config/tube_mpc_params.yaml`, declared with a
`rcl_interfaces::msg::ParameterDescriptor` carrying a description and, where meaningful, a range
(`gamma` in `(0,1]`, `control_rate_hz > 0`).

| Class | Keys | Why |
| --- | --- | --- |
| **read-only** | `model`, `horizon`, `dt`, `cbf_variant`, `use_tube_mpc`, everything under `tube:` | changing them needs a different generated solver, or an `initialize()` that must not run in the loop |
| **live-changeable** | `gamma`, `Q`, `R`, `Qf`, `safety_margin`, `ego_radius`, `infeasible_policy` | cheap to apply; useful during tuning |
| **rejected** | everything else | with a reason string in the `SetParametersResult` |

Register an `add_on_set_parameters_callback` that validates before accepting. A parameter callback
that accepts a value the solver will later reject moves the failure from a clear message to a
mysterious one.

**Parameter names must match the YAML byte-for-byte.** A near-miss silently uses the default, and
the symptom is a tuning change that does nothing.

The YAML keys and defaults are fixed in the two config files; treat them as the contract and keep
them in sync with `declareParameters()` in the same change.

## §9.3 Topics

| Direction | Default | Type | Notes |
| --- | --- | --- | --- |
| sub | `~/odom` | `nav_msgs/Odometry` | `SensorDataQoS` |
| sub | `~/goal` | `geometry_msgs/PoseStamped` | default QoS |
| sub | `~/obstacles` | `visualization_msgs/MarkerArray` | `SensorDataQoS` |
| pub | `~/cmd_vel` | `geometry_msgs/TwistStamped` | the applied input |
| pub | `~/predicted_path` | `nav_msgs/Path` | `x_pred`, for RViz |
| pub | `~/cbf_values` | `std_msgs/Float64MultiArray` | `h` over the horizon, with a layout describing the `(stage, obstacle)` shape so PlotJuggler can slice it |
| pub | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | solver health |

Put the control timer in its own `MutuallyExclusive` callback group so a slow solve cannot
re-enter itself.

## §9.4 The control loop and failure policy

```
1  copy the shared state under state_mutex_; release the lock before solving
2  if (now − last_odom_stamp_) > odom_timeout_s:
       publish the fallback input, DiagnosticStatus ERROR "odometry stale (<age> s)", return
3  solve
4  if !solution.usable(): handleSolverFailure(solution); return
5  publish cmd, predicted path, cbf values, diagnostics; consecutive_failures_ = 0
```

Never hold the mutex across the solve. The solve is the longest operation in the node and the
subscriptions are the freshest data source; blocking them is how you end up controlling on a
stale state during exactly the moments that matter.

`infeasible_policy`:

| Policy | Behaviour |
| --- | --- |
| `hold_last` | republish the previous input |
| `zero` | publish zeros |
| `brake` | maximum-deceleration input admissible under `u_min`/`u_max` |
| `previous_horizon` | apply `u_pred.col(i)` from the last successful solve, advancing `i` each failure; fall back to `brake` when the stored horizon runs out |

`previous_horizon` is the default because it is the only one with any relationship to the
constraints that were satisfied when it was computed. **Say exactly that in the code comment, and
say that none of the four is a guarantee** ([01_OVERVIEW.md §1.6](01_OVERVIEW.md)). The relative
merits are measured, not assumed, in [12_ANALYSIS.md §12.4](12_ANALYSIS.md).

Diagnostics on failure: `WARN` on the first, `ERROR` from `max_consecutive_failures` onward,
always carrying `infeasibility_reason` and `first_active_cbf_step` as key/value pairs. Do not
throw and do not shut down — recovery behaviour is a subject of study here, not an error path.

## §9.5 Conversions — where the real bugs live

**`stateFromOdometry`.** `nav_msgs/Odometry` carries **odom-frame pose** and **body-frame twist**.
For `kDoubleIntegrator2D` the state's `(vx, vy)` are world-frame, so the twist must be rotated by
the pose yaw. Write the rotation explicitly with a comment. Getting this wrong produces a
controller that works in simulation with zero initial yaw and fails on hardware — the most
expensive class of bug in the repository because the demo passes.

Per model:

```
kDoubleIntegrator2D : [px, py, vx, vy]          twist rotated into odom
kUnicycle2D         : [px, py, yaw]
kBicycleKinematic   : [px, py, yaw, v]          v = ‖body linear‖, signed by its x component
kQuadrotorPlanar    : [px, pz, vx, vz, pitch, pitch_rate]
```

**`obstaclesFromMarkers`.** `SPHERE` and `CYLINDER` only; `radius = scale.x / 2`. Ignore other
marker types silently but count them, and log the count throttled — a publisher sending `CUBE`
markers otherwise looks like an obstacle-detection failure. Document the velocity-encoding
convention in the launch file that produces the markers, not only here.

**`twistFromInput`.** Unicycle → `linear.x = u(0)`, `angular.z = u(1)`. Double integrator →
accelerations in `linear.x`/`linear.y`, with the plant integrating them; that is unusual enough
that it must be stated in the launch file's docstring as well as in the code.

## §9.6 Simulator scripts — `mpc_cbf_unified/scripts/`

Not in `INFO.md`; the launch files have no plant without them
([01_OVERVIEW.md §1.5](01_OVERVIEW.md)). Install per
[03_BUILD_SYSTEM.md §3.7](03_BUILD_SYSTEM.md).

| Script | Role |
| --- | --- |
| `sim_double_integrator.py` | integrates the plant at `1/dt`, publishes `Odometry`; `--disturbance {uniform,worst_case,gust}`, `--w-max`, `--seed` |
| `sim_bicycle.py` | bicycle plant + lead-vehicle marker |
| `sim_quadrotor_planar.py` | planar quadrotor plant + moving obstacle marker |
| `track_reference.py` | centre-line reference publisher for the racing scenario |
| `log_min_barrier.py` | records `min_k h(x_k)` per run to CSV for the sweep notebook |

These are **test fixtures, not products**. Keep them small, deterministic under a seed, and free
of hidden smoothing, saturation or rate-limiting that would flatter the controller. If a simulator
clamps a control the controller asked for, the safety result is about the clamp, not the
controller.

The plant must integrate at a step **no larger than `dt`** ([16_CONVENTIONS.md §16.5](16_CONVENTIONS.md));
sub-stepping is fine and preferable, and inter-sample `h` should be checked, not just `h` at the
sample instants.

`worst_case` in `sim_double_integrator.py` means the vertex of `W` maximising `−∇hᵀw` at the
current state — the adversary the RPI set must cover. It is the disturbance mode A8 is measured
against.

## §9.7 Launch files

Each of the four already documents its arguments and node list in its docstring. Requirements for
all four:

- Every argument gets a `DeclareLaunchArgument` with a `description`, so
  `ros2 launch … --show-args` is self-explanatory.
- Parameter overrides overlay the YAML; never duplicate a value that already lives in a config
  file.
- The default invocation works with **no arguments**.
- Anything that records data (`record_bag`, `trials > 1`) must run headless.

| File | Scenario | Notes |
| --- | --- | --- |
| `2d_obstacle.launch.py` | ACC 2021 2-D example | the A8-adjacent smoke test; `rviz:=true` by default |
| `car_racing.launch.py` | bicycle overtaking | `controller:={mpc_cbf,mpc_dc}` selects the variant for the comparison GIF |
| `quadrotor_dynamic_obstacle.launch.py` | planar quadrotor vs fast obstacle | `controller:={mpc_cbf,cbf_qp}`; `cbf_qp` is the `N=1` generated solver |
| `tube_mpc_disturbance.launch.py` | wind | `trials>1` writes one CSV row per trial for §12.5 |

`car_racing.launch.py` and `quadrotor_dynamic_obstacle.launch.py` exist to produce the two
comparison GIFs. Both must be runnable twice with a single argument changed, so the two traces in
each GIF differ in exactly one thing.
