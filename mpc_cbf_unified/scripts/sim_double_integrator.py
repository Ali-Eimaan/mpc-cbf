#!/usr/bin/env python3
# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
2-D double-integrator simulator.

Integrates the double-integrator plant
    p_dot = v,  v_dot = u,
with Euler sub-stepping at a finer rate than the control rate. The control
input arrives on /cmd_vel as a TwistStamped where linear.x/y are the
accelerations [m/s^2] (the node's double-integrator convention, §9.5).
Odometry is published on /odom, the scenario on /obstacles (SPHERE markers),
the goal on /goal, and the episode index on /episode.

Disturbances: with `--disturbance` the velocity is perturbed once per control
step by w drawn from the tube W box (position half-width 0.005 m, velocity
half-width 0.02 m/s, matching config/tube_mpc_params.yaml) scaled by
`--w-max`. `worst_case` picks the vertex of W maximising -grad h^T w (i.e.
pushing toward the obstacle); `uniform` samples the box; `gust` applies a
constant vertex direction for a random 0.5-2 s window before re-sampling.

Episodes: the run is organised into episodes that reset when the goal is
reached. With `--trials 1` (the interactive demo default) the simulation runs
until the node is shut down; with `--trials N > 1` it runs exactly N episodes
and then shuts down (the batch mode used by disturbance sweeps).
"""

import argparse
import math
import random

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray


class DoubleIntegratorSim(Node):
    """Publishes odometry, the goal, and the obstacle; integrates cmd_vel."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('sim_double_integrator')
        self.args = args
        self.rng = random.Random(args.seed)

        self.state = [0.0, 0.0, 0.0, 0.0]  # px, py, vx, vy
        self.cmd = [0.0, 0.0]
        self.episode = 0
        self.gust_dir = self._sample_vertex()
        self.gust_until = self._now() + self.rng.uniform(0.5, 2.0)

        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.on_cmd, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.episode_pub = self.create_publisher(Int32, '/episode', 10)

        self.timer = self.create_timer(1.0 / args.rate, self.step)

    def on_cmd(self, msg: TwistStamped) -> None:
        """Store the latest acceleration command."""
        self.cmd = [msg.twist.linear.x, msg.twist.linear.y]

    def _now(self) -> float:
        """Return the monotonic wall-clock time in seconds."""
        return self.get_clock().now().nanoseconds / 1.0e9

    def _sample_vertex(self) -> list[float]:
        """Return a random vertex of the W box."""
        return [
            self.rng.choice([-1.0, 1.0]) * 0.005,
            self.rng.choice([-1.0, 1.0]) * 0.005,
            self.rng.choice([-1.0, 1.0]) * 0.02,
            self.rng.choice([-1.0, 1.0]) * 0.02,
        ]

    def disturbance(self) -> list[float]:
        """Return one per-control-step disturbance w in W (scaled by --w-max)."""
        w_max = self.args.w_max
        profile = self.args.disturbance
        if profile == 'none':
            return [0.0, 0.0, 0.0, 0.0]
        if profile == 'worst_case':
            # Vertex of W maximising -grad h^T w. grad h = 2(x - o) in the
            # position components only, so w pushes position toward the
            # obstacle; zero-gradient (velocity) components take the
            # +half-width vertex (deterministic tie-break).
            wx, wy = self.state[0], self.state[1]
            ox, oy = self.args.obstacle_x, self.args.obstacle_y
            dx, dy = ox - wx, oy - wy
            return [
                (math.copysign(0.005, dx) if dx != 0.0 else 0.005) * w_max,
                (math.copysign(0.005, dy) if dy != 0.0 else 0.005) * w_max,
                0.02 * w_max,
                0.02 * w_max,
            ]
        if profile == 'uniform':
            return [
                self.rng.uniform(-0.005, 0.005) * w_max,
                self.rng.uniform(-0.005, 0.005) * w_max,
                self.rng.uniform(-0.02, 0.02) * w_max,
                self.rng.uniform(-0.02, 0.02) * w_max,
            ]
        # gust: constant vertex direction for a random window.
        if self._now() >= self.gust_until:
            self.gust_dir = self._sample_vertex()
            self.gust_until = self._now() + self.rng.uniform(0.5, 2.0)
        return [w * w_max for w in self.gust_dir]

    def step(self) -> None:
        """Advance the plant one control period, then publish everything."""
        dt = 1.0 / self.args.rate
        substeps = self.args.substeps
        ds = dt / substeps

        # Disturbance is applied once per control step (discrete W in the
        # tube model: x_{k+1} = A x_k + B u_k + w_k).
        w = self.disturbance()
        self.state[2] += w[2]
        self.state[3] += w[3]

        # Integrate the plant with sub-stepping; check h at every sub-step
        # (inter-sample constraint, §12.3).
        min_h = float('inf')
        for _ in range(substeps):
            self.state[2] += self.cmd[0] * ds
            self.state[3] += self.cmd[1] * ds
            self.state[0] += self.state[2] * ds
            self.state[1] += self.state[3] * ds
            h = self.barrier_value()
            min_h = min(min_h, h)
        if min_h < 0.0:
            self.get_logger().warn(
                'inter-sample barrier violation h=%.3f (obstacle clipped)' % min_h,
                throttle_duration_sec=1.0,
            )

        self.publish_odom()
        self.publish_obstacles()
        self.publish_goal()

        if self.goal_reached():
            self.get_logger().info(
                'episode %d complete: min barrier h=%.3f' % (self.episode, min_h)
            )
            if self.args.trials > 1 and self.episode + 1 >= self.args.trials:
                self.get_logger().info(
                    'batch of %d episodes done, shutting down' % self.args.trials
                )
                rclpy.shutdown()
            else:
                self.reset_episode()

    def barrier_value(self) -> float:
        """h(x) = |p - o|^2 - r_eff^2 for the single static obstacle."""
        r_eff = self.args.obstacle_radius + self.args.ego_radius + self.args.safety_margin
        dx = self.state[0] - self.args.obstacle_x
        dy = self.state[1] - self.args.obstacle_y
        return dx * dx + dy * dy - r_eff * r_eff

    def goal_reached(self) -> bool:
        """Report whether the state is within 5 cm of the goal and stopped."""
        gx, gy = self.args.goal_x, self.args.goal_y
        dist = math.hypot(self.state[0] - gx, self.state[1] - gy)
        speed = math.hypot(self.state[2], self.state[3])
        return dist < 0.05 and speed < 0.05

    def reset_episode(self) -> None:
        """Back to the start pose and publish the new episode index."""
        self.episode += 1
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.cmd = [0.0, 0.0]
        msg = Int32()
        msg.data = self.episode
        self.episode_pub.publish(msg)

    def publish_odom(self) -> None:
        """Publish /odom (the node consumes it as the state source)."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.twist.twist.linear.x = self.state[2]
        msg.twist.twist.linear.y = self.state[3]
        self.odom_pub.publish(msg)

    def publish_obstacles(self) -> None:
        """Publish one SPHERE marker; radius = scale.x / 2 (node convention)."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'static_obstacle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.args.obstacle_x
        marker.pose.position.y = self.args.obstacle_y
        marker.pose.position.z = 0.0
        marker.scale.x = 2.0 * self.args.obstacle_radius
        marker.scale.y = 2.0 * self.args.obstacle_radius
        marker.scale.z = 0.1
        marker.color.r = 0.9
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.8
        marker.frame_locked = True  # static obstacle: no velocity field
        array = MarkerArray()
        array.markers.append(marker)
        self.obstacle_pub.publish(array)

    def publish_goal(self) -> None:
        """Publish a constant /goal pose (the node tracks it)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.args.goal_x
        msg.pose.position.y = self.args.goal_y
        self.goal_pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and run the simulator."""
    parser = argparse.ArgumentParser(
        description='Double-integrator plant simulator for MPC-CBF demos'
    )
    parser.add_argument('--goal-x', type=float, default=1.0)
    parser.add_argument('--goal-y', type=float, default=1.0)
    parser.add_argument('--obstacle-x', type=float, default=0.5)
    parser.add_argument('--obstacle-y', type=float, default=0.5)
    parser.add_argument('--obstacle-radius', type=float, default=0.2)
    parser.add_argument('--ego-radius', type=float, default=0.15)
    parser.add_argument('--safety-margin', type=float, default=0.05)
    parser.add_argument(
        '--disturbance',
        type=str,
        default='none',
        choices=['none', 'uniform', 'worst_case', 'gust'],
    )
    parser.add_argument('--w-max', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--substeps', type=int, default=4)
    args = parser.parse_args(argv)

    rclpy.init()
    node = DoubleIntegratorSim(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
