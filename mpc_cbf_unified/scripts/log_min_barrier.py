#!/usr/bin/env python3
# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
Logs min_k h(x_k) per episode to CSV.

Listens on /odom, /obstacles and /episode and records, for every episode, the
minimum barrier value over time:
    min_k h(x_k),  h(x) = min_o (|p - o|^2 - r_eff^2),
with r_eff = obstacle_radius + ego_radius + safety_margin (the same effective
radius the controller enforces, §4.2). A negative minimum means the obstacle
was clipped. The output CSV (one row per episode: episode, min_barrier_value,
seed) feeds analysis/disturbance_robustness_sweep.ipynb.
"""

import argparse
import csv
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray


class MinBarrierLogger(Node):
    """Accumulates per-episode minimum barrier values and writes the CSV."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('log_min_barrier')
        self.args = args
        self.rows: list[tuple[int, float, int]] = []
        self.episode = 0
        self.min_h = float('inf')
        self.obstacles: list[tuple[float, float, float]] = []  # x, y, r

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.obstacle_sub = self.create_subscription(
            MarkerArray, '/obstacles', self.on_obstacles, 10
        )
        self.episode_sub = self.create_subscription(Int32, '/episode', self.on_episode, 10)

    def on_obstacles(self, msg: MarkerArray) -> None:
        """Keep the latest obstacle set (x, y, radius)."""
        self.obstacles = []
        for marker in msg.markers:
            radius = marker.scale.x / 2.0
            self.obstacles.append(
                (marker.pose.position.x, marker.pose.position.y, radius)
            )

    def on_odom(self, msg: Odometry) -> None:
        """Track the minimum barrier over the current episode."""
        if not self.obstacles:
            return
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        margin = self.args.ego_radius + self.args.safety_margin
        h = min(
            (px - ox) ** 2 + (py - oy) ** 2 - (radius + margin) ** 2
            for ox, oy, radius in self.obstacles
        )
        self.min_h = min(self.min_h, h)

    def on_episode(self, msg: Int32) -> None:
        """Close the previous episode when the index advances."""
        if msg.data != self.episode:
            self.close_episode()
            self.episode = msg.data

    def close_episode(self) -> None:
        """Append the current episode's row (if any odometry arrived)."""
        if math.isfinite(self.min_h):
            self.rows.append((self.episode, self.min_h, self.args.seed))
            self.get_logger().info(
                'episode %d: min barrier h=%.3f' % (self.episode, self.min_h)
            )
        self.min_h = float('inf')

    def write_csv(self) -> None:
        """Flush all rows to --output."""
        self.close_episode()
        if not self.rows:
            return
        with open(self.args.output, 'w', newline='', encoding='utf-8') as fh:
            writer = csv.writer(fh)
            writer.writerow(['episode', 'min_barrier_value', 'seed'])
            writer.writerows(self.rows)
        self.get_logger().info(
            'wrote %d rows to %s' % (len(self.rows), self.args.output)
        )


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and run the logger."""
    parser = argparse.ArgumentParser(description='Per-episode min-barrier logger')
    parser.add_argument('--ego-radius', type=float, default=0.15)
    parser.add_argument('--safety-margin', type=float, default=0.05)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--output', type=str, default='tube_min_barrier_sweep.csv')
    args = parser.parse_args(argv)

    rclpy.init()
    node = MinBarrierLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
