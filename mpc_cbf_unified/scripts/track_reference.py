#!/usr/bin/env python3
# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
Centre-line reference publisher for the overtake demo.

Publishes /goal on the track centre line (y = 0, yaw = 0) advancing at
`--speed`. The reference position wraps around `--track-length` so it never
runs away; car_racing.launch.py sets the ego reference speed (1.5 m/s) above
the lead vehicle's (0.8 m/s), forcing an overtake within the lane.
"""

import argparse
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class TrackReference(Node):
    """Advances a centre-line goal at a constant reference speed."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('track_reference')
        self.args = args
        self.t0 = self.get_clock().now()
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.timer = self.create_timer(1.0 / args.rate, self.step)

    def step(self) -> None:
        """Publish the current centre-line goal pose."""
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1.0e9
        x = (self.args.speed * elapsed) % self.args.track_length
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = 0.0
        msg.pose.orientation.z = math.sin(0.0)
        msg.pose.orientation.w = math.cos(0.0)
        self.goal_pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and run the reference publisher."""
    parser = argparse.ArgumentParser(description='Centre-line goal publisher')
    parser.add_argument('--speed', type=float, default=1.5)
    parser.add_argument('--track-length', type=float, default=60.0)
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = TrackReference(args)
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
