#!/usr/bin/env python3
# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
Kinematic-bicycle racing simulator.

Integrates the kinematic bicycle model used by car_racing.launch.py:
    p_x_dot = v cos(theta),  p_y_dot = v sin(theta),
    theta_dot = v tan(delta) / L,  v_dot = a,
with L = 0.35 m, sub-stepped below the control rate. cmd_vel.linear.x is the
acceleration and cmd_vel.angular.z the steering angle (node convention §9.5).

The lead vehicle is a CYLINDER marker moving along +x at `--lead-speed`
(frame_locked := false, velocity in the `points` field). The ego must overtake
it within the lane; track_reference.py supplies the /goal reference on the
centre line.
"""

import argparse
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class BicycleSim(Node):
    """Kinematic-bicycle plant with a moving lead-vehicle obstacle."""

    L = 0.35  # wheelbase [m]

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('sim_bicycle')
        self.args = args
        self.state = [0.0, 0.0, 0.0, 1.5]  # px, py, theta, v
        self.cmd = [0.0, 0.0]  # a, delta
        self.lead_x = args.lead_x

        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.on_cmd, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)

        self.timer = self.create_timer(1.0 / args.rate, self.step)

    def on_cmd(self, msg: TwistStamped) -> None:
        """Store the latest (acceleration, steering) command."""
        self.cmd = [msg.twist.linear.x, msg.twist.angular.z]

    def step(self) -> None:
        """Advance the plant and the lead vehicle, then publish everything."""
        dt = 1.0 / self.args.rate
        ds = dt / self.args.substeps
        a, delta = self.cmd
        for _ in range(self.args.substeps):
            px, py, theta, v = self.state
            self.state[0] += v * math.cos(theta) * ds
            self.state[1] += v * math.sin(theta) * ds
            self.state[2] += v * math.tan(delta) / self.L * ds
            self.state[3] += a * ds
        # Keep the lead in front of the ego once overtaken (it never stops).
        self.lead_x += self.args.lead_speed * dt

        self.publish_odom()
        self.publish_obstacles()

    def publish_odom(self) -> None:
        """Odom in body frame; the node rotates the twist by yaw."""
        px, py, theta, v = self.state
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = px
        msg.pose.pose.position.y = py
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.twist.twist.linear.x = v
        self.odom_pub.publish(msg)

    def publish_obstacles(self) -> None:
        """Publish the lead vehicle as a CYLINDER moving at --lead-speed."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'lead_vehicle'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.lead_x
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 2.0 * self.args.obstacle_radius
        marker.scale.y = 2.0 * self.args.obstacle_radius
        marker.scale.z = 0.4
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.9
        marker.color.a = 0.8
        marker.frame_locked = False  # dynamic: velocity in the points field
        marker.points.append(marker.pose.position)
        marker.points[0].x = self.args.lead_speed  # vx in the points field
        marker.points[0].y = 0.0
        marker.points[0].z = 0.0
        array = MarkerArray()
        array.markers.append(marker)
        self.obstacle_pub.publish(array)


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and run the bicycle simulator."""
    parser = argparse.ArgumentParser(
        description='Kinematic-bicycle plant simulator for the overtake demo'
    )
    parser.add_argument('--lead-speed', type=float, default=0.8)
    parser.add_argument('--lead-x', type=float, default=2.0)
    parser.add_argument('--obstacle-radius', type=float, default=0.45)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--substeps', type=int, default=4)
    args = parser.parse_args(argv)

    rclpy.init()
    node = BicycleSim(args)
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
