#!/usr/bin/env python3
# Copyright (c) 2026, Ali-Eimaan. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
Planar-quadrotor simulator with a moving obstacle.

Integrates the planar quadrotor (m = 1 kg, I = 0.1 kg m^2, g = 9.81 m/s^2):
    p_x_dot = vx,   p_z_dot = vz,
    v_x_dot = (T / m) sin(phi),   v_z_dot = (T / m) cos(phi) - g,
    phi_dot = phi_rate,   phi_rate_dot = tau / I.
cmd_vel.linear.x is the thrust T and cmd_vel.angular.y the pitch torque tau
(node convention §9.5). The state is [px, pz, vx, vz, phi, phi_rate].

The obstacle is a SPHERE marker moving at constant `--obstacle-speed` toward
the vehicle start (frame_locked := false, velocity in the `points` field): the
solver's constant-velocity prediction is exact here, so a CBF-QP failure is
not a prediction-error artefact. The quad avoids vertically.
"""

import argparse
import math

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class QuadrotorPlanarSim(Node):
    """Planar-quadrotor plant with a constant-velocity obstacle."""

    G = 9.81
    MASS = 1.0
    INERTIA = 0.1

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('sim_quadrotor_planar')
        self.args = args
        self.state = [0.0, args.start_z, 0.0, 0.0, 0.0, 0.0]
        self.cmd = [self.G, 0.0]  # T, tau
        self.obstacle_x = args.obstacle_x

        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.on_cmd, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)

        self.timer = self.create_timer(1.0 / args.rate, self.step)

    def on_cmd(self, msg: TwistStamped) -> None:
        """Store the latest (thrust, pitch-torque) command."""
        self.cmd = [msg.twist.linear.x, msg.twist.angular.y]

    def step(self) -> None:
        """Advance the plant and the obstacle, then publish everything."""
        dt = 1.0 / self.args.rate
        ds = dt / self.args.substeps
        thrust, tau = self.cmd
        for _ in range(self.args.substeps):
            px, pz, vx, vz, phi, phi_rate = self.state
            self.state[0] += vx * ds
            self.state[1] += vz * ds
            self.state[2] += thrust / self.MASS * math.sin(phi) * ds
            self.state[3] += (
                thrust / self.MASS * math.cos(phi) - self.G
            ) * ds
            self.state[4] += phi_rate * ds
            self.state[5] += tau / self.INERTIA * ds
        self.obstacle_x -= self.args.obstacle_speed * dt  # toward the start

        self.publish_odom()
        self.publish_obstacles()
        self.publish_goal()

    def publish_odom(self) -> None:
        """Odom with pitch about Y; twist in the body frame."""
        px, pz, vx, vz, phi, _ = self.state
        c, s = math.cos(phi), math.sin(phi)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = px
        msg.pose.pose.position.z = pz  # planar: pz in position.z (node §9.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = math.sin(phi / 2.0)
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = math.cos(phi / 2.0)
        # Body-frame twist: v_b = R(phi)^T v_w with R = [c s; -s c]; the
        # out-of-plane body axis carries no velocity (linear.y == 0).
        msg.twist.twist.linear.x = c * vx - s * vz
        msg.twist.twist.linear.z = s * vx + c * vz
        msg.twist.twist.angular.y = self.state[5]
        self.odom_pub.publish(msg)

    def publish_obstacles(self) -> None:
        """Publish a SPHERE marker moving at constant velocity toward the quad."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'moving_obstacle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.obstacle_x
        marker.pose.position.z = self.args.start_z
        marker.scale.x = 2.0 * self.args.obstacle_radius
        marker.scale.y = 2.0 * self.args.obstacle_radius
        marker.scale.z = 2.0 * self.args.obstacle_radius
        marker.color.r = 0.9
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.8
        marker.frame_locked = False  # dynamic: velocity in the points field
        marker.points.append(marker.pose.position)
        marker.points[0].x = -self.args.obstacle_speed  # vx in points field
        marker.points[0].y = 0.0
        marker.points[0].z = 0.0
        array = MarkerArray()
        array.markers.append(marker)
        self.obstacle_pub.publish(array)

    def publish_goal(self) -> None:
        """Constant goal pose; the quad tracks it while avoiding the obstacle."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.args.goal_x
        msg.pose.position.z = self.args.goal_z
        self.goal_pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    """Parse arguments and run the planar-quadrotor simulator."""
    parser = argparse.ArgumentParser(
        description='Planar-quadrotor plant simulator for the dynamic-obstacle demo'
    )
    parser.add_argument('--goal-x', type=float, default=4.0)
    parser.add_argument('--goal-z', type=float, default=0.3)
    parser.add_argument('--start-z', type=float, default=0.3)
    parser.add_argument('--obstacle-x', type=float, default=2.0)
    parser.add_argument('--obstacle-speed', type=float, default=2.0)
    parser.add_argument('--obstacle-radius', type=float, default=0.4)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--substeps', type=int, default=4)
    args = parser.parse_args(argv)

    rclpy.init()
    node = QuadrotorPlanarSim(args)
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
