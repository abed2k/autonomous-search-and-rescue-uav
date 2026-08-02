#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleOdometry
import math

class MicroXRCEWaypointNav(Node):
    def __init__(self):
        super().__init__('microxrce_waypoint_nav')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        # ✅ QoS for PX4 odometry (best-effort, not reliable)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # Waypoints (x, y, z, yaw) in NED frame
        self.waypoints = [
            (0.0, 0.0, -5.0, 0.0),   # Takeoff to 5m altitude
            (5.0, 0.0, -5.0, 0.0),   # Move forward
            (5.0, 5.0, -5.0, 0.0),   # Move right
            (0.0, 5.0, -5.0, 0.0),   # Move back
            (0.0, 0.0, -5.0, 0.0),   # Return home
        ]
        self.current_waypoint_index = 0
        self.current_position = (0.0, 0.0, 0.0)
        self.reached_threshold = 0.5  # meters

        # 30 Hz timer
        self.timer = self.create_timer(0.033, self.control_loop)

        self.get_logger().info('✅ MicroXRCE Waypoint Navigation Node Started with Odometry Feedback')

    def odom_callback(self, msg: VehicleOdometry):
        # PX4 VehicleOdometry gives position in NED
        self.current_position = (msg.position[0], msg.position[1], msg.position[2])

    def control_loop(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("🎉 All waypoints reached")
            return

        target = self.waypoints[self.current_waypoint_index]

        # Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_pub.publish(offboard_msg)

        # Publish TrajectorySetpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position[0] = target[0]
        setpoint.position[1] = target[1]
        setpoint.position[2] = target[2]
        setpoint.yaw = target[3]
        self.trajectory_pub.publish(setpoint)

        # Check if reached
        if self._is_close(self.current_position, target):
            self.get_logger().info(
                f"✅ Reached waypoint {self.current_waypoint_index}: {target}"
            )
            self.current_waypoint_index += 1

    def _is_close(self, pos, target):
        dx = pos[0] - target[0]
        dy = pos[1] - target[1]
        dz = pos[2] - target[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return dist < self.reached_threshold


def main(args=None):
    rclpy.init(args=args)
    node = MicroXRCEWaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
