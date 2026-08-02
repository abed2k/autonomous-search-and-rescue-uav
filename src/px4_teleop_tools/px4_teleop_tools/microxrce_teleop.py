#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
import threading

class MicroXRCETeleop(Node):
    def __init__(self):
        super().__init__('microxrce_teleop')
        
        # Subscribe to keyboard commands
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        
        # Publisher for TrajectorySetpoint
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Publisher for OffboardControlMode
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        
        # Timer for continuous publishing
        self.timer = self.create_timer(0.033, self.publish_setpoints)  # 30Hz
        
        self.current_twist = Twist()
        self.current_altitude = -5.0  # Default altitude (NED frame: negative = up)
        self.get_logger().info('MicroXRCE Teleop Node Started with Altitude Hold')
        
    def twist_callback(self, msg):
        self.current_twist = msg
        
        # Adjust altitude based on up/down commands
        if msg.linear.z > 0:  # Moving up
            self.current_altitude -= 0.1  # NED: more negative = higher
        elif msg.linear.z < 0:  # Moving down
            self.current_altitude += 0.1  # NED: less negative = lower
        
    def publish_setpoints(self):
        # 1. Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True       # ✅ NOW USING POSITION CONTROL (for altitude)
        offboard_msg.velocity = True       # ✅ USING velocity control for horizontal
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_pub.publish(offboard_msg)
        
        # 2. Publish TrajectorySetpoint with BOTH position and velocity
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # POSITION control for altitude (Z-axis)
        setpoint.position[0] = float('nan')  # X position - not controlled
        setpoint.position[1] = float('nan')  # Y position - not controlled  
        setpoint.position[2] = self.current_altitude  # Z position (altitude) - CONTROLLED ✅
        
        # VELOCITY control for horizontal movement
        setpoint.velocity[0] = self.current_twist.linear.x   # Forward (X)
        setpoint.velocity[1] = -self.current_twist.linear.y  # Right (Y)
        setpoint.velocity[2] = float('nan')  # Z velocity - not controlled (using position instead)
        
        setpoint.yawspeed = -self.current_twist.angular.z    # Yaw rate
        
        self.trajectory_pub.publish(setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = MicroXRCETeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
