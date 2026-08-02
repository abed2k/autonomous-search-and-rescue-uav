#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class odom_to_tf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        if not self.has_parameter("use_sim_time"): #for the node to use sim time to prevent any timestamp errors
            self.declare_parameter("use_sim_time", True)

        qos_profile = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscribe_ = self.create_subscription(VehicleOdometry, "fmu/out/vehicle_odometry", self.calc_tf, qos_profile) #subscribing to px4 odometry
        self.get_logger().info("subscribed to fmu/out/vehicle_odometry and calculated tf")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) #initialize broadcaster

    def calc_tf(self, msg: VehicleOdometry):
        stamp = self.get_clock().now().to_msg() #using current ros time
        
        #ned to enu for position 
        x_enu = float(msg.position[0]) #east
        y_enu = float(-msg.position[1]) #north
        z_enu = float(-msg.position[2]) #up

        #ned to enu for orientation
        q_enu = [float(msg.q[0]), float(msg.q[1]), float(-msg.q[2]), float(-msg.q[3])]  #[w, x, y, z]

        t_base = TransformStamped() #object t_base of class TransformStamped
        t_base.header.stamp = stamp #using current ros time
        t_base.header.frame_id = "map" #parent link
        t_base.child_frame_id = "base_link" #child link

        t_base.transform.translation.x = x_enu #translation transform
        t_base.transform.translation.y = y_enu
        t_base.transform.translation.z = z_enu

        t_base.transform.rotation.x = q_enu[1] #orientation transform
        t_base.transform.rotation.y = q_enu[2]
        t_base.transform.rotation.z = q_enu[3]
        t_base.transform.rotation.w = q_enu[0]

        self.tf_broadcaster.sendTransform(t_base) #broadcast tf

def main(args=None):
    rclpy.init(args=args)
    node = odom_to_tf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()