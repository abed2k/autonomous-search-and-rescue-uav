#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <cmath>
#include <mutex>
#include <limits>  // NEW

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>

class PathFollower : public rclcpp::Node {
public:
  PathFollower() : Node("path_follower") {
    // Parameters for safety layer
    this->declare_parameter<std::string>("octomap_topic", "/octomap_full");
    // MATCH planner defaults
    this->declare_parameter<double>("collision_radius", 0.25);
    this->declare_parameter<double>("safety_margin", 0.05);

    this->get_parameter("octomap_topic", octomap_topic_);
    this->get_parameter("collision_radius", collision_radius_);
    this->get_parameter("safety_margin", safety_margin_);

    // Use BestEffort QoS to match PX4
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos_profile,
      std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

    // Octomap subscriber (for safety checks)
    auto octomap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    octomap_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    octomap_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, octomap_qos,
      std::bind(&PathFollower::octomapCallback, this, std::placeholders::_1));

    // Publishers  
    setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    
    offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);

    // Control timer (20Hz for PX4)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PathFollower::controlLoop, this));

    current_waypoint_index_ = 0;
    has_path_ = false;
    is_offboard_ = false;
    has_odom_ = false;
    current_enu_x_ = 0.0;
    current_enu_y_ = 0.0;
    current_enu_z_ = 0.0;

    has_hold_target_ = false;
    hold_enu_x_ = hold_enu_y_ = hold_enu_z_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "🔄 Path Follower started - Waiting for paths...");
    RCLCPP_INFO(this->get_logger(), "🛡️  Safety: octomap_topic='%s', collision_radius=%.2f, safety_margin=%.2f",
                octomap_topic_.c_str(), collision_radius_, safety_margin_);
  }

private:
  // ======== Callbacks ========

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path");
      return;
    }
    
    path_ = *msg;
    has_path_ = true;
    has_hold_target_ = false; // new path will define a new hold point

    // NEW: start at the closest waypoint to current pose (if we have odom),
    // so we don't snap back to the beginning of the path.
    if (has_odom_) {
      double best_dist = std::numeric_limits<double>::infinity();
      size_t best_index = 0;

      for (size_t i = 0; i < path_.poses.size(); ++i) {
        const auto &p = path_.poses[i].pose.position;
        double dx = p.x - current_enu_x_;
        double dy = p.y - current_enu_y_;
        double dz = p.z - current_enu_z_;
        double d = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_dist) {
          best_dist = d;
          best_index = i;
        }
      }

      current_waypoint_index_ = best_index;
      RCLCPP_INFO(this->get_logger(),
                  "📋 New path received with %zu waypoints, starting at closest waypoint %zu (distance %.2f m)",
                  path_.poses.size(), current_waypoint_index_ + 1, best_dist);
    } else {
      current_waypoint_index_ = 0;
      RCLCPP_INFO(this->get_logger(),
                  "📋 New path received with %zu waypoints (no odom yet, starting at waypoint 1)",
                  path_.poses.size());
    }
  }

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_odom_ = *msg;
    has_odom_ = true;
    
    // Convert NED to ENU for distance calculations
    current_enu_x_ = msg->position[0];
    current_enu_y_ = -msg->position[1];
    current_enu_z_ = -msg->position[2];
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(octree_mutex_);

    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
    if (tree && tree->getTreeType() == "OcTree") {
      octomap::OcTree *dt = dynamic_cast<octomap::OcTree *>(tree.release());
      octree_.reset(dt);
      RCLCPP_DEBUG(this->get_logger(), "Octomap updated in PathFollower");
    } else {
      RCLCPP_WARN(this->get_logger(), "PathFollower: Received non-OcTree octomap");
    }
  }

  void controlLoop() {
    if (!has_odom_) {
      return;
    }

    const uint64_t now = this->now().nanoseconds() / 1000;

    // 1) Always publish OffboardControlMode at control rate
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_msg.timestamp = now;
    offboard_pub_->publish(offboard_msg);

    // 2) Decide what to track: path waypoint or hold / current position
    double target_enu_x;
    double target_enu_y;
    double target_enu_z;

    bool using_path_waypoint = false;

    if (has_path_ && current_waypoint_index_ < path_.poses.size()) {
      auto &target_pose = path_.poses[current_waypoint_index_];
      double wp_x = target_pose.pose.position.x;
      double wp_y = target_pose.pose.position.y;
      double wp_z = target_pose.pose.position.z;

      // SAFETY CHECK: segment current pose -> waypoint
      if (isSegmentSafe(current_enu_x_, current_enu_y_, current_enu_z_,
                        wp_x, wp_y, wp_z)) {
        target_enu_x = wp_x;
        target_enu_y = wp_y;
        target_enu_z = wp_z;
        using_path_waypoint = true;

        hold_enu_x_ = target_enu_x;
        hold_enu_y_ = target_enu_y;
        hold_enu_z_ = target_enu_z;
        has_hold_target_ = true;

        double dx = target_enu_x - current_enu_x_;
        double dy = target_enu_y - current_enu_y_;
        double dz = target_enu_z - current_enu_z_;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < 0.8) {
          current_waypoint_index_++;
          if (current_waypoint_index_ >= path_.poses.size()) {
            RCLCPP_INFO(this->get_logger(), "🎯 Reached final waypoint!");
            has_path_ = false;
          } else {
            RCLCPP_INFO(this->get_logger(), "➡️  Advancing to waypoint %zu/%zu",
                        current_waypoint_index_ + 1, path_.poses.size());
          }
        }

      } else {
        // NEW: segment to this waypoint is unsafe -> try to jump forward
        size_t bad_index = current_waypoint_index_;
        bool found_safe_next = false;

        for (size_t i = current_waypoint_index_ + 1; i < path_.poses.size(); ++i) {
          auto &future_pose = path_.poses[i];
          double fx = future_pose.pose.position.x;
          double fy = future_pose.pose.position.y;
          double fz = future_pose.pose.position.z;

          if (isSegmentSafe(current_enu_x_, current_enu_y_, current_enu_z_,
                            fx, fy, fz)) {
            current_waypoint_index_ = i;
            RCLCPP_WARN(this->get_logger(),
                        "🚫 Segment to waypoint %zu is unsafe; skipping ahead to waypoint %zu/%zu",
                        bad_index, current_waypoint_index_ + 1, path_.poses.size());
            found_safe_next = true;
            break;
          }
        }

        if (found_safe_next) {
          // We'll use the new waypoint on the next control cycle
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "🚫 Segment to waypoint %zu is unsafe and no later waypoint is reachable. Rejecting current path.",
                      bad_index);
          has_path_ = false;
        }
      }
    }

    if (!using_path_waypoint) {
      if (has_hold_target_) {
        target_enu_x = hold_enu_x_;
        target_enu_y = hold_enu_y_;
        target_enu_z = hold_enu_z_;
      } else {
        target_enu_x = current_enu_x_;
        target_enu_y = current_enu_y_;
        target_enu_z = current_enu_z_;
      }
    }

    publishSetpoint(target_enu_x, target_enu_y, target_enu_z, now);
  }

  void publishSetpoint(double enu_x, double enu_y, double enu_z, uint64_t timestamp_us) {
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    
    // Convert ENU to NED for PX4
    setpoint.position[0] = enu_x;
    setpoint.position[1] = -enu_y;
    setpoint.position[2] = -enu_z;
    
    setpoint.velocity[0] = 0.0;
    setpoint.velocity[1] = 0.0; 
    setpoint.velocity[2] = 0.0;
    
    setpoint.acceleration[0] = 0.0;
    setpoint.acceleration[1] = 0.0;
    setpoint.acceleration[2] = 0.0;
    
    setpoint.yaw = 0.0;
    setpoint.yawspeed = 0.0;
    setpoint.timestamp = timestamp_us;
    
    setpoint_pub_->publish(setpoint);
    
    RCLCPP_DEBUG(this->get_logger(), "📤 Setpoint: NED(%.2f, %.2f, %.2f)", 
                 setpoint.position[0], setpoint.position[1], setpoint.position[2]);
  }

  // ======== SAFETY HELPERS (octomap) ========

  bool isPositionValid(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    if (!octree_) {
      // If we have no map yet, be permissive
      return true;
    }

    // Check the exact voxel
    octomap::OcTreeNode *exact_node = octree_->search(x, y, z);
    if (exact_node && octree_->isNodeOccupied(exact_node)) {
      return false;
    }

    // Sample around the position within collision_radius_ + safety_margin_
    const int samples = 6;
    const double max_r = collision_radius_ + safety_margin_;

    for (int i = -samples; i <= samples; ++i) {
      for (int j = -samples; j <= samples; ++j) {
        for (int k = -samples; k <= samples; ++k) {
          double dx = (i * max_r) / samples;
          double dy = (j * max_r) / samples;
          double dz = (k * max_r) / samples;

          double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (dist > max_r) continue;

          double qx = x + dx;
          double qy = y + dy;
          double qz = z + dz;

          octomap::OcTreeNode *node = octree_->search(qx, qy, qz);
          if (node && octree_->isNodeOccupied(node)) {
            return false;
          }
        }
      }
    }

    return true;
  }

  bool isSegmentSafe(double x0, double y0, double z0,
                     double x1, double y1, double z1) {
    const int steps = 15;
    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      double x = x0 + t * (x1 - x0);
      double y = y0 + t * (y1 - y0);
      double z = z0 + t * (z1 - z0);

      if (!isPositionValid(x, y, z)) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Segment unsafe at sample %d/%d: ENU(%.2f, %.2f, %.2f)",
                     i, steps, x, y, z);
        return false;
      }
    }
    return true;
  }

  // ======== Members ========

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path path_;
  size_t current_waypoint_index_;
  bool has_path_;
  bool is_offboard_;
  bool has_odom_;
  double current_enu_x_, current_enu_y_, current_enu_z_;
  px4_msgs::msg::VehicleOdometry current_odom_;

  bool has_hold_target_;
  double hold_enu_x_, hold_enu_y_, hold_enu_z_;

  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;
  std::string octomap_topic_;
  double collision_radius_;
  double safety_margin_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
