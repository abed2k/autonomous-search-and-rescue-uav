#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "frontier_extraction_msgs/msg/frontier.hpp"
#include "frontier_extraction_srvs/srv/get_frontiers.hpp"

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class AutonomousRRTStarPlanner : public rclcpp::Node
{
public:
  AutonomousRRTStarPlanner()
  : Node("autonomous_rrtstar_planner"),
    octree_(nullptr),
    have_start_(false),
    has_goal_(false),
    planning_in_progress_(false),
    autonomous_mode_(false),
    exploration_active_(false),
    last_frontier_update_(this->now())
  {
    // === WORLD LIMITS (hard) ===
    constexpr double WORLD_MIN_X = -9.0;
    constexpr double WORLD_MAX_X = 26.0;
    constexpr double WORLD_MIN_Y = -16.0;
    constexpr double WORLD_MAX_Y = 17.0;
    constexpr double WORLD_MIN_Z = 0.5;
    constexpr double WORLD_MAX_Z = 5.5;

    // params
    this->declare_parameter<std::string>("vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
    this->declare_parameter<std::string>("octomap_topic", "/octomap_full");
    this->declare_parameter<std::string>("goal_topic", "/planner_goal");
    this->declare_parameter<std::string>("planned_path_topic", "/planned_path");
    this->declare_parameter<double>("planning_time", 3.0);

    // MATCH path_follower-ish defaults
    this->declare_parameter<double>("collision_radius", 0.7);

    // Default bounds: maze box (will be clamped to WORLD_* anyway)
    this->declare_parameter<double>("min_x", WORLD_MIN_X);
    this->declare_parameter<double>("max_x", WORLD_MAX_X);
    this->declare_parameter<double>("min_y", WORLD_MIN_Y);
    this->declare_parameter<double>("max_y", WORLD_MAX_Y);
    this->declare_parameter<double>("min_z", WORLD_MIN_Z);
    this->declare_parameter<double>("max_z", WORLD_MAX_Z);

    // Exploration box defaults (can be smaller; clamped to world box later)
    this->declare_parameter<double>("explore_min_x", WORLD_MIN_X);
    this->declare_parameter<double>("explore_max_x", WORLD_MAX_X);
    this->declare_parameter<double>("explore_min_y", WORLD_MIN_Y);
    this->declare_parameter<double>("explore_max_y", WORLD_MAX_Y);
    this->declare_parameter<double>("explore_min_z", WORLD_MIN_Z);
    this->declare_parameter<double>("explore_max_z", WORLD_MAX_Z);
    
    // BIT* specific parameters
    this->declare_parameter<int>("samples_per_batch", 200);
    this->declare_parameter<int>("max_attempts", 5);

    this->declare_parameter<double>("safety_margin", 0.1);
    this->declare_parameter<int>("max_waypoints", 25);

    // Autonomous exploration parameters
    this->declare_parameter<bool>("autonomous_mode", false);
    this->declare_parameter<std::string>("frontier_topic", "/frontiers");
    this->declare_parameter<double>("exploration_timeout", 30.0);
    this->declare_parameter<double>("min_frontier_utility", 0.3);
    this->declare_parameter<double>("frontier_update_interval", 2.0);
    this->declare_parameter<double>("min_goal_distance", 1.0);
    this->declare_parameter<double>("visited_goal_radius", 3.0);

    // encourage going where the map is still unknown
    this->declare_parameter<double>("unknown_gain_radius", 1.5);
    this->declare_parameter<double>("min_unknown_ratio", 0.02); // slightly relaxed

    // Get parameters
    vehicle_topic_ = this->get_parameter("vehicle_odometry_topic").as_string();
    octomap_topic_ = this->get_parameter("octomap_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    planned_path_topic_ = this->get_parameter("planned_path_topic").as_string();
    planning_time_ = this->get_parameter("planning_time").as_double();
    collision_radius_ = this->get_parameter("collision_radius").as_double();
    
    // BIT* parameters
    samples_per_batch_ = this->get_parameter("samples_per_batch").as_int();
    max_attempts_ = this->get_parameter("max_attempts").as_int();
    safety_margin_ = this->get_parameter("safety_margin").as_double();
    max_waypoints_ = this->get_parameter("max_waypoints").as_int();

    // Autonomous parameters
    autonomous_mode_ = this->get_parameter("autonomous_mode").as_bool();
    frontier_topic_ = this->get_parameter("frontier_topic").as_string();
    exploration_timeout_ = this->get_parameter("exploration_timeout").as_double();
    min_frontier_utility_ = this->get_parameter("min_frontier_utility").as_double();
    frontier_update_interval_ = this->get_parameter("frontier_update_interval").as_double();
    min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();
    visited_goal_radius_ = this->get_parameter("visited_goal_radius").as_double();
    unknown_gain_radius_ = this->get_parameter("unknown_gain_radius").as_double();
    min_unknown_ratio_ = this->get_parameter("min_unknown_ratio").as_double();

    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();

    // === HARD-CLAMP PARAMS TO WORLD BOX ===
    min_x_ = std::max(min_x_, WORLD_MIN_X);
    max_x_ = std::min(max_x_, WORLD_MAX_X);
    min_y_ = std::max(min_y_, WORLD_MIN_Y);
    max_y_ = std::min(max_y_, WORLD_MAX_Y);
    min_z_ = std::max(min_z_, WORLD_MIN_Z);
    max_z_ = std::min(max_z_, WORLD_MAX_Z);

    // Exploration box (read + clamp inside world box)
    explore_min_x_ = this->get_parameter("explore_min_x").as_double();
    explore_max_x_ = this->get_parameter("explore_max_x").as_double();
    explore_min_y_ = this->get_parameter("explore_min_y").as_double();
    explore_max_y_ = this->get_parameter("explore_max_y").as_double();
    explore_min_z_ = this->get_parameter("explore_min_z").as_double();
    explore_max_z_ = this->get_parameter("explore_max_z").as_double();

    explore_min_x_ = std::max(explore_min_x_, min_x_);
    explore_max_x_ = std::min(explore_max_x_, max_x_);
    explore_min_y_ = std::max(explore_min_y_, min_y_);
    explore_max_y_ = std::min(explore_max_y_, max_y_);
    explore_min_z_ = std::max(explore_min_z_, min_z_);
    explore_max_z_ = std::min(explore_max_z_, max_z_);

    // subscribers / publishers
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_topic_, qos_profile,
      std::bind(&AutonomousRRTStarPlanner::px4OdomCallback, this, std::placeholders::_1));

    auto octomap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    octomap_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    octomap_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, octomap_qos,
      std::bind(&AutonomousRRTStarPlanner::octomapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      std::bind(&AutonomousRRTStarPlanner::goalCallback, this, std::placeholders::_1));

    // Service client for frontier extraction
    frontier_client_ = this->create_client<frontier_extraction_srvs::srv::GetFrontiers>("/get_frontiers");

    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(planned_path_topic_, 10);

    // Planning timer
    planning_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AutonomousRRTStarPlanner::planningLoop, this));

    // Frontier update timer
    frontier_timer_ = this->create_wall_timer(
      std::chrono::seconds(static_cast<int>(frontier_update_interval_)),
      std::bind(&AutonomousRRTStarPlanner::updateFrontiers, this));

    // Exploration monitoring timer
    exploration_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AutonomousRRTStarPlanner::explorationMonitor, this));

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "🤖 Autonomous BIT* Planner Started!");
    RCLCPP_INFO(this->get_logger(), "Autonomous Mode: %s", autonomous_mode_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "Vehicle odom topic: %s", vehicle_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Octomap topic: %s", octomap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Goal topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Path topic: %s", planned_path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning time: %.1f seconds", planning_time_);
    RCLCPP_INFO(this->get_logger(), "World bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
                min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    RCLCPP_INFO(this->get_logger(), "Explore box: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
                explore_min_x_, explore_max_x_,
                explore_min_y_, explore_max_y_,
                explore_min_z_, explore_max_z_);
    RCLCPP_INFO(this->get_logger(), "Autonomous exploration: %s", autonomous_mode_ ? "ACTIVE" : "INACTIVE");
    RCLCPP_INFO(this->get_logger(), "Visited goal radius: %.2f m", visited_goal_radius_);
    RCLCPP_INFO(this->get_logger(), "Unknown gain radius: %.2f m, min_unknown_ratio: %.2f",
                unknown_gain_radius_, min_unknown_ratio_);
    RCLCPP_INFO(this->get_logger(), "Collision radius: %.2f m, safety_margin: %.2f m",
                collision_radius_, safety_margin_);
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

  ~AutonomousRRTStarPlanner() {
    if (planning_thread_.joinable()) {
      planning_in_progress_ = false;
      planning_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "🧹 Autonomous planner node cleaned up");
  }

private:
  struct VisitedGoal {
    double x, y, z;
  };

  // --- subscribers / publishers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Client<frontier_extraction_srvs::srv::GetFrontiers>::SharedPtr frontier_client_;
  
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr frontier_timer_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;

  // parameters
  std::string vehicle_topic_, octomap_topic_, goal_topic_, planned_path_topic_;
  std::string frontier_topic_;
  double planning_time_;
  double collision_radius_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  // exploration box
  double explore_min_x_, explore_max_x_;
  double explore_min_y_, explore_max_y_;
  double explore_min_z_, explore_max_z_;
  int samples_per_batch_, max_attempts_, max_waypoints_;
  double safety_margin_;
  bool autonomous_mode_;
  double exploration_timeout_, min_frontier_utility_, frontier_update_interval_;
  double min_goal_distance_;
  double visited_goal_radius_;
  double unknown_gain_radius_;
  double min_unknown_ratio_;

  // state
  std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex start_mutex_;
  std::mutex frontier_mutex_;
  double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0;
  double goal_x_ = 0.0, goal_y_ = 0.0, goal_z_ = 0.0;
  bool have_start_;
  bool has_goal_;
  std::atomic<bool> planning_in_progress_;
  bool exploration_active_;
  std::thread planning_thread_;
  
  std::vector<frontier_extraction_msgs::msg::Frontier> current_frontiers_;
  rclcpp::Time last_exploration_goal_time_;
  rclcpp::Time last_frontier_update_;
  std::vector<VisitedGoal> visited_goals_;

  double current_goal_initial_dist_ = 0.0;

  // ---- helpers ----
  bool insideExploreBox(double x, double y, double z) const
  {
    return (x >= explore_min_x_ && x <= explore_max_x_ &&
            y >= explore_min_y_ && y <= explore_max_y_ &&
            z >= explore_min_z_ && z <= explore_max_z_);
  }

  bool isNearVisited(double x, double y, double z) const
  {
    for (const auto &g : visited_goals_) {
      double d = std::sqrt(
        std::pow(x - g.x, 2) +
        std::pow(y - g.y, 2) +
        std::pow(z - g.z, 2));
      if (d < visited_goal_radius_) {
        return true;
      }
    }
    return false;
  }

  void recordVisitedGoal(double x, double y, double z)
  {
    visited_goals_.push_back(VisitedGoal{x, y, z});
    RCLCPP_INFO(this->get_logger(),
                "📍 Marked visited goal at (%.2f, %.2f, %.2f). Total visited: %zu",
                x, y, z, visited_goals_.size());
  }

  // How much unknown space around (x,y,z)? (0..1)
  double unknownRatio(double x, double y, double z, double radius)
  {
    std::lock_guard<std::mutex> lk(octree_mutex_);
    if (!octree_) return 1.0;

    double res = octree_->getResolution();
    if (res <= 0.0) res = 0.1;

    int steps = std::max(1, static_cast<int>(radius / res));
    int total = 0;
    int unknown = 0;

    for (int ix = -steps; ix <= steps; ++ix) {
      for (int iy = -steps; iy <= steps; ++iy) {
        for (int iz = -steps; iz <= steps; ++iz) {
          double qx = x + ix * res;
          double qy = y + iy * res;
          double qz = z + iz * res;

          double d = std::sqrt(ix*ix + iy*iy + iz*iz) * res;
          if (d > radius) continue;

          if (qx < min_x_ || qx > max_x_ ||
              qy < min_y_ || qy > max_y_ ||
              qz < min_z_ || qz > max_z_) {
            continue;
          }

          ++total;
          octomap::OcTreeNode* node = octree_->search(qx, qy, qz);
          if (!node) {
            ++unknown;
          }
        }
      }
    }

    if (total == 0) return 0.0;
    return static_cast<double>(unknown) / static_cast<double>(total);
  }

  // Find a collision-free goal near the frontier centroid (cx,cy,cz)
  bool pickGoalNearFrontier(double cx, double cy, double cz,
                            double &gx, double &gy, double &gz)
  {
    // Try centroid first (only if inside explore box)
    if (insideExploreBox(cx, cy, cz) &&
        isPositionValid(cx, cy, cz)) {
      gx = cx;
      gy = cy;
      gz = cz;
      return true;
    }

    const double max_r   = 1.0;   // 1 m radius around frontier
    const double step_r  = 0.2;
    const double max_dz  = 0.6;   // up/down 0.6 m
    const double step_dz = 0.2;
    const int    num_ang = 16;

    for (double dz = -max_dz; dz <= max_dz; dz += step_dz) {
      for (double r = 0.0; r <= max_r; r += step_r) {
        for (int i = 0; i < num_ang; ++i) {
          double angle = (2.0 * M_PI * i) / static_cast<double>(num_ang);
          double x = cx + r * std::cos(angle);
          double y = cy + r * std::sin(angle);
          double z = cz + dz;

          if (!insideExploreBox(x, y, z)) {
            continue;
          }

          if (isPositionValid(x, y, z)) {
            gx = x;
            gy = y;
            gz = z;
            return true;
          }
        }
      }
    }

    return false;
  }

  // ---- callbacks ----
  void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    start_x_ = msg->position[0];
    start_y_ = -msg->position[1];
    start_z_ = -msg->position[2];
    
    if (!have_start_) {
      have_start_ = true;
      RCLCPP_INFO(this->get_logger(), "✓ Received first odometry: ENU(%.2f, %.2f, %.2f)", 
                  start_x_, start_y_, start_z_);
    }
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(octree_mutex_);
    
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
    
    if (tree && tree->getTreeType() == "OcTree")
    {
        octomap::OcTree* dt = dynamic_cast<octomap::OcTree*>(tree.release());
        octree_.reset(dt);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received non-ocTree octomap");
    }
  }

  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!have_start_) {
      RCLCPP_WARN(this->get_logger(), "No start pose yet (waiting for vehicle odometry)");
      return;
    }

    autonomous_mode_ = false;
    exploration_active_ = false;

    // Clamp manual goal to world bounds so user can't send us outside
    goal_x_ = std::min(std::max(msg->point.x, min_x_), max_x_);
    goal_y_ = std::min(std::max(msg->point.y, min_y_), max_y_);
    goal_z_ = std::min(std::max(msg->point.z, min_z_), max_z_);

    has_goal_ = true;

    double dx = goal_x_ - start_x_;
    double dy = goal_y_ - start_y_;
    double dz = goal_z_ - start_z_;
    current_goal_initial_dist_ = std::sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO(this->get_logger(),
                "🎯 Manual goal set to (clamped): ENU(%.2f, %.2f, %.2f)",
                goal_x_, goal_y_, goal_z_);
  }

  void updateFrontiers()
  {
    if (!autonomous_mode_ || !have_start_ || !octree_) return;
    requestFrontiers();
  }

  void requestFrontiers()
  {
    if (!frontier_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Frontier service not available - clearing frontier list");
      std::lock_guard<std::mutex> lk(frontier_mutex_);
      current_frontiers_.clear();
      return;
    }

    auto request = std::make_shared<frontier_extraction_srvs::srv::GetFrontiers::Request>();
    request->robot_pose.x = start_x_;
    request->robot_pose.y = start_y_;
    request->robot_pose.z = start_z_;

    auto response_callback = [this](rclcpp::Client<frontier_extraction_srvs::srv::GetFrontiers>::SharedFuture future) {
      try {
        auto response = future.get();
        std::lock_guard<std::mutex> lk(frontier_mutex_);
        current_frontiers_ = response->frontiers;
        last_frontier_update_ = this->now();
        RCLCPP_INFO(this->get_logger(), "🔄 Updated frontiers via service: %zu clusters", current_frontiers_.size());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Frontier service call failed: %s", e.what());
        std::lock_guard<std::mutex> lk(frontier_mutex_);
        current_frontiers_.clear();
      }
    };

    frontier_client_->async_send_request(request, response_callback);
  }

  void selectExplorationGoal()
  {
    if (!autonomous_mode_ || !have_start_) return;

    // Snapshot of start pose
    double sx, sy, sz;
    {
      std::lock_guard<std::mutex> lk_start(start_mutex_);
      sx = start_x_;
      sy = start_y_;
      sz = start_z_;
    }

    std::lock_guard<std::mutex> lk(frontier_mutex_);
    
    double best_utility = -1.0;
    double best_gx = 0.0, best_gy = 0.0, best_gz = 0.0;
    double best_cx = 0.0, best_cy = 0.0, best_cz = 0.0;
    bool found_any = false;
    
    for (const auto& frontier : current_frontiers_) {
      double cx = frontier.centroid.x;
      double cy = frontier.centroid.y;
      double cz = frontier.centroid.z;

      // Ignore frontier centroids outside exploration box
      if (!insideExploreBox(cx, cy, cz)) {
        continue;
      }

      if (frontier.number_of_points < 10) continue;

      // pick a free goal near the frontier
      double gx, gy, gz;
      if (!pickGoalNearFrontier(cx, cy, cz, gx, gy, gz)) {
        continue;
      }

      double goal_dist = std::sqrt(
          std::pow(gx - sx, 2) +
          std::pow(gy - sy, 2) +
          std::pow(gz - sz, 2));

      if (goal_dist < min_goal_distance_) {
        continue;
      }

      if (isNearVisited(gx, gy, gz)) {
        continue;
      }

      double frontier_dist = std::sqrt(
          std::pow(cx - sx, 2) +
          std::pow(cy - sy, 2) +
          std::pow(cz - sz, 2));

      double unk = unknownRatio(cx, cy, cz, unknown_gain_radius_);
      if (unk < min_unknown_ratio_) {
        continue;
      }

      double utility = frontier.number_of_points * frontier_dist * (1.0 + unk);

      if (utility > best_utility && utility > min_frontier_utility_) {
        best_utility = utility;
        best_gx = gx; best_gy = gy; best_gz = gz;
        best_cx = cx; best_cy = cy; best_cz = cz;
        found_any = true;
      }
    }
    
    if (!found_any) {
      RCLCPP_WARN(this->get_logger(),
                  "No suitable frontiers found (blocked / too close / visited) - falling back to random goal");

      std::mt19937 rng(std::random_device{}());
      std::uniform_real_distribution<double> dist_x(explore_min_x_ + 0.5, explore_max_x_ - 0.5);
      std::uniform_real_distribution<double> dist_y(explore_min_y_ + 0.5, explore_max_y_ - 0.5);
      std::uniform_real_distribution<double> dist_z(explore_min_z_ + 0.2, explore_max_z_ - 0.2);

      bool random_found = false;
      double best_rand_utility = -1.0;
      double rgx=0, rgy=0, rgz=0;

      for (int i = 0; i < 120; ++i) {
        double x = dist_x(rng);
        double y = dist_y(rng);
        double z = dist_z(rng);

        if (!insideExploreBox(x, y, z)) continue;

        double d = std::sqrt(
          std::pow(x - sx, 2) +
          std::pow(y - sy, 2) +
          std::pow(z - sz, 2));

        if (d < min_goal_distance_) continue;
        if (isNearVisited(x, y, z)) continue;
        if (!isPositionValid(x, y, z)) continue;

        double util = d;
        if (util > best_rand_utility) {
          best_rand_utility = util;
          rgx = x; rgy = y; rgz = z;
          random_found = true;
        }
      }

      if (!random_found) {
        RCLCPP_WARN(this->get_logger(), "Random exploration: could not find a valid goal");
        return;
      }

      {
        std::lock_guard<std::mutex> lk_start(start_mutex_);
        goal_x_ = rgx;
        goal_y_ = rgy;
        goal_z_ = rgz;
        has_goal_ = true;
        exploration_active_ = true;
        last_exploration_goal_time_ = this->now();
      }

      current_goal_initial_dist_ = std::sqrt(
          std::pow(goal_x_ - sx, 2) +
          std::pow(goal_y_ - sy, 2) +
          std::pow(goal_z_ - sz, 2));

      RCLCPP_INFO(this->get_logger(),
                  "🎯 Random exploration goal: ENU(%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_z_);
      return;
    }

    {
      std::lock_guard<std::mutex> lk_start(start_mutex_);
      goal_x_ = best_gx;
      goal_y_ = best_gy;
      goal_z_ = best_gz;
      has_goal_ = true;
      exploration_active_ = true;
      last_exploration_goal_time_ = this->now();
    }

    current_goal_initial_dist_ = std::sqrt(
        std::pow(best_gx - sx, 2) +
        std::pow(best_gy - sy, 2) +
        std::pow(best_gz - sz, 2));

    RCLCPP_INFO(this->get_logger(),
                "🎯 Selected exploration goal: ENU(%.2f, %.2f, %.2f) "
                "(from frontier ENU(%.2f, %.2f, %.2f)) - Utility: %.3f",
                goal_x_, goal_y_, goal_z_,
                best_cx, best_cy, best_cz,
                best_utility);
  }

  void explorationMonitor()
  {
    if (!autonomous_mode_) return;

    if (!has_goal_ && have_start_) {
      selectExplorationGoal();
    }

    // If the current goal has become invalid (new obstacle or left explore box),
    // drop it and pick a new one.
    if (has_goal_) {
      if (!insideExploreBox(goal_x_, goal_y_, goal_z_) ||
          !isPositionValid(goal_x_, goal_y_, goal_z_)) {
        RCLCPP_WARN(this->get_logger(),
                    "Current goal (%.2f, %.2f, %.2f) became invalid - dropping and replanning",
                    goal_x_, goal_y_, goal_z_);
        has_goal_ = false;
        exploration_active_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        selectExplorationGoal();
        return;
      }
    }

    if (!exploration_active_) return;

    auto now = this->now();
    auto time_since_goal = (now - last_exploration_goal_time_).seconds();

    // 1) Check if goal was reached
    double distance_to_goal = std::sqrt(
        std::pow(goal_x_ - start_x_, 2) + 
        std::pow(goal_y_ - start_y_, 2) + 
        std::pow(goal_z_ - start_z_, 2));
    
    if (distance_to_goal < 0.8) {
      RCLCPP_INFO(this->get_logger(), "✅ Reached exploration goal!");
      recordVisitedGoal(goal_x_, goal_y_, goal_z_);
      has_goal_ = false;
      exploration_active_ = false;
      
      std::this_thread::sleep_for(std::chrono::seconds(1));
      selectExplorationGoal();
      return;
    }

    // 2) If we timed out, DROP the goal but DON'T mark it visited
    if (time_since_goal > exploration_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "⏰ Exploration goal timed out (%.1f s) at (%.2f, %.2f, %.2f) - "
                  "dropping goal and selecting a new one",
                  time_since_goal, goal_x_, goal_y_, goal_z_);

      has_goal_ = false;
      exploration_active_ = false;

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      selectExplorationGoal();
      return;
    }
  }

  void planningLoop()
  {
    if (planning_thread_.joinable() && !planning_in_progress_) {
      planning_thread_.join();
    }

    if (!has_goal_ || !have_start_ || !octree_) {
      return;
    }

    double distance_to_goal = std::sqrt(
        std::pow(goal_x_ - start_x_, 2) + 
        std::pow(goal_y_ - start_y_, 2) + 
        std::pow(goal_z_ - start_z_, 2));
    
    if (distance_to_goal < 0.8) {
      RCLCPP_INFO(this->get_logger(), "✅ Reached goal!");
      has_goal_ = false;
      if (autonomous_mode_) {
        exploration_active_ = false;
      }
      return;
    }

    if (planning_in_progress_) {
      return;
    }

    planning_in_progress_ = true;

    planning_thread_ = std::thread([this]() {
      this->continuousPlan();
    });
  }

  void continuousPlan()
  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    
    if (!planning_in_progress_) {
      return;
    }
    
    double straight_line_dist = std::sqrt(
        std::pow(goal_x_ - start_x_, 2) + 
        std::pow(goal_y_ - start_y_, 2) + 
        std::pow(goal_z_ - start_z_, 2));
    
    double adaptive_timeout = std::min(4.0, std::max(1.5, straight_line_dist / 8.0));
    
    for (int attempt = 1; attempt <= max_attempts_; attempt++) {
        if (!planning_in_progress_) {
            break;
        }
        
        if (plan(start_x_, start_y_, start_z_, goal_x_, goal_y_, goal_z_, attempt, adaptive_timeout)) {
            planning_in_progress_ = false;
            return;
        }
        
        if (attempt < max_attempts_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "❌ Replanning failed - will retry next cycle");
    planning_in_progress_ = false;
  }

  // *** WORLD LIMITS + OCCUPIED-INFLATION CHECK ***
  bool isPositionValid(double x, double y, double z)
  {
    // Hard world limits
    if (x < min_x_ || x > max_x_ ||
        y < min_y_ || y > max_y_ ||
        z < min_z_ || z > max_z_) {
      return false;
    }

    std::lock_guard<std::mutex> lk(octree_mutex_);
    if (!octree_) return true;

    // exact voxel
    octomap::OcTreeNode* exact_node = octree_->search(x, y, z);
    if (exact_node && octree_->isNodeOccupied(exact_node)) {
      return false;
    }

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

          if (qx < min_x_ || qx > max_x_ ||
              qy < min_y_ || qy > max_y_ ||
              qz < min_z_ || qz > max_z_) {
            continue;
          }

          octomap::OcTreeNode* node = octree_->search(qx, qy, qz);
          // UNKNOWN ok; only block explicit occupied
          if (node && octree_->isNodeOccupied(node)) {
            return false;
          }
        }
      }
    }

    return true;
  }

  bool isStateValid(const ob::State *state)
  {
    const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
    double x = (*rstate)[0];
    double y = (*rstate)[1];
    double z = (*rstate)[2];

    return isPositionValid(x, y, z);
  }

  bool isEdgeValid(const ob::State *s1,
                   const ob::State *s2,
                   const ob::SpaceInformationPtr &si)
  {
    return si->checkMotion(s1, s2);
  }

  bool plan(double sx, double sy, double sz,
            double gx, double gy, double gz,
            int attempt,
            double timeout = -1)
  {
    if (timeout < 0) timeout = planning_time_;

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, min_x_);
    bounds.setHigh(0, max_x_);
    bounds.setLow(1, min_y_);
    bounds.setHigh(1, max_y_);
    bounds.setLow(2, min_z_);
    bounds.setHigh(2, max_z_);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([this](const ob::State *s) { return this->isStateValid(s); });
    
    auto si = ss.getSpaceInformation();
    si->setStateValidityCheckingResolution(0.005);

    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    ss.setOptimizationObjective(objective);

    auto planner(std::make_shared<og::BITstar>(si));
    
    int adaptive_samples = samples_per_batch_;
    if (attempt > 1) {
      adaptive_samples *= 2;
    }
    
    planner->setSamplesPerBatch(adaptive_samples);
    planner->setPruning(true);
    planner->setJustInTimeSampling(true);
    
    ss.setPlanner(planner);

    // virtual start near robot (with z clamped to bounds)
    double psx = sx;
    double psy = sy;
    double psz = sz;

    if (psz < min_z_) psz = min_z_ + 0.05;
    if (psz > max_z_) psz = max_z_ - 0.05;

    if (!isPositionValid(psx, psy, psz)) {
      RCLCPP_WARN(this->get_logger(),
                  "⚠️ Robot start (%.2f, %.2f, %.2f) is invalid, searching for nearby valid start...",
                  sx, sy, sz);

      bool found = false;
      const double search_radius = 0.8;
      const int num_samples = 16;

      for (int i = 0; i < num_samples && !found; ++i) {
        double angle = (2.0 * M_PI * i) / static_cast<double>(num_samples);
        double nx = sx + search_radius * std::cos(angle);
        double ny = sy + search_radius * std::sin(angle);
        double nz = psz; // use clamped altitude

        if (nx < min_x_ || nx > max_x_ ||
            ny < min_y_ || ny > max_y_ ||
            nz < min_z_ || nz > max_z_) {
          continue;
        }

        if (isPositionValid(nx, ny, nz)) {
          psx = nx;
          psy = ny;
          psz = nz;
          found = true;
        }
      }

      if (!found) {
        RCLCPP_WARN(this->get_logger(),
                    "❌ Could not find a valid planning start near robot pose "
                    "(%.2f, %.2f, %.2f). Skipping this planning cycle.",
                    sx, sy, sz);
        return false;
      }

      RCLCPP_INFO(this->get_logger(),
                  "✅ Using virtual start at (%.2f, %.2f, %.2f) instead of robot pose",
                  psx, psy, psz);
    }

    // Clamp goal into bounds (should already be, but just in case)
    gx = std::min(std::max(gx, min_x_), max_x_);
    gy = std::min(std::max(gy, min_y_), max_y_);
    gz = std::min(std::max(gz, min_z_), max_z_);

    ob::ScopedState<> start(space), goal(space);
    start[0] = psx; start[1] = psy; start[2] = psz;
    goal[0]  = gx;  goal[1]  = gy;  goal[2]  = gz;
    ss.setStartAndGoalStates(start, goal);

    double attempt_timeout = timeout;
    if (attempt > 1) {
      attempt_timeout *= 1.5;
    }
    
    ob::PlannerStatus solved = ss.solve(attempt_timeout);
    
    if (solved)
    {
      ss.simplifySolution();

      og::PathGeometric path = ss.getSolutionPath();
      
      if (isPathValid(path, si)) {
        publishPath(path);
        RCLCPP_INFO(this->get_logger(), "📊 Planned path length: %.2f meters", path.length());
        return true;
      }
      
      RCLCPP_WARN(this->get_logger(), "❌ Solution path validation failed!");
    }
    
    return false;
  }

  bool isPathValid(const og::PathGeometric &path,
                   const ob::SpaceInformationPtr &si)
  {
    og::PathGeometric non_const_path = path;
    const std::vector<ob::State*>& states = non_const_path.getStates();

    if (states.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "🚫 Path too short (states=%zu)", states.size());
      return false;
    }
      
    for (size_t i = 0; i < states.size(); ++i) {
      const auto *s = states[i]->as<ob::RealVectorStateSpace::StateType>();
      if (!isStateValid(s)) {
        RCLCPP_WARN(this->get_logger(), "🚫 Path vertex %zu is in collision or out of bounds", i);
        return false;
      }
    }

    for (size_t i = 0; i + 1 < states.size(); ++i) {
      if (!isEdgeValid(states[i], states[i+1], si)) {
        RCLCPP_WARN(this->get_logger(), "🚫 Path segment %zu -> %zu is in collision", i, i+1);
        return false;
      }
    }
      
    return true;
  }

  void publishPath(const og::PathGeometric &path)
  {
      nav_msgs::msg::Path out;
      out.header.stamp = this->now();
      out.header.frame_id = "map";

      og::PathGeometric non_const_path = path;
      const std::vector<ob::State*>& states = non_const_path.getStates();
      
      size_t target_waypoints = std::min(states.size(), static_cast<size_t>(max_waypoints_));
      size_t step = std::max(1ul, states.size() / target_waypoints);
      
      for (size_t i = 0; i < states.size(); i += step) {
          const auto *s = states[i]->as<ob::RealVectorStateSpace::StateType>();
          geometry_msgs::msg::PoseStamped ps;
          ps.header = out.header;
          ps.pose.position.x = (*s)[0];
          ps.pose.position.y = (*s)[1];
          ps.pose.position.z = (*s)[2];
          ps.pose.orientation.w = 1.0;
          out.poses.push_back(ps);
      }

      if (!states.empty()) {
          const auto *s = states.back()->as<ob::RealVectorStateSpace::StateType>();
          geometry_msgs::msg::PoseStamped ps;
          ps.header = out.header;
          ps.pose.position.x = (*s)[0];
          ps.pose.position.y = (*s)[1];
          ps.pose.position.z = (*s)[2];
          ps.pose.orientation.w = 1.0;
          out.poses.push_back(ps);
      }

      path_pub_->publish(out);
      RCLCPP_INFO(this->get_logger(), "📤 Published path with %zu waypoints", out.poses.size());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 Starting Autonomous BIT* Octomap Planner Node...");
  auto node = std::make_shared<AutonomousRRTStarPlanner>();
  RCLCPP_INFO(rclcpp::get_logger("main"), "🟢 Autonomous planner node is running and ready!");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("main"), "🛑 Autonomous planner node shutting down...");
  rclcpp::shutdown();
  return 0;
}
