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

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTStarOctomapPlanner : public rclcpp::Node
{
public:
  RRTStarOctomapPlanner()
  : Node("rrtstar_octomap_planner"),
    octree_(nullptr),
    have_start_(false),
    has_goal_(false),
    planning_in_progress_(false)
  {
    // params
    this->declare_parameter<std::string>("vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
    this->declare_parameter<std::string>("octomap_topic", "/octomap_full");
    this->declare_parameter<std::string>("goal_topic", "/planner_goal");
    this->declare_parameter<std::string>("planned_path_topic", "/planned_path");
    this->declare_parameter<double>("planning_time", 3.0);  // Increased for complex paths
    this->declare_parameter<double>("collision_radius", 0.8);  // Increased for safety
    this->declare_parameter<double>("min_x", -50.0);
    this->declare_parameter<double>("max_x",  50.0);
    this->declare_parameter<double>("min_y", -50.0);
    this->declare_parameter<double>("max_y",  50.0);
    this->declare_parameter<double>("min_z", -5.0);
    this->declare_parameter<double>("max_z",  50.0);
    
    // BIT* specific parameters
    this->declare_parameter<int>("samples_per_batch", 200);  // Increased for better exploration
    this->declare_parameter<int>("max_attempts", 5);  // More attempts for complex paths
    this->declare_parameter<double>("safety_margin", 0.5);  // Increased for safety
    this->declare_parameter<int>("max_waypoints", 25);  // More waypoints for smoother paths

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

    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();

    // subscribers / publishers
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_topic_, qos_profile,
      std::bind(&RRTStarOctomapPlanner::px4OdomCallback, this, std::placeholders::_1));

    auto octomap_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    octomap_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    octomap_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, octomap_qos,
      std::bind(&RRTStarOctomapPlanner::octomapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10,
      std::bind(&RRTStarOctomapPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(planned_path_topic_, 10);

    // Continuous planning timer for real-time exploration
    planning_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),  // Replan every 500ms
      std::bind(&RRTStarOctomapPlanner::planningLoop, this));

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "BIT* Octomap Planner Node Started!");
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "Vehicle odom topic: %s", vehicle_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Octomap topic: %s", octomap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Goal topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Path topic: %s", planned_path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning time: %.1f seconds", planning_time_);
    RCLCPP_INFO(this->get_logger(), "Max attempts: %d", max_attempts_);
    RCLCPP_INFO(this->get_logger(), "Collision radius: %.1f meters", collision_radius_);
    RCLCPP_INFO(this->get_logger(), "Safety margin: %.1f meters", safety_margin_);
    RCLCPP_INFO(this->get_logger(), "Max waypoints: %d", max_waypoints_);
    RCLCPP_INFO(this->get_logger(), "Workspace bounds: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]", 
                min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    RCLCPP_INFO(this->get_logger(), "Real-time exploration mode: ACTIVE");
    RCLCPP_INFO(this->get_logger(), "Waiting for odometry, octomap, and goals...");
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

  // Add destructor for proper cleanup
  ~RRTStarOctomapPlanner() {
    // Signal the planning thread to stop
    planning_in_progress_ = true;
    
    // Wait for the planning thread to finish
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
    
    RCLCPP_INFO(this->get_logger(), "🧹 Planner node cleaned up");
  }

private:
  // --- subscribers / publishers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // parameters
  std::string vehicle_topic_, octomap_topic_, goal_topic_, planned_path_topic_;
  double planning_time_;
  double collision_radius_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  int samples_per_batch_, max_attempts_, max_waypoints_;
  double safety_margin_;

  // state
  std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex start_mutex_;
  double start_x_ = 0.0, start_y_ = 0.0, start_z_ = 0.0;
  double goal_x_ = 0.0, goal_y_ = 0.0, goal_z_ = 0.0;
  bool have_start_;
  bool has_goal_;
  std::atomic<bool> planning_in_progress_;
  std::thread planning_thread_;

  // ---- callbacks ----
  void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    // PX4 publishes NED, convert to ENU for planning
    start_x_ = msg->position[0];   // NED.y -> ENU.x
    start_y_ = -msg->position[1];  // NED.x -> ENU.y  
    start_z_ = -msg->position[2];  // NED.z -> ENU.z
    
    if (!have_start_) {
      have_start_ = true;
      RCLCPP_INFO(this->get_logger(), "✓ Received first odometry: ENU(%.2f, %.2f, %.2f)", 
                  start_x_, start_y_, start_z_);
    }
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(octree_mutex_);
    
    RCLCPP_DEBUG(this->get_logger(), "Processing octomap update...");
    
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
    
    if (tree && tree->getTreeType() == "OcTree")
    {
        octomap::OcTree* dt = dynamic_cast<octomap::OcTree*>(tree.release());
        octree_.reset(dt);
        
        int occupied_nodes = 0;
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied_nodes++;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "🗺️ Octomap updated - Resolution: %.3f, Occupied nodes: %d", 
                   octree_->getResolution(), occupied_nodes);
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

    goal_x_ = msg->point.x;
    goal_y_ = msg->point.y;
    goal_z_ = msg->point.z;
    has_goal_ = true;

    RCLCPP_INFO(this->get_logger(), "🎯 Goal set to: ENU(%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_z_);
    RCLCPP_INFO(this->get_logger(), "🚀 Beginning real-time navigation with mapping...");
    
    // Trigger immediate planning
    planningLoop();
  }

  // Continuous planning loop for real-time exploration
  void planningLoop()
  {
    if (!has_goal_ || !have_start_ || !octree_ || planning_in_progress_) {
      return;
    }

    // Check if we've reached the goal - REDUCED from 2.0 to 0.8 for better precision
    double distance_to_goal = sqrt(pow(goal_x_ - start_x_, 2) + 
                                  pow(goal_y_ - start_y_, 2) + 
                                  pow(goal_z_ - start_z_, 2));
    
    if (distance_to_goal < 0.8) { // Reduced for better goal precision
      RCLCPP_INFO(this->get_logger(), "✅ Reached goal!");
      has_goal_ = false;
      return;
    }

    planning_in_progress_ = true;

    // Plan in a separate thread to avoid blocking
    if (!planning_thread_.joinable()) {
      planning_thread_ = std::thread([this]() {
        this->continuousPlan();
      });
    }
  }

  void continuousPlan()
  {
    std::lock_guard<std::mutex> lk(start_mutex_);
    
    // Check if we should stop planning
    if (planning_in_progress_ == false) {
      return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "🔄 Replanning... Current pos: ENU(%.2f, %.2f, %.2f)", 
                 start_x_, start_y_, start_z_);

    // Use adaptive planning time based on path complexity
    double straight_line_dist = sqrt(pow(goal_x_ - start_x_, 2) + 
                                    pow(goal_y_ - start_y_, 2) + 
                                    pow(goal_z_ - start_z_, 2));
    
    // More time for complex paths (U-turns, large detours)
    double adaptive_timeout = std::min(4.0, std::max(1.5, straight_line_dist / 8.0));
    
    // Try planning with adaptive parameters
    for (int attempt = 1; attempt <= max_attempts_; attempt++) {
        // Check if we should stop before each attempt
        if (planning_in_progress_ == false) {
            break;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Replanning attempt %d/%d", attempt, max_attempts_);
        
        if (plan(start_x_, start_y_, start_z_, goal_x_, goal_y_, goal_z_, attempt, adaptive_timeout)) {
            RCLCPP_DEBUG(this->get_logger(), "✅ Replanning successful");
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

  // ---------- OMPL stuff ----------
  bool isStateValid(const ob::State *state)
  {
    const auto *rstate = state->as<ob::RealVectorStateSpace::StateType>();
    double x = (*rstate)[0];
    double y = (*rstate)[1];
    double z = (*rstate)[2];

    std::lock_guard<std::mutex> lk(octree_mutex_);
    if (!octree_) return true;

    // More robust collision checking with safety margin
    double resolution = octree_->getResolution();
    
    // Check exact position first
    octomap::OcTreeNode* exact_node = octree_->search(x, y, z);
    if (exact_node && octree_->isNodeOccupied(exact_node)) {
        return false;
    }

    // Enhanced collision checking with more samples
    int samples = 8; // Increased sampling for better safety
    for (int i = -samples; i <= samples; i++) {
        for (int j = -samples; j <= samples; j++) {
            for (int k = -samples; k <= samples; k++) {
                double dx = (i * collision_radius_) / samples;
                double dy = (j * collision_radius_) / samples;
                double dz = (k * collision_radius_) / samples;
                
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist > collision_radius_ + safety_margin_) continue;
                
                double qx = x + dx;
                double qy = y + dy;
                double qz = z + dz;
                
                octomap::OcTreeNode* node = octree_->search(qx, qy, qz);
                if (node && octree_->isNodeOccupied(node)) {
                    return false;
                }
            }
        }
    }
    return true;
  }

  bool plan(double sx, double sy, double sz, double gx, double gy, double gz, int attempt, double timeout = -1)
  {
    if (timeout < 0) timeout = planning_time_;

    RCLCPP_DEBUG(this->get_logger(), "🔄 Planning from ENU(%.2f, %.2f, %.2f) to ENU(%.2f, %.2f, %.2f) - Attempt %d",
                sx, sy, sz, gx, gy, gz, attempt);

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
    si->setStateValidityCheckingResolution(0.01); // More precise checking

    // BIT* with optimization objective
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    ss.setOptimizationObjective(objective);

    auto planner(std::make_shared<og::BITstar>(si));
    
    // Configure BIT* parameters for better exploration
    int adaptive_samples = samples_per_batch_;
    if (attempt > 1) {
        adaptive_samples *= 2; // Double samples for retries (better for U-turns)
    }
    
    planner->setSamplesPerBatch(adaptive_samples);
    planner->setPruning(true);
    planner->setJustInTimeSampling(true); // Better for real-time
    
    ss.setPlanner(planner);

    ob::ScopedState<> start(space), goal(space);
    start[0] = sx; start[1] = sy; start[2] = sz;
    goal[0]  = gx;  goal[1]  = gy;  goal[2]  = gz;
    ss.setStartAndGoalStates(start, goal);

    // Adaptive timeout - more time for complex paths
    double attempt_timeout = timeout;
    if (attempt > 1) {
        attempt_timeout *= 1.5; // More time for retries
    }
    
    RCLCPP_DEBUG(this->get_logger(), "⏱️  Planning timeout: %.1fs, Samples: %d", attempt_timeout, adaptive_samples);
    
    ob::PlannerStatus solved = ss.solve(attempt_timeout);
    
    if (solved)
    {
        // Validate the path before publishing
        og::PathGeometric path = ss.getSolutionPath();
        
        if (isPathValid(path)) {
            publishPath(path);
            RCLCPP_INFO(this->get_logger(), "📊 Planned path length: %.2f meters", path.length());
            return true;
        }
        
        RCLCPP_WARN(this->get_logger(), "❌ Solution path validation failed!");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "❌ Planning attempt %d failed", attempt);
    }
    
    return false;
  }

  // Validate that the entire path is collision-free
  bool isPathValid(const og::PathGeometric &path)
  {
      // Create a non-const copy to access getStates()
      og::PathGeometric non_const_path = path;
      const std::vector<ob::State*>& states = non_const_path.getStates();
      
      for (size_t i = 0; i < states.size(); ++i) {
          const auto *s = states[i]->as<ob::RealVectorStateSpace::StateType>();
          if (!isStateValid(s)) {
              RCLCPP_WARN(this->get_logger(), "🚫 Path validation failed at waypoint %zu", i);
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

      // Create a non-const copy to work around OMPL API limitation
      og::PathGeometric non_const_path = path;
      const std::vector<ob::State*>& states = non_const_path.getStates();
      
      // Use more waypoints for smoother navigation (increased from 15 to max_waypoints_)
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

      // Always include the final waypoint
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
  RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 Starting BIT* Octomap Planner Node...");
  auto node = std::make_shared<RRTStarOctomapPlanner>();
  RCLCPP_INFO(rclcpp::get_logger("main"), "🟢 Planner node is running and ready!");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("main"), "🛑 Planner node shutting down...");
  rclcpp::shutdown();
  return 0;
}