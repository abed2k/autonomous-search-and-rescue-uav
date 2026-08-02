#ifndef __COLLECT_OBJ_POSE__
#define __COLLECT_OBJ_POSE__

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include <exploration_manager/SharedClass.h>
#include <chrono>

using namespace BT;
using namespace std::chrono_literals;

class CollectObjectPose : public BT::SyncActionNode
{
public:
    CollectObjectPose(const std::string& name,
                      const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    // Commented out: service dependency missing
    // rclcpp::Client<object_detection_srvs::srv::GetObjectsInfo>::SharedPtr get_objects_info_srv_;
    // object_detection_srvs::srv::GetObjectsInfo::Request::SharedPtr get_objects_req_;
    // rclcpp::Client<object_detection_srvs::srv::GetObjectsInfo>::SharedFuture get_objects_fut_;
    // object_detection_srvs::srv::GetObjectsInfo::Response::SharedPtr get_objects_res_;

    double angle_, distance_to_object_pose_;
};

#endif
