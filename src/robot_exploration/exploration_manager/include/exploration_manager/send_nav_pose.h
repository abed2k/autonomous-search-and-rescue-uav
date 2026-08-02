#ifndef __SEND_NAV_POSE__
#define __SEND_NAV_POSE__

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace BT;

class SendNavPose : public BT::SyncActionNode
{
public:
    SendNavPose(const std::string& name,
                const BT::NodeConfig &config,
                rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts() {
        return {};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    geometry_msgs::msg::Pose temp_nav_pose_;
    double distance_to_nav_target_, distance_to_object_pose_, angle_;
    bool updated_nav_;

    double rob_yaw_, nav_yaw_;
};

#endif

