#include <exploration_manager/collect_object_pose.h>

CollectObjectPose::CollectObjectPose(const std::string& name,
                                     const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    // No external service needed
    RCLCPP_INFO(node_->get_logger(), "CollectObjectPose: Service dependency removed");
}

BT::NodeStatus CollectObjectPose::tick()
{
    RCLCPP_INFO(node_->get_logger(), "CollectObjectPose");

    if(bt_data_->current_task >= static_cast<int>(bt_data_->tasks.size()))
        return BT::NodeStatus::FAILURE;

    // Simulate object detection by assuming object exists at some offset (or keep last known pose)
    if(bt_data_->known_object_pose)
    {
        RCLCPP_INFO(node_->get_logger(), "CollectObjectPose: Object already known");
    }
    else
    {
        // Placeholder object pose — user can replace with real detection later
        bt_data_->object_pose.transform.translation.x = bt_data_->last_robot_pose.transform.translation.x + 1.0;
        bt_data_->object_pose.transform.translation.y = bt_data_->last_robot_pose.transform.translation.y + 0.5;
        bt_data_->object_pose.transform.translation.z = bt_data_->last_robot_pose.transform.translation.z;

        bt_data_->need_exploration = false;
        bt_data_->known_object_pose = true;

        RCLCPP_INFO(node_->get_logger(), "CollectObjectPose: Simulated object pose set at (%f, %f, %f)",
                    bt_data_->object_pose.transform.translation.x,
                    bt_data_->object_pose.transform.translation.y,
                    bt_data_->object_pose.transform.translation.z);
    }

    return BT::NodeStatus::SUCCESS;
}
