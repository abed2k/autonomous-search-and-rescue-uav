#include <exploration_manager/send_nav_pose.h>

SendNavPose::SendNavPose(const std::string& name,
                         const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    node_->declare_parameter("robot_exploration.distance_to_object_pose", 0.85);
    distance_to_object_pose_ = node_->get_parameter("robot_exploration.distance_to_object_pose").as_double();

    updated_nav_ = false;
}

BT::NodeStatus SendNavPose::tick()
{
    RCLCPP_DEBUG(node_->get_logger(), "SendNavPose");

    // 1. Update the intermediate target (based on the task)
    if(bt_data_->need_exploration || bt_data_->tasks[bt_data_->current_task].id == 1)
    {
        // If Task is "Reach Target" and you know the object --> Nav Target is based on the Object
        if(bt_data_->known_object_pose)
        {
            double dx = bt_data_->object_pose.transform.translation.x - bt_data_->last_robot_pose.transform.translation.x;
            double dy = bt_data_->object_pose.transform.translation.y - bt_data_->last_robot_pose.transform.translation.y;

            angle_ = atan2(dy, dx);

            // Transform into [-pi, pi]
            if(angle_ > M_PI)
                angle_ -= 2*M_PI*(1.0 + std::floor(angle_/(2*M_PI)));
            else if(angle_ < -M_PI)
                angle_ += 2*M_PI*(1.0 + std::floor(-angle_/(2*M_PI)));

            bt_data_->locomotion_target.position.x = bt_data_->object_pose.transform.translation.x - distance_to_object_pose_*cos(angle_);
            bt_data_->locomotion_target.position.y = bt_data_->object_pose.transform.translation.y - distance_to_object_pose_*sin(angle_);

            // Orientation to face the object
            bt_data_->locomotion_target.orientation.x = 0;
            bt_data_->locomotion_target.orientation.y = 0;
            bt_data_->locomotion_target.orientation.z = sin(angle_/2.0);
            bt_data_->locomotion_target.orientation.w = cos(angle_/2.0);
        }
    }
    else if(bt_data_->tasks[bt_data_->current_task].id == 2) // Inspection Task
    {
        if(bt_data_->tasks[bt_data_->current_task].getNavTargetsNumber() == 0)
            return BT::NodeStatus::FAILURE;

        temp_nav_pose_ = bt_data_->tasks[bt_data_->current_task].getLastNavTarget();

        if(temp_nav_pose_ != bt_data_->locomotion_target)
        {
            RCLCPP_INFO(node_->get_logger(), "New goal: %f %f --> %f %f", 
                        bt_data_->locomotion_target.position.x, bt_data_->locomotion_target.position.y,
                        temp_nav_pose_.position.x, temp_nav_pose_.position.y);

            bt_data_->locomotion_target = temp_nav_pose_;
            updated_nav_ = true;
        }

        // Compute distance to previous nav target
        distance_to_nav_target_ = pow(bt_data_->last_robot_pose.transform.translation.x - bt_data_->locomotion_target.position.x, 2) +
                                  pow(bt_data_->last_robot_pose.transform.translation.y - bt_data_->locomotion_target.position.y, 2);

        // Yaw error
        double rob_yaw = atan2(2.0*(bt_data_->last_robot_pose.transform.rotation.x*bt_data_->last_robot_pose.transform.rotation.y +
                                    bt_data_->last_robot_pose.transform.rotation.w*bt_data_->last_robot_pose.transform.rotation.z),
                               1.0 - 2.0*(pow(bt_data_->last_robot_pose.transform.rotation.y,2) +
                                           pow(bt_data_->last_robot_pose.transform.rotation.z,2)));

        double nav_yaw = atan2(2.0*(bt_data_->locomotion_target.orientation.x*bt_data_->locomotion_target.orientation.y + 
                                    bt_data_->locomotion_target.orientation.w*bt_data_->locomotion_target.orientation.z),
                               1.0 - 2.0*(pow(bt_data_->locomotion_target.orientation.y,2) +
                                           pow(bt_data_->locomotion_target.orientation.z,2)));

        angle_ = fabs(nav_yaw - rob_yaw);
        if(angle_ > 2*M_PI) angle_ -= 2*M_PI;
        if(angle_ > M_PI) angle_ = 2*M_PI - angle_;

        // Check if we reached inspection target
        if(!bt_data_->is_driving && distance_to_nav_target_ < 0.25*0.25 && angle_ < 0.20)
        {
            RCLCPP_INFO(node_->get_logger(), "Inspection Target Reached");
            bt_data_->acquire_image = true;
            return BT::NodeStatus::FAILURE;
        }

        if(!bt_data_->is_driving)
            RCLCPP_INFO(node_->get_logger(), "Inspection Target (%d/%d) to better define (sqr distance: %f, AngR: %f, AngT: %f)", 
                        bt_data_->tasks[bt_data_->current_task].inspection_steps, INSPECTION_IMAGES,
                        distance_to_nav_target_, rob_yaw, nav_yaw);

        // If previous target is almost the same as the new one (<10cm)
        if(pow(bt_data_->locomotion_target.position.x - temp_nav_pose_.position.x, 2) +
           pow(bt_data_->locomotion_target.position.y - temp_nav_pose_.position.y, 2) < 0.01f)
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Locomotion target: %f %f",
                bt_data_->locomotion_target.position.x,
                bt_data_->locomotion_target.position.y);

    updated_nav_ = false;
    return BT::NodeStatus::SUCCESS;
}
