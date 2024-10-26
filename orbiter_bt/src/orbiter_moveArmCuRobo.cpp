#include "orbiter_moveArmCuRobo.h"

MoveArm_CuRobo::MoveArm_CuRobo(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::MoveArm>("target_pose"))
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm CuRobo Version Creating");
}

BT::PortsList MoveArm_CuRobo::providedPorts()
{
    return {
        BT::InputPort<std::string>("arm_goal"), // arm goal in map frame
        BT::InputPort<std::string>("traj_type"), // arm name
        };
}

BT::NodeStatus MoveArm_CuRobo::onStart()
{
    auto goal = getInput<std::string>("arm_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [arm_goal]");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Pose_Serializing: %s", goal.value().c_str());
    auto pose = bt_string_serialize::stringToPose(goal.value());
    RCLCPP_INFO(node_->get_logger(), "Pose_Sending %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
    auto request = std::make_shared<orbiter_bt::srv::MoveArm::Request>();
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = "base_link";
    RCLCPP_INFO(node_->get_logger(), "Pose_Stamped_Sending %f, %f, %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
    request->target_pose = pose_stamped;
    int traj_type = bt_string_serialize::stringToInt(getInput<std::string>("traj_type").value());
    RCLCPP_INFO(node_->get_logger(), "Traj_Type_Sending %d", traj_type);
    request->traj_type.data = traj_type;
    auto result = client->async_send_request(request, std::bind(&MoveArm_CuRobo::result_callback, this, std::placeholders::_1));

}

BT::NodeStatus MoveArm_CuRobo::onRunning()
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm Running!");
    if (finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Move Arm finishec!");
        if (move_arm_result)
        {
            RCLCPP_INFO(node_->get_logger(), "Move Arm SUCCESS!");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Move Arm FAILED!");
            return BT::NodeStatus::FAILURE;
        }
        finished = false; // reset the flag
    }
    return BT::NodeStatus::RUNNING;
}

void MoveArm_CuRobo::result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm result get!");
    auto response = result.get();
    move_arm_result = response->success;
    finished = true;
}