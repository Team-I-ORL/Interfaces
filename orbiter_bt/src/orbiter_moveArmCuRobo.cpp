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

    auto pose = bt_string_serialize::stringToPose(goal.value());
    auto request = std::make_shared<orbiter_bt::srv::MoveArm::Request>();
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = "base_link";

    auto result = client->async_send_request(request, std::bind(&MoveArm_CuRobo::result_callback, this, std::placeholders::_1));

}

BT::NodeStatus MoveArm_CuRobo::onRunning()
{
    if (finished)
    {
        if (move_arm_result)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
        finished = false; // reset the flag
    }
    return BT::NodeStatus::RUNNING;
}

void MoveArm_CuRobo::result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result)
{
    auto response = result.get();
    move_arm_result = response->success;
    finished = true;
}