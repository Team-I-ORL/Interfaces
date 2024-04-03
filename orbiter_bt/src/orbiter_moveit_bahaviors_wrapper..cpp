#include "orbiter_moveit_behaviors_wrapper.h"
MoveArm::MoveArm(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::MoveArm>("move_arm"))
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm Wrapper Version Creating");
}

BT::PortsList MoveArm::providedPorts()
{
    return {
        BT::InputPort<std::string>("goal"),
        };
}  

BT::NodeStatus MoveArm::onStart()
{
    auto goal = getInput<std::string>("goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [goal]");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<orbiter_bt::srv::MoveArm::Request>();
    geometry_msgs::msg::Pose targetPose;
    targetPose.position.x = 0.5;
    targetPose.position.y = 0.5;
    targetPose.position.z = 0.5;
    targetPose.orientation.x = 0.0;
    targetPose.orientation.y = 0.0;
    targetPose.orientation.z = 0.0;
    targetPose.orientation.w = 1.0;
    request->target_pose = targetPose;

    RCLCPP_INFO(node_->get_logger(), "Goal Created At: x: %f, y: %f, z: %f", targetPose.position.x, targetPose.position.y, targetPose.position.z);
    auto result = client->async_send_request(request, std::bind(&MoveArm::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm::onRunning()
{
    if (finished)
    {
        if (moveit_result == true)
        {
            RCLCPP_INFO(node_->get_logger(), "Move Arm Succeeded");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Move Arm Failed");
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void MoveArm::result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result)
{
    finished = true;
    auto response = result.get();
    moveit_result = response->success;
}