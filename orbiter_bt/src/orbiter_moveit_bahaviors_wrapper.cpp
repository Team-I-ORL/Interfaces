#include "orbiter_moveit_behaviors_wrapper.h"
MoveArm_Wrapper::MoveArm_Wrapper(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::MoveArm>("move_arm"))
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm Wrapper Version Creating");
}

BT::PortsList MoveArm_Wrapper::providedPorts()
{
    return {
        BT::InputPort<std::string>("arm_goal"),
        };
}  

BT::NodeStatus MoveArm_Wrapper::onStart()
{
    auto goal = getInput<std::string>("arm_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [arm_goal]");
        return BT::NodeStatus::FAILURE;
    }

    std::vector<double> goalVec = bt_string_serialize::stringToVector(goal.value());
    auto request = std::make_shared<orbiter_bt::srv::MoveArm::Request>();
    geometry_msgs::msg::Pose targetPose;
    targetPose.position.x = goalVec[0];
    targetPose.position.y = goalVec[1];
    targetPose.position.z = goalVec[2];
    targetPose.orientation.x = 0.0;
    targetPose.orientation.y = 0.0;
    targetPose.orientation.z = 0.0;
    targetPose.orientation.w = 1.0;
    request->target_pose = targetPose;

    RCLCPP_INFO(node_->get_logger(), "Goal Created At: x: %f, y: %f, z: %f", targetPose.position.x, targetPose.position.y, targetPose.position.z);
    auto result = client->async_send_request(request, std::bind(&MoveArm_Wrapper::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm_Wrapper::onRunning()
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

void MoveArm_Wrapper::result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result)
{
    finished = true;
    auto response = result.get();
    moveit_result = response->success;
}