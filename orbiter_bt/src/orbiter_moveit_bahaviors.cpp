#include "orbiter_moveit_behaviors.h"

MoveArm::MoveArm(   const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "MoveArm has been created.");
    action_client_ = rclcpp_action::create_client<moveitInterface>(node_, "move_group");
    moveitDoneFlag = false;
}

BT::PortsList MoveArm::providedPorts()
{
    return {
        // BT::InputPort<std::string>("goal"),
        // BT::OutputPort<std::string>("result")
        };
}

BT::NodeStatus MoveArm::onStart()
{   
    // // Get goal from input port
    // BT::Optional<std::string> goal = getInput<std::string>("goal");
    // if (!goal)
    // {
    //     throw BT::RuntimeError("missing required input [goal]");
    // }
    // RCLCPP_INFO(node_->get_logger(), "Moving to %s", goal.value().c_str());    
    // Send goal to action server
    auto send_goal_options = rclcpp_action::Client<moveitInterface>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MoveArm::move_group_callback, this, std::placeholders::_1);
    
    geometry_msgs::msg::PoseStamped target_pose1; 
    auto goal_msg = moveitInterface::Goal();
    goal_msg.request.group_name = "arm";
    goal_msg.request.goal_constraints[0].position_constraints[0].header.frame_id = "base_link";
    goal_msg.request.goal_constraints[0].position_constraints[0].link_name = "end_effector";
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.5;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.5;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.z = 0.5;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0.0;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0.0;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0.0;
    goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1.0;
    goal_msg.request.goal_constraints[0].position_constraints[0].weight = 1.0;

    // send goal
    action_client_->async_send_goal(goal_msg, send_goal_options);
    moveitDoneFlag = false;
    RCLCPP_INFO(node_->get_logger(), "Goal sent to moveit server");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm::onRunning()
{
    if (moveitDoneFlag)
    {
        RCLCPP_INFO(node_->get_logger(), "Moveit action server has finished");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveArm::move_group_callback(const GoalHandleMove::WrappedResult &result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            moveitDoneFlag = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            moveitDoneFlag = true;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            moveitDoneFlag = true;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            moveitDoneFlag = true;
            break;
    }
}