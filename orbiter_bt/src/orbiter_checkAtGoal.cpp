#include "orbiter_checkAtGoal.h"

CheckAtGoal::CheckAtGoal(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config),
    node_(node),
    tfBuffer(node_->get_clock()),
    tf_listener(tfBuffer)
{
    RCLCPP_INFO(node_->get_logger(), "CheckAtGoal has been created.");
}

BT::PortsList CheckAtGoal::providedPorts()
{
    return {
        BT::InputPort<std::string>("nav_goal"),
    };
}

BT::NodeStatus CheckAtGoal::tick()
{
    auto goal = getInput<std::string>("nav_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_goal]");
        return BT::NodeStatus::FAILURE;
    }
    auto goal_vec = bt_string_serialize::stringToVector(goal.value());
    auto goal_x = goal_vec[0];
    auto goal_y = goal_vec[1];
    auto goal_theta = goal_vec[2];
    RCLCPP_INFO(node_->get_logger(), "Checking if at goal: X: %f, Y: %f, theta: %f", goal_x, goal_y, goal_theta);

    // Get current pose
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }
    auto current_x = transformStamped.transform.translation.x;
    auto current_y = transformStamped.transform.translation.y;
    RCLCPP_INFO(node_->get_logger(), "Current Pose: X: %f, Y: %f", current_x, current_y);

    // Check if at goal
    const float goal_tolerance = 0.2;
    if (std::abs(current_x - goal_x) < goal_tolerance && std::abs(current_y - goal_y) < goal_tolerance)
    {
        RCLCPP_INFO(node_->get_logger(), "At goal!");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Not at goal!!");
        RCLCPP_INFO(node_->get_logger(), "X_Diff: %f, Y_Diff: %f", std::abs(current_x - goal_x), std::abs(current_y - goal_y));
        return BT::NodeStatus::FAILURE;
    }
}