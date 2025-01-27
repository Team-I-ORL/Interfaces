#include "orbiter_checkSucked.h"

CheckSucked::CheckSucked(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config),
    node_(node),
    sub_(node_->create_subscription<std_msgs::msg::Bool>("/suction_status", 10, std::bind(&CheckSucked::callback, this, std::placeholders::_1)))
{
    RCLCPP_INFO(node_->get_logger(), "CheckSucked has been created.");
}

BT::NodeStatus CheckSucked::tick()
{
    if (sucked)
    {
        RCLCPP_INFO(node_->get_logger(), "Sucked!");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Not sucked!!");
        return BT::NodeStatus::FAILURE;
    }
}