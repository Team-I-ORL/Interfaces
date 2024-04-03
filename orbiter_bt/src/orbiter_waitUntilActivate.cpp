#include "orbiter_waitUntilActivate.h"

wait_until_activate::wait_until_activate(const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    subscription = node_->create_subscription<orbiter_bt::msg::Fetch>(
        "fetcher", 10, std::bind(&wait_until_activate::callback, this, std::placeholders::_1));
}

BT::PortsList wait_until_activate::providedPorts()
{
    return {
        BT::OutputPort<std::string>("id")
    };
}

BT::NodeStatus wait_until_activate::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "Waiting for activation");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus wait_until_activate::onRunning()
{
    if (activate)
    {
        RCLCPP_INFO(node_->get_logger(), "Activated");
        setOutput("id", bt_string_serialize::intToString(id));
        return BT::NodeStatus::SUCCESS;
    }
    else
    {   
        RCLCPP_INFO(node_->get_logger(), "Waiting for activation");
        return BT::NodeStatus::RUNNING;
    }
}

void wait_until_activate::callback(const orbiter_bt::msg::Fetch::SharedPtr msg)
{
    if (msg->status == true)
    {
        RCLCPP_INFO(node_->get_logger(), "Activated");
        activate = true;
        id = msg->inventory_id;
    }
}
