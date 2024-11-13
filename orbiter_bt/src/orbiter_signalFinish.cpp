#include "orbiter_signalFinish.h"

SingFinished::SingFinished(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "SingFinished has been created.");
    pub = node_->create_publisher<std_msgs::msg::String>("request_clear_restocking_item", 5);
}

BT::PortsList SingFinished::providedPorts()
{
    return {
        BT::InputPort<std::string>("itemname")
    };
}

BT::NodeStatus SingFinished::tick()
{
    std_msgs::msg::String msg;
    BT::Optional<std::string> itemname = getInput<std::string>("itemname");
    if (!itemname)
    {
        throw BT::RuntimeError("missing required input [itemname]");
    }
    msg.data = itemname.value();
    pub->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Request to clear restocking item %s", itemname.value().c_str());
    return BT::NodeStatus::SUCCESS;
}