#include "orbiter_speak.h"

speak :: speak(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
    pub_ = node_->create_publisher<std_msgs::msg::String>("orbiter_speak", 1);
}

BT::PortsList speak :: providedPorts()
{
    return{ BT::InputPort<std::string>("sentence") };
}

BT::NodeStatus speak :: tick()
{
    std::string sentence;
    if (!getInput<std::string>("sentence", sentence))
    {
        throw BT::RuntimeError("missing required input [sentence]: ", sentence);
    };

    std_msgs::msg::String msg;
    msg.data = sentence;
    pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
}