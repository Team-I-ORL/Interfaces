#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include <string>
#include <std_msgs/msg/string.hpp>

class speak : public BT::SyncActionNode{
    public:
    speak(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};