#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"

class checkArmGoal : public BT::ConditionNode{
    checkArmGoal(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};