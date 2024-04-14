#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include <string>

class repetition_manager : public BT::SyncActionNode{
    // This node will return SUCCESS after num_repeats iterations, use with a fallback node
    public:
    repetition_manager(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};