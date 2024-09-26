#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include "random"
#include "bt_string_serialize.h"
class RandomizeYaw : public BT::SyncActionNode{
    public:
    RandomizeYaw(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    bool leftAlready = false;
};