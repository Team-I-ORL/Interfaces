#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
class clearInputs : public BT::SyncActionNode
{
public:
    clearInputs(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};