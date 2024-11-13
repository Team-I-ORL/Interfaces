#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <bt_string_serialize.h>
#include "std_msgs/msg/string.hpp"
class SingFinished : public BT::SyncActionNode
{
    public:
        SingFinished(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
};