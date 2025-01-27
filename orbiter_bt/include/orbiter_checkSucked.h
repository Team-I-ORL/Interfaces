#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include "std_msgs/msg/bool.hpp"

class CheckSucked : public BT::ConditionNode{
    public:
        CheckSucked(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
        static BT::PortsList providedPorts() {return {};};
        bool sucked = false;
        BT::NodeStatus tick() override;
        void callback(const std_msgs::msg::Bool::SharedPtr msg){
            sucked = msg->data;
        }
};