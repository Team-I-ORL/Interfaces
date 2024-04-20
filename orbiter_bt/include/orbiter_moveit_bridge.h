#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <bt_string_serialize.h>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class Moveit_Bridge : public BT::StatefulActionNode{
    public:
        Moveit_Bridge(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
        rclcpp::Node::SharedPtr node_;
        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override; 
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
        bool finished = false;
        bool moveit_result = false;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
        void status_callback(const std_msgs::msg::String::SharedPtr msg);
};
