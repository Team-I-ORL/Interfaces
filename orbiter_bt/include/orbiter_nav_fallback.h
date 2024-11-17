#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <bt_string_serialize.h>
#include "orbiter_bt/srv/cmd_vel_dist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/bool.hpp"

class NavFallback : public BT::StatefulActionNode // using async action
{
    public:
        NavFallback(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<orbiter_bt::srv::CmdVelDist>::SharedPtr client;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr moved_pub;

        bool finished = false;
        bool success = false;
        void result_callback(rclcpp::Client<orbiter_bt::srv::CmdVelDist>::SharedFuture result);

        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
};