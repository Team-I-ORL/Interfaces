#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "orbiter_bt/srv/next_action.hpp"
#include <bt_string_serialize.h>

class GetNextAction : public BT::StatefulActionNode // using async action
{
    public:
        GetNextAction(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<orbiter_bt::srv::NextAction>::SharedPtr client;
        bool finished = false;
        void result_callback(rclcpp::Client<orbiter_bt::srv::NextAction>::SharedFuture result);

        // overides for the BT::StatefulActionNode
        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
};