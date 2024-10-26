#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "orbiter_bt/srv/move_head.hpp"
#include <bt_string_serialize.h>

class MoveHead : public BT::StatefulActionNode // using async action
{
    public:
        MoveHead(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<orbiter_bt::srv::MoveHead>::SharedPtr client;
        bool finished = false;
        bool move_head_result = false;
        rclcpp::Time start_time;
        void result_callback(rclcpp::Client<orbiter_bt::srv::MoveHead>::SharedFuture result);

        // overides for the BT::StatefulActionNode
        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
};