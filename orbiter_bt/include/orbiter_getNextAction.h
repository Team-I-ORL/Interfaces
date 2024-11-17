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
        bool success = false;
        void result_callback(rclcpp::Client<orbiter_bt::srv::NextAction>::SharedFuture result);

        const std::unordered_map<std::string, std::string> item_to_aruco_id_restock= {
            {"obj1", "aruco1"},
            {"obj2", "aruco2"},
            {"obj3", "aruco3"}
        };

        const std::unordered_map<std::string, std::string> item_to_aruco_id_retrieve = {
            {"obj1", "aruco3"},
            {"obj2", "aruco3"},
            {"obj3", "aruco3"}
        };

        // overides for the BT::StatefulActionNode
        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
};