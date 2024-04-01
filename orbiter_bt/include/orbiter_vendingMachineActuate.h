#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ims_interfaces/srv/vending_machine.hpp"
#include "bt_string_serialize.h"
class ActuateVendingMachine : public BT::StatefulActionNode
{
    public:
        ActuateVendingMachine(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);

        rclcpp::Node::SharedPtr node_;

        rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedPtr vending_machine_client;
        
        static BT::PortsList providedPorts();

        bool finished;
        bool successful;

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
        
        void result_callback(rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedFuture result);
};