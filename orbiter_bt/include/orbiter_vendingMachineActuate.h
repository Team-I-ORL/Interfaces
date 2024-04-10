#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
// #include "ims_interfaces/srv/vending_machine.hpp"
#include "bt_string_serialize.h"
#include "std_msgs/msg/string.hpp"

class ActuateVendingMachine : public BT::StatefulActionNode
{
    public:
        ActuateVendingMachine(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);

        rclcpp::Node::SharedPtr node_;

        // rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedPtr vending_machine_client;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vending_machine_publisher;
        std::chrono::time_point<std::chrono::system_clock> end_time;
        static BT::PortsList providedPorts();

        // bool finished;
        // bool successful;

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
        
        // void result_callback(rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedFuture result);
};