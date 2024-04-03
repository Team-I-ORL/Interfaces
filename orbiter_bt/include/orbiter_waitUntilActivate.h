#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include "std_msgs/msg/string.hpp"
#include "orbiter_bt/msg/fetch.hpp"

class wait_until_activate : public BT::StatefulActionNode
{
    public:
        wait_until_activate(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<orbiter_bt::msg::Fetch>::SharedPtr subscription;
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
    private:
        bool activate = false;
        int id = 0;
        void callback(const orbiter_bt::msg::Fetch::SharedPtr msg);
};