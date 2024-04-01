#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "orbiter_bt/srv/ims.hpp"
#include "ims_interfaces/srv/item.hpp"
#include "bt_string_serialize.h"
class GetItemInfo : public BT::StatefulActionNode{
public:
    GetItemInfo(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    // rclcpp::Client<orbiter_bt::srv::Ims>::SharedPtr client;
    rclcpp::Client<ims_interfaces::srv::Item>::SharedPtr item_client;
    bool finished = false;

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    void result_callback(rclcpp::Client<ims_interfaces::srv::Item>::SharedFuture result);
};