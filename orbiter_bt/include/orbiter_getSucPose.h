#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "orbiter_bt/srv/get_suc_pose.hpp"
#include "bt_string_serialize.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

class GetItemPose : public BT::StatefulActionNode{
public:
    GetItemPose(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedPtr client;
    bool finished = false;

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    void result_callback(rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedFuture result);
};

