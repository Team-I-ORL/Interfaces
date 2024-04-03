#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "orbiter_bt/srv/move_arm.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

// #include <geometry_msgs/msg/pose.hpp>

class MoveArm : public BT::StatefulActionNode // using async action
{
public:
    MoveArm(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedPtr client;
    bool finished = false;
    bool moveit_result = false;
    void result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result);

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

};