#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include <geometry_msgs/msg/pose.hpp>

class MoveArm : public BT::StatefulActionNode // using async action
{
public:
    MoveArm(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);

    using moveitInterface = moveit_msgs::action::MoveGroup;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<moveitInterface>;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<moveitInterface>::SharedPtr action_client_;
    bool moveitDoneFlag;
    void move_group_callback(const GoalHandleMove::WrappedResult &result);

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

};