#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <tf2_ros/transform_listener.h>

class MoveArm : public BT::StatefulActionNode // using async action
{
public:
    MoveArm(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;

    const std::string PLANNING_GROUP = "manipulator";
    const std::string BASE_LINK = "base_link";
    std::string EE_LINK;

    geometry_msgs::msg::PoseStamped target_pose;
    moveit::planning_interface::MoveGroupInterface move_group_interface; 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener;

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};


    bool goalChecker();

};