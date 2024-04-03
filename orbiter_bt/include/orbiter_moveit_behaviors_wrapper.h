#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "orbiter_bt/srv/move_arm.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <bt_string_serialize.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/buffer_core.h>

// #include <geometry_msgs/msg/pose.hpp>

class MoveArm_Wrapper : public BT::StatefulActionNode // using async action
{
public:
    MoveArm_Wrapper(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedPtr client;
    bool finished = false;
    bool moveit_result = false;
    void result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener;

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

};