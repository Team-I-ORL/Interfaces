#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/srv/aruco_pose.hpp"
#include "bt_string_serialize.h"

class arucoArmPose : public BT::StatefulActionNode{
public:
    arucoArmPose(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<ros2_aruco_interfaces::srv::ArucoPose>::SharedPtr aruco_pose_client;
    
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    bool finished = false; 
    
    void result_callback(rclcpp::Client<ros2_aruco_interfaces::srv::ArucoPose>::SharedFuture result);

};