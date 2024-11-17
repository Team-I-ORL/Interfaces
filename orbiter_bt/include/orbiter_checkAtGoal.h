#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_string_serialize.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CheckAtGoal : public BT::ConditionNode{
    public:
        CheckAtGoal(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);
        rclcpp::Node::SharedPtr node_;
        static BT::PortsList providedPorts();
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf_listener;
        BT::NodeStatus tick() override;
};