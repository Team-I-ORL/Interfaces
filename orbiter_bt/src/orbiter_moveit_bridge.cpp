#include "orbiter_moveit_bridge.h"

Moveit_Bridge::Moveit_Bridge(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("moveit_goal", 10);
    status_subscriber_ = node_->create_subscription<std_msgs::msg::String>("moveit_status", 10, std::bind(&Moveit_Bridge::status_callback, this, std::placeholders::_1));
}

BT::PortsList Moveit_Bridge::providedPorts()
{
    return {
        BT::InputPort<std::string>("arm_goal"), // arm goal in map frame
    };
}

BT::NodeStatus Moveit_Bridge::onStart()
{
    auto goal = getInput<std::string>("arm_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [arm_goal]");
        return BT::NodeStatus::FAILURE;
    }

    std::vector<double> goalVec = bt_string_serialize::stringToVector(goal.value());
    double x = goalVec[0];
    double y = goalVec[1];
    double z = goalVec[2];

    geometry_msgs::msg::PoseStamped target_base_link, target_map;
    target_base_link.header.frame_id = "base_link";
    target_base_link.pose.position.x = x;
    target_base_link.pose.position.y = y;
    target_base_link.pose.position.z = z;
    target_base_link.pose.orientation.x = 0;
    target_base_link.pose.orientation.y = 0;
    target_base_link.pose.orientation.z = 0;
    target_base_link.pose.orientation.w = 1;

    pose_publisher_->publish(target_base_link);
    RCLCPP_INFO(node_->get_logger(), "Published goal to moveit");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Moveit_Bridge::onRunning()
{
    if (finished){
        if (moveit_result){
            return BT::NodeStatus::SUCCESS;
        }
        else{
            return BT::NodeStatus::FAILURE;
        }
        finished = false; // reset the flag
    }
    return BT::NodeStatus::RUNNING;
}

void Moveit_Bridge::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    finished = true;
    if (msg->data == "success")
    {
        moveit_result = true;
    }
    else
    {
        moveit_result = false;
    }
}