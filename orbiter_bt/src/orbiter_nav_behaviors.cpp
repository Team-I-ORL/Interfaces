#include "orbiter_nav_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GoToPose has been created.");
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    nav_done_flag = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("nav_goal"),
        BT::InputPort<std::string>("yaw"),
        // BT::OutputPort<std::string>("result")
        };
}

BT::NodeStatus GoToPose::onStart()
{   
    // Get location names from input port
    BT::Optional<std::string> loc = getInput<std::string>("nav_goal");
    BT::Optional<std::string> yaw = getInput<std::string>("yaw");
    if (!loc)
    {
        throw BT::RuntimeError("missing required input [nav_goal]");
    }
    if (!yaw)
    {
        throw BT::RuntimeError("missing required input [yaw]");
    }
    // Get goal from IMS based on location name
    std::vector<double> goal = bt_string_serialize::stringToVector(loc.value());
    int theta = bt_string_serialize::stringToInt(yaw.value());
    

    // Send goal to action server
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::navigate_to_pose_callback, this, std::placeholders::_1);
    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal[0];
    goal_msg.pose.pose.position.y = goal[1];
    tf2::Quaternion q;
    q.setZ(0.6);
    q.setW(1.0);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);
    // RCLCPP_INFO(node_->get_logger(), "Goal: X: %f, Y: %f, theta: %i", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, theta);
    // std::cout << goal_msg << std::endl;
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        return BT::NodeStatus::FAILURE;
    }
    action_client_->async_send_goal(goal_msg, send_goal_options);
    nav_done_flag = false;
    RCLCPP_INFO(node_->get_logger(), "Goal sent");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{   
    if (nav_done_flag)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::navigate_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result){
        nav_done_flag = true;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Goal reached");
            setOutput("result", "Goal reached");
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Goal aborted");
            setOutput("result", "Goal aborted");
        }
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Goal canceled");
            setOutput("result", "Goal canceled");
        }
    }
}