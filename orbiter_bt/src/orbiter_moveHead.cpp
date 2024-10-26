#include "orbiter_moveHead.h"

MoveHead::MoveHead(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::MoveHead>("move_head"))
{
    RCLCPP_INFO(node_->get_logger(), "Move Head Creating");
}

BT::PortsList MoveHead::providedPorts()
{
    return {
        BT::InputPort<std::string>("head_goal"),
        };
}

BT::NodeStatus MoveHead::onStart()
{
    auto goal = getInput<std::string>("head_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [head_goal]");
        return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<orbiter_bt::srv::MoveHead::Request>();
    request->what = goal.value();

    RCLCPP_INFO(node_->get_logger(), "Sending Head Goal: %s", goal.value().c_str());
    auto result = client->async_send_request(request, std::bind(&MoveHead::result_callback, this, std::placeholders::_1));
}

BT::NodeStatus MoveHead::onRunning()
{   
    RCLCPP_INFO(node_->get_logger(), "Move Head Running!");
    auto time_now = node_->now();
    if ((time_now - start_time).seconds() > 10)
    {
        RCLCPP_INFO(node_->get_logger(), "Move Head Timeout!");
        return BT::NodeStatus::FAILURE;
    }
    if (finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Move Head Finished!");
        if (move_head_result)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void MoveHead::result_callback(rclcpp::Client<orbiter_bt::srv::MoveHead>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Move Head Result Received");
    move_head_result = result.get()->success;
    finished = true;
}