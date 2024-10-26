#include "orbiter_suck.h"

SuctionCmd::SuctionCmd(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::Suck>("suction_command"))
{
    RCLCPP_INFO(node_->get_logger(), "Suction Command Creating");
}

BT::PortsList SuctionCmd::providedPorts()
{
    return {
        BT::InputPort<std::string>("cmd"),
        };
}

BT::NodeStatus SuctionCmd::onStart()
{
    auto goal = getInput<std::string>("cmd");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [cmd]");
        return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<orbiter_bt::srv::Suck::Request>();
    request->command = bt_string_serialize::stringToInt(goal.value());

    auto result = client->async_send_request(request, std::bind(&SuctionCmd::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SuctionCmd::onRunning()
{   
    RCLCPP_INFO(node_->get_logger(), "Suction Command Running!");
    auto time_now = node_->now();
    if ((time_now - start_time).seconds() > 10)
    {
        RCLCPP_INFO(node_->get_logger(), "Suction Command Timeout!");
        return BT::NodeStatus::FAILURE;
    }
    if (finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Suction Command Finished!");
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

void SuctionCmd::result_callback(rclcpp::Client<orbiter_bt::srv::Suck>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Move Head Result Received");
    move_head_result = result.get()->success;
    finished = true;
}