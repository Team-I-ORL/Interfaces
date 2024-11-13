#include "orbiter_getNextAction.h"

GetNextAction::GetNextAction(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    client = node_->create_client<orbiter_bt::srv::NextAction>("get_next_action");
    this->finished = false;
}

BT::PortsList GetNextAction::providedPorts()
{
    return {
        BT::OutputPort<std::string>("next_action"),
        BT::OutputPort<std::string>("itemname")
    };
}

BT::NodeStatus GetNextAction::onStart()
{
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<orbiter_bt::srv::NextAction::Request>();
    auto future_result = client->async_send_request(request, std::bind(&GetNextAction::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextAction::onRunning()
{
    if (finished) {
        finished = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GetNextAction::result_callback(rclcpp::Client<orbiter_bt::srv::NextAction>::SharedFuture result)
{
    auto response = result.get();
    finished = true;
    setOutput("next_action", response->next_action);
    setOutput("itemname", response->item);
}