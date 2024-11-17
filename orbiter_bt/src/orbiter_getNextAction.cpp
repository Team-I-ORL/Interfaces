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
        BT::OutputPort<std::string>("itemname"),
        BT::OutputPort<std::string>("aruco_id")
    };
}

BT::NodeStatus GetNextAction::onStart()
{
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        return BT::NodeStatus::FAILURE;
    }
    finished = false;
    success = true;
    auto request = std::make_shared<orbiter_bt::srv::NextAction::Request>();
    auto future_result = client->async_send_request(request, std::bind(&GetNextAction::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextAction::onRunning()
{
    if (finished) {
        if (!success){
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GetNextAction::result_callback(rclcpp::Client<orbiter_bt::srv::NextAction>::SharedFuture result)
{
    auto response = result.get();
    finished = true;
    RCLCPP_INFO(node_->get_logger(), "Next Action: %s", response->next_action.c_str());
    setOutput("next_action", response->next_action);
    setOutput("itemname", response->item);
    RCLCPP_INFO(node_->get_logger(), "Item: %s", response->item.c_str());
    if (response->next_action == "Restocking"){
        if (item_to_aruco_id_restock.find(response->item) == item_to_aruco_id_restock.end()){
            RCLCPP_ERROR(node_->get_logger(), "Item not found in restock map");
            success = false;
            return;
        }
        setOutput("aruco_id", item_to_aruco_id_restock.at(response->item));
        RCLCPP_INFO(node_->get_logger(), "Aruco ID: %s", item_to_aruco_id_restock.at(response->item).c_str());
        success = true;
    }
    else if (response->next_action == "Retrieval"){
        if (item_to_aruco_id_retrieve.find(response->item) == item_to_aruco_id_retrieve.end()){
            RCLCPP_ERROR(node_->get_logger(), "Item not found in retrieve map");
            success = false;
            return;
        }
        setOutput("aruco_id", item_to_aruco_id_retrieve.at(response->item));
        success = true;
        RCLCPP_INFO(node_->get_logger(), "Aruco ID: %s", item_to_aruco_id_retrieve.at(response->item).c_str());
    }
}