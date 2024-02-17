#include "orbiter_getItemInfo.h"

GetItemInfo::GetItemInfo(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemInfo has been created.");
    client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    this->finished = false;
}

BT::PortsList GetItemInfo::providedPorts()
{
    return {
        BT::InputPort<std::string>("itemName"),
        BT::OutputPort<std::string>("itemInfo"),
        BT::OutputPort<std::string>("loc")};
       
}

BT::NodeStatus GetItemInfo::onStart()
{
    // Get item name from input port
    BT::Optional<std::string> itemName = getInput<std::string>("itemName");
    if (!itemName)
    {
        throw BT::RuntimeError("missing required input [itemName]");
    }
    RCLCPP_INFO(node_->get_logger(), "Getting info for %s", itemName.value().c_str());
    auto request = std::make_shared<orbiter_bt::srv::Ims::Request>();
    request->itemname = itemName.value();
    auto result = client->async_send_request(request, std::bind(&GetItemInfo::result_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Request sent, waiting for response.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetItemInfo::onRunning()
{
    if (this->finished)
    {
        return BT::NodeStatus::SUCCESS;
        RCLCPP_INFO(node_->get_logger(), "Response received, success");
    }
    else
    {
        return BT::NodeStatus::RUNNING;
        RCLCPP_INFO(node_->get_logger(), "Waiting for response, running");
    }
    
}

void GetItemInfo::result_callback(rclcpp::Client<orbiter_bt::srv::Ims>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    RCLCPP_INFO(node_->get_logger(), "Item Info: %s", response->iteminfo.c_str());
    RCLCPP_INFO(node_->get_logger(), "Location: %s", response->loc.c_str());
    setOutput("itemInfo", response->iteminfo);
    setOutput("loc", response->loc);
    this->finished = true;
}