#include "orbiter_getItemInfo.h"

GetItemInfo::GetItemInfo(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemInfo has been created.");
    rclcpp::Client<orbiter_interfaces::srv::ims>::SharedPtr client 
        = node_->create_client<orbiter_interfaces::srv::GetItemInfo>("get_item_info");
}

BT::PortsList GetItemInfo::providedPorts()
{
    return {
        BT::InputPort<std::string>("itemName"),
        BT::OutputPort<std::string>("itemInfo")};
        BT::OutputPort<std::string>("loc");
}

BT::NodeStatus GetItemInfo::tick()
{
    // Get item name from input port
    BT::Optional<std::string> itemName = getInput<std::string>("itemName");
    if (!itemName)
    {
        throw BT::RuntimeError("missing required input [itemName]");
    }
    RCLCPP_INFO(node_->get_logger(), "Getting info for %s", itemName.value().c_str());
    auto request = std::make_shared<orbiter_interfaces::srv::GetItemInfo::Request>();
    request -> itemName = "example";
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Item Info: %s", result.get()->iteminfo);
        RCLCPP_INFO(node_->get_logger(), "Location: %s", result.get()->loc);
        setOutput("itemInfo", result.get()->iteminfo);
        setOutput("loc", result.get()->loc);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
        return BT::NodeStatus::FAILURE;
    }
    
}