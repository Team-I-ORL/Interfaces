#include "orbiter_getItemInfo.h"

GetItemInfo::GetItemInfo(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemInfo has been created.");
    // client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    item_client = node_->create_client<ims_interfaces::srv::Item>("item_service"); // need to change this to item service
    this->finished = false;
}

BT::PortsList GetItemInfo::providedPorts()
{
    return {
        BT::InputPort<std::string>("id"),
        BT::OutputPort<std::string>("name"),
        BT::OutputPort<std::string>("location"),
        BT::OutputPort<std::string>("yaw"),
        BT::OutputPort<std::string>("quantity"),
        };
       
}

BT::NodeStatus GetItemInfo::onStart()
{
    // Get item name from input port
    BT::Optional<std::string> id = getInput<std::string>("id");
    if (!id)
    {
        throw BT::RuntimeError("missing required input [id]");
    }

    RCLCPP_INFO(node_->get_logger(), "Getting info for %s", id.value().c_str());
    auto request = std::make_shared<ims_interfaces::srv::Item::Request>();

    request->id = bt_string_serialize::stringToInt(id.value());
    auto result = item_client->async_send_request(request, std::bind(&GetItemInfo::result_callback, this, std::placeholders::_1));
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

void GetItemInfo::result_callback(rclcpp::Client<ims_interfaces::srv::Item>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    
    setOutput("name", response->name);
    setOutput("location", bt_string_serialize::geoMsgPtToString(response->location));
    setOutput("yaw", bt_string_serialize::intToString(response->yaw));
    setOutput("quantity", bt_string_serialize::intToString(response->quantity));

    this->finished = true;
}