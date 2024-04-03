#include "orbiter_getItemInfo.h"

GetItemInfo::GetItemInfo(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemInfo has been created.");
    // client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    item_client = node_->create_client<ims_interfaces::srv::Item>("dims"); // need to change this to item service
    this->finished = false;
}

BT::PortsList GetItemInfo::providedPorts()
{
    return {
        BT::InputPort<std::string>("id"),
        BT::OutputPort<std::string>("item_name"),
        BT::OutputPort<std::string>("arm_goal"),
        BT::OutputPort<std::string>("nav_goal"),
        BT::OutputPort<std::string>("yaw"),
        BT::OutputPort<std::string>("quantity"),
        BT::OutputPort<std::string>("vending_machine_id")
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
    
    setOutput("item_name", response->name.data);
    RCLCPP_INFO(node_->get_logger(), "Item name: %s", response->name.data.c_str());
    auto arm_goal = response->location;
    arm_goal.x = arm_goal.x + 0.1;
    arm_goal.y = arm_goal.y;
    arm_goal.z = 0.0;
    setOutput("arm_goal", bt_string_serialize::geoMsgPtToString(arm_goal));
    auto nav_goal = response->location;
    // RCLCPP_INFO(node_->get_logger(), "Arm goal: x=%f, y=%f, z=%f", arm_goal.x, arm_goal.y, arm_goal.z);
    nav_goal.x = nav_goal.x + 0.5;
    nav_goal.y = nav_goal.y;
    nav_goal.z = 0.0;
    // RCLCPP_INFO(node_->get_logger(), "Nav goal: x=%f, y=%f", nav_goal.x, nav_goal.y);
    setOutput("nav_goal", bt_string_serialize::geoMsgPtToString(nav_goal));
    setOutput("yaw", bt_string_serialize::intToString(response->yaw));
    // RCLCPP_INFO(node_->get_logger(), "Yaw: %lu", response->yaw);
    setOutput("quantity", bt_string_serialize::intToString(response->quantity));
    // RCLCPP_INFO(node_->get_logger(), "Quantity: %i", response->quantity);
    setOutput("vending_machine_id", bt_string_serialize::intToString(response->vending_machine_id));
    // RCLCPP_INFO(node_->get_logger(), "Vending machine id: %i", response->vending_machine_id);

    this->finished = true;
}