#include "orbiter_getSucPose.h"

GetItemPose::GetItemPose(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemPose has been created.");
    // client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    client = node_->create_client<orbiter_bt::srv::GetSucPose>("get_suc_pose");
    this->finished = false;
}

BT::PortsList GetItemPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("itemName"),
        BT::OutputPort<std::string>("pose")
    };
}

BT::NodeStatus GetItemPose::onStart()
{
    // Get item name from input port
    BT::Optional<std::string> object = getInput<std::string>("itemName");
    if (!object)
    {
        throw BT::RuntimeError("missing required input [itemName]");
    }

    RCLCPP_INFO(node_->get_logger(), "Getting pose for %s", object.value().c_str());
    auto request = std::make_shared<orbiter_bt::srv::GetSucPose::Request>();
    std_msgs::msg::String item;
    item.data = object.value();
    request->item = item;
    auto result = client->async_send_request(request, std::bind(&GetItemPose::result_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Request sent, waiting for response.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetItemPose::onRunning()
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

void GetItemPose::result_callback(rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    auto pose = response->pose;
    std::string pose_str = bt_string_serialize::poseToString(pose);
    setOutput("pose", pose_str);
    RCLCPP_INFO(node_->get_logger(), "Pose: %s", pose_str.c_str());
    this->finished = true;
}
