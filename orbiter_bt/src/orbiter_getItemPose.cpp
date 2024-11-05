#include "orbiter_getItemPose.h"

GetItemPose::GetItemPose(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemPose has been created.");
    // client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    suc_client = node_->create_client<orbiter_bt::srv::GetSucPose>("get_suc_pose");
    drop_client = node_->create_client<orbiter_bt::srv::GetDropPose>("get_drop_pose");
    update_scene_pub = node_->create_publisher<std_msgs::msg::Bool>("update_collision_env", 5);
    this->finished = false;
}

BT::PortsList GetItemPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("type"), // suc or drop
        BT::InputPort<std::string>("itemName"), // item name for suc or aruco_id for drop
        BT::OutputPort<std::string>("pose")
    };
}

BT::NodeStatus GetItemPose::onStart()
{   
    // Get type from input port
    BT::Optional<std::string> typeInput = getInput<std::string>("type");
    if (!typeInput)
    {
        throw BT::RuntimeError("missing required input [type]");
    }
    type = typeInput.value();

    // Get item name from input port
    BT::Optional<std::string> object = getInput<std::string>("itemName");
    if (!object)
    {
        throw BT::RuntimeError("missing required input [itemName]");
    }

    if (type == "suc"){
        RCLCPP_INFO(node_->get_logger(), "Getting pose for %s", object.value().c_str());
        auto request = std::make_shared<orbiter_bt::srv::GetSucPose::Request>();
        std_msgs::msg::String item;
        item.data = object.value();
        request->item = item;
        auto result = suc_client->async_send_request(request, std::bind(&GetItemPose::suc_result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for Suction Pose, waiting for response.");

        std_msgs::msg::Bool msg;
        msg.data = true;
        update_scene_pub->publish(msg);
    }
    else if (type == "drop"){
        RCLCPP_INFO(node_->get_logger(), "Getting drop pose for %s", object.value().c_str());
        auto request = std::make_shared<orbiter_bt::srv::GetDropPose::Request>();
        request->aruco_id = bt_string_serialize::stringToInt(object.value());
        auto result = drop_client->async_send_request(request, std::bind(&GetItemPose::drop_result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for Drop Pose, waiting for response.");

        std_msgs::msg::Bool msg;
        msg.data = false;
        update_scene_pub->publish(msg);
        sleep(1);

        std_msgs::msg::Bool msg2;
        msg2.data = true;
        update_scene_pub->publish(msg2);
    }
    else{
        throw BT::RuntimeError("Invalid type input , must be 'suc' or 'drop'");
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetItemPose::onRunning()
{
    if (this->finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Response received, success");
        this->finished = false;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for response, running");
        return BT::NodeStatus::RUNNING;
    }
    
}

void GetItemPose::suc_result_callback(rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    auto pose = response->pose;
    std::string pose_str = bt_string_serialize::poseToString(pose);
    setOutput("pose", pose_str);
    RCLCPP_INFO(node_->get_logger(), "Suction Pose: %s", pose_str.c_str());
    this->finished = true;
}

void GetItemPose::drop_result_callback(rclcpp::Client<orbiter_bt::srv::GetDropPose>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    auto pose = response->pose;
    std::string pose_str = bt_string_serialize::poseToString(pose);
    setOutput("pose", pose_str);
    RCLCPP_INFO(node_->get_logger(), "Drop Pose: %s", pose_str.c_str());
    this->finished = true;
}