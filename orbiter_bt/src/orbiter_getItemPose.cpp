#include "orbiter_getItemPose.h"

GetItemPose::GetItemPose(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "GetItemPose has been created.");
    // client  = node_->create_client<orbiter_bt::srv::Ims>("ims_service");
    suc_client = node_->create_client<orbiter_bt::srv::GetSucPose>("get_suc_pose");
    drop_client = node_->create_client<orbiter_bt::srv::GetDropPose>("get_drop_pose_from_head");
    update_scene_pub = node_->create_publisher<std_msgs::msg::Bool>("update_collision_env", 5);
    this->suc_finished = false;
    this->suc_success = false;
    this->drop_finished = false;
    this->drop_success = false;
}

BT::PortsList GetItemPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("type"), // suc or drop
        BT::InputPort<std::string>("itemname"), // item name for suc or aruco_id for drop
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
    BT::Optional<std::string> object = getInput<std::string>("itemname");
    if (!object)
    {   
        throw BT::RuntimeError("Get Item Pose missing required input [itemname]");
    }

    if (type == "suc"){
        this->suc_success = false;
        this->suc_finished = false;
        RCLCPP_INFO(node_->get_logger(), "Getting Suction Pose for %s", object.value().c_str());
        auto request = std::make_shared<orbiter_bt::srv::GetSucPose::Request>();
        std_msgs::msg::String item;
        item.data = object.value();
        request->item = item;
        auto result = suc_client->async_send_request(request, std::bind(&GetItemPose::suc_result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for Suction Pose, waiting for response.");

<<<<<<< HEAD
        // std_msgs::msg::Bool msg;
        // msg.data = false;
=======
        std_msgs::msg::Bool msg;
        msg.data = false;
>>>>>>> 27767336c3852c76f8cfa4b15286c758cdb4f343
        // update_scene_pub->publish(msg);
    }
    else if (type == "drop"){
        this->drop_finished = false;
        this->drop_success = false;
        RCLCPP_INFO(node_->get_logger(), "Getting drop pose for %s", object.value().c_str());
        auto request = std::make_shared<orbiter_bt::srv::GetDropPose::Request>();
        if (object.value().size() < 6){
            throw BT::RuntimeError("Invalid input for itemname for Drop, must be 'arucoX'");
        }
        char arucoID = object.value()[5]; // arucoX -> X
        request->aruco_id = arucoID - '0';
        auto result = drop_client->async_send_request(request, std::bind(&GetItemPose::drop_result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for Drop Pose, waiting for response.");

<<<<<<< HEAD
        // std_msgs::msg::Bool msg;
        // msg.data = true;
=======
        std_msgs::msg::Bool msg;
        msg.data = true;
>>>>>>> 27767336c3852c76f8cfa4b15286c758cdb4f343
        // update_scene_pub->publish(msg);
    }
    else{
        throw BT::RuntimeError("Invalid type input , must be 'suc' or 'drop'");
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetItemPose::onRunning()
{
    if (this->suc_finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Response received!");
        if (this->suc_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Pose received, success for Suction Pose");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {   
            RCLCPP_ERROR(node_->get_logger(), "Pose Estimation Failed for Suction Pose");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
        this->suc_finished = false;
        this->suc_success = false;
    }
    else if (this->drop_finished)
    {
        RCLCPP_INFO(node_->get_logger(), "Response received!");
        if (this->drop_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Pose received, success for Drop Pose");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {   
            RCLCPP_ERROR(node_->get_logger(), "Pose Estimation Failed for Drop Pose");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
        this->drop_finished = false;
        this->drop_success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for response, running");
        return BT::NodeStatus::RUNNING;
    }
    
}

void GetItemPose::suc_result_callback(rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received!");
    auto response = result.get();
    auto pose = response->pose;
    
    if (pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0){
        RCLCPP_ERROR(node_->get_logger(), "Suction Pose not found");
        this->suc_finished = true;
        this->suc_success = false;
        return;
    }

    std::string pose_str = bt_string_serialize::poseToString(pose);
    setOutput("pose", pose_str);
    RCLCPP_INFO(node_->get_logger(), "Suction Pose: %s", pose_str.c_str());
    this->suc_finished = true;
    this->suc_success = true;
}

void GetItemPose::drop_result_callback(rclcpp::Client<orbiter_bt::srv::GetDropPose>::SharedFuture result)
{
    RCLCPP_INFO(node_->get_logger(), "Response received, success");
    auto response = result.get();
    auto pose = response->pose;
    std::string pose_str = bt_string_serialize::poseToString(pose);
    setOutput("pose", pose_str);
    RCLCPP_INFO(node_->get_logger(), "Drop Pose: %s", pose_str.c_str());
    this->drop_finished = true;
    this->drop_success = true;
}