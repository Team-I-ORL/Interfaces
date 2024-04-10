#include <orbiter_getVendingLoc.h>

arucoArmPose::arucoArmPose(const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    aruco_pose_client = node_->create_client<ros2_aruco_interfaces::srv::ArucoPose>("get_aruco_pose");
}

BT::PortsList arucoArmPose::providedPorts()
{
    return { BT::InputPort<std::string>("vending_machine_id"),
    BT::OutputPort<std::string>("arm_goal"),
    };
}

BT::NodeStatus arucoArmPose::onStart()
{
    BT::Optional<std::string> vending_machine_id = getInput<std::string>("vending_machine_id");
    if (!vending_machine_id)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [vending_machine_id]");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<ros2_aruco_interfaces::srv::ArucoPose::Request>();
    request->id = bt_string_serialize::stringToInt(vending_machine_id.value());
    auto result = aruco_pose_client->async_send_request(request, std::bind(&arucoArmPose::result_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Request sent, waiting for response.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus arucoArmPose::onRunning()
{
    if (this->finished)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}

void arucoArmPose::result_callback(rclcpp::Client<ros2_aruco_interfaces::srv::ArucoPose>::SharedFuture result)
{
    auto response = result.get();
    auto arucoPose = response->pose;
    std::string arm_goal = bt_string_serialize::vectorToString({arucoPose.position.x, arucoPose.position.y, arucoPose.position.z});
    RCLCPP_INFO(node_->get_logger(), "Received arm goal: %s", arm_goal.c_str());
    setOutput("arm_goal", arm_goal);
    this->finished = true;
    return;
}

