#include "orbiter_moveit_behaviors_wrapper.h"
MoveArm_Wrapper::MoveArm_Wrapper(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::MoveArm>("move_arm")),
    tfBuffer(node->get_clock()),
    tf_listener(tfBuffer)
{
    RCLCPP_INFO(node_->get_logger(), "Move Arm Wrapper Version Creating");
}

BT::PortsList MoveArm_Wrapper::providedPorts()
{
    return {
        BT::InputPort<std::string>("arm_goal"), // arm goal in map frame
        };
}  

BT::NodeStatus MoveArm_Wrapper::onStart()
{
    auto goal = getInput<std::string>("arm_goal");
    if (!goal)
    {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [arm_goal]");
        return BT::NodeStatus::FAILURE;
    }

    std::vector<double> goalVec = bt_string_serialize::stringToVector(goal.value());
    double x = goalVec[0];
    double y = goalVec[1];
    double z = goalVec[2];

    auto request = std::make_shared<orbiter_bt::srv::MoveArm::Request>();
    geometry_msgs::msg::PoseStamped target_map;
    geometry_msgs::msg::PoseStamped targetPose;
    target_map.header.frame_id = "map";
    target_map.pose.position.x = x;
    target_map.pose.position.y = y;
    target_map.pose.position.z = z;
    target_map.pose.orientation.w = 1.0;

    if (!tfBuffer.canTransform("map", "base_link", tf2::TimePointZero)) {
        RCLCPP_ERROR(node_->get_logger(), "Transform not available !!!");
        return BT::NodeStatus::FAILURE;
    }

    tfBuffer.transform(target_map, targetPose, "base_link");
    
    request->target_pose = targetPose;

    RCLCPP_INFO(node_->get_logger(), "Goal Created At: x: %f, y: %f, z: %f", targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z);
    auto result = client->async_send_request(request, std::bind(&MoveArm_Wrapper::result_callback, this, std::placeholders::_1));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm_Wrapper::onRunning()
{
    if (finished)
    {
        if (moveit_result == true)
        {
            RCLCPP_INFO(node_->get_logger(), "Move Arm Succeeded");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Move Arm Failed");
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void MoveArm_Wrapper::result_callback(rclcpp::Client<orbiter_bt::srv::MoveArm>::SharedFuture result)
{
    finished = true;
    auto response = result.get();
    moveit_result = response->success;
}