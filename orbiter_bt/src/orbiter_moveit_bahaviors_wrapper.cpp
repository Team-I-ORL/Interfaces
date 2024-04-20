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
    geometry_msgs::msg::PoseStamped target_base_link, target_map;
    // Go to named pose home if goal is 0,0,0

    if (x == 0.0 && y == 0.0 && z == 0.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Goal is 0,0,0, returning to home pose");
        target_base_link.header.frame_id = "home";
        request->target_pose = target_base_link;
    }
    else if (x == 0.0 && y == 0.0 && z == 1.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Goal is 0,0,1, going to inter_pose_1");
        target_base_link.header.frame_id = "inter_pose_1";
        request->target_pose = target_base_link;
    }
    else if (x == 0.0 && y == 0.0 && z == 2.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Goal is 0,0,2, going to inter_pose_2");
        target_base_link.header.frame_id = "inter_pose_2";
        request->target_pose = target_base_link;
    }
    else{
        target_base_link.header.frame_id = "base_link";
        target_base_link.pose.position.x = x;
        target_base_link.pose.position.y = y;
        target_base_link.pose.position.z = z;
        target_base_link.pose.orientation.w = 1.0;
        // RCLCPP_INFO(node_->get_logger(), "Position BL: x: %f, y: %f, z: %f", target_base_link.pose.position.x, target_base_link.pose.position.y, target_base_link.pose.position.z);

        // Wait for transform to be available
        const std::string BASE_LINK = "base_link";

        // if (!tfBuffer.canTransform("map", BASE_LINK, tf2::TimePointZero)) {
        //     RCLCPP_ERROR(node_->get_logger(), "Transform not available !!!");
        //     return BT::NodeStatus::FAILURE;
        // }

        // Wait for transform
        // while (rclcpp::ok()) {
        //     try {
        //         tfBuffer.lookupTransform("map", BASE_LINK, tf2::TimePointZero);
        //         break;
        //     } catch (tf2::TransformException &ex) {
        //         RCLCPP_WARN(node_->get_logger(), "Waiting for [map -> %s] transform to become available", BASE_LINK);
        //         rclcpp::sleep_for(std::chrono::milliseconds(100));
        //     }
        // }

        // try {
        //     tfBuffer.transform(target_base_link, target_map, "map");
        // } catch (tf2::TransformException &ex) {
        //     RCLCPP_ERROR(node_->get_logger(), "Could not transform point: %s", ex.what());
        //     return BT::NodeStatus::FAILURE;
        // }

        //////////////////////////////////

        // tfBuffer.transform(target_base_link, target_map, "map");
        // auto target_map = tfBuffer.transform(target_base_link, "map", tf2::durationFromSec(3.0));

        request->target_pose = target_base_link;
        RCLCPP_INFO(node_->get_logger(), "Goal Created At: x: %f, y: %f, z: %f", target_base_link.pose.position.x, target_base_link.pose.position.y, target_base_link.pose.position.z);
    }
    
    // Check if the server is available
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Move arm service is not available");
        return BT::NodeStatus::FAILURE;
    }
    auto result = client->async_send_request(request, std::bind(&MoveArm_Wrapper::result_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Sent goal to move arm service");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm_Wrapper::onRunning()
{
    if (finished)
    {
        if (moveit_result == true)
        {   
            finished = false;
            RCLCPP_INFO(node_->get_logger(), "Move Arm Succeeded");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {   
            finished = false;
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