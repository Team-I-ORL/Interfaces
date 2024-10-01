#include "perception_client.h"

PerceptionClient::PerceptionClient() : 
    Node("perception_manager")
{
    _get_suc_pose_client = this->create_client<orbiter_bt::srv::GetSucPose>("get_suc_pose");
    _rgb_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "rgb_image", 5, std::bind(&PerceptionClient::rgb_callback, this, std::placeholders::_1));
    _depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "depth_image", 5, std::bind(&PerceptionClient::depth_callback, this, std::placeholders::_1));
    _camera_info_subscription = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 5, std::bind(&PerceptionClient::camera_info_callback, this, std::placeholders::_1));
}

void PerceptionClient::send_request()
{
    auto request = std::make_shared<orbiter_bt::srv::GetSucPose::Request>();
    request->item.data = "item";
    request->color_image = rgb_image;
    request->depth_image = depth_image;
    request->camera_info = camera_info;

    while (!_get_suc_pose_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = _get_suc_pose_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service get_suc_pose");
        return;
    }

    auto response = result.get()->pose;
    RCLCPP_INFO(this->get_logger(), "Response: %f %f %f", response.position.x, response.position.y, response.position.z);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionClient>();
    node->send_request();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}