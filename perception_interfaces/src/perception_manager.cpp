#include "perception_manager.h"

PerceptionManager::PerceptionManager() : 
    Node("perception_manager"),
    tfBuffer(this->get_clock()),
    tf_listener(tfBuffer),
    tf_broadcaster(this)
{
    // Declare parameters
    this->declare_parameter<std::string>("base_link_name", "/base_link");
    this->declare_parameter<std::string>("camera_link_name", "/head_camera_rgb_optical_frame");
    this->declare_parameter<std::string>("segmask_service_name", "/dummy_segment");
    this->declare_parameter<std::string>("sucpose_service_name", "/sucpose_service");

    // Retrieve parameter values
    this->get_parameter("base_link_name", base_link_name);
    this->get_parameter("camera_link_name", camera_link_name);
    this->get_parameter("segmask_service_name", segmask_service_name);
    this->get_parameter("sucpose_service_name", sucpose_service_name);

    _sucpose_client = this->create_client<perception_interfaces::srv::Sucpose>(sucpose_service_name);
    _segmask_client = this->create_client<perception_interfaces::srv::Segmask>(segmask_service_name);
    _get_suc_pose_service = this->create_service<orbiter_bt::srv::GetSucPose>("get_suc_pose",
        std::bind(&PerceptionManager::_get_suc_pose, this, std::placeholders::_1, std::placeholders::_2));
}

void PerceptionManager::_get_suc_pose(const std::shared_ptr<orbiter_bt::srv::GetSucPose::Request> request,
                           std::shared_ptr<orbiter_bt::srv::GetSucPose::Response> response)
{
    // Retrieve request information
    std::string item = request->item.data;
    sensor_msgs::msg::Image color_image = request->color_image;
    sensor_msgs::msg::Image depth_image = request->depth_image;
    sensor_msgs::msg::CameraInfo camera_info = request->camera_info;

    // Call segmentation mask service
    auto segmask_request = std::make_shared<perception_interfaces::srv::Segmask::Request>();
    segmask_request->object_of_interest.data = item;
    segmask_request->color_image = color_image;
    segmask_request->camera_info = camera_info;

    while (!_segmask_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto segmask_result = _segmask_client->async_send_request(segmask_request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), segmask_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service segmask");
        return;
    }

    auto segmask_response = segmask_result.get();
    sensor_msgs::msg::Image segmask = segmask_response->segmask;

    // Call suc pose service
    auto sucpose_request = std::make_shared<perception_interfaces::srv::Sucpose::Request>();
    sucpose_request->color_image = color_image;
    sucpose_request->depth_image = depth_image;
    sucpose_request->segmask = segmask;
    sucpose_request->camera_info = camera_info;

    while (!_sucpose_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto sucpose_result = _sucpose_client->async_send_request(sucpose_request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), sucpose_result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service sucpose");
        return;
    }

    auto sucpose_response = sucpose_result.get(); // Suction pose in camera frame

    //transform suction pose to base frame
    geometry_msgs::msg::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform(base_link_name, camera_link_name,
                                                    tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }

    geometry_msgs::msg::Pose suction_pose_base_link;
    try {
        tf2::doTransform(sucpose_response->pose, suction_pose_base_link, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }

    response->pose = suction_pose_base_link;

    // Publish suction pose in base frame
    geometry_msgs::msg::TransformStamped transformStampedBase;
    transformStampedBase.header.stamp = this->now();
    transformStampedBase.header.frame_id = base_link_name;
    transformStampedBase.child_frame_id = "suction_pose";
    transformStampedBase.transform.translation.x = suction_pose_base_link.position.x;
    transformStampedBase.transform.translation.y = suction_pose_base_link.position.y;
    transformStampedBase.transform.translation.z = suction_pose_base_link.position.z;
    transformStampedBase.transform.rotation = suction_pose_base_link.orientation;
    tf_broadcaster.sendTransform(transformStampedBase);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionManager>());
    rclcpp::shutdown();
    return 0;
}