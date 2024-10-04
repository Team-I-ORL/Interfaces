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
    this->declare_parameter<std::string>("segmask_service_name", "/dummy_segment_srv");
    this->declare_parameter<std::string>("sucpose_service_name", "/sucpose_service");

    // Retrieve parameter values
    this->get_parameter("base_link_name", base_link_name);
    this->get_parameter("camera_link_name", camera_link_name);
    this->get_parameter("segmask_service_name", segmask_service_name);
    this->get_parameter("sucpose_service_name", sucpose_service_name);

    _client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _sucpose_client = this->create_client<perception_interfaces::srv::Sucpose>(sucpose_service_name,
        rmw_qos_profile_services_default, _client_callback_group);
    _segmask_client = this->create_client<perception_interfaces::srv::Segmask>(segmask_service_name,
        rmw_qos_profile_services_default, _client_callback_group);
    
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

    auto segmask_result = _segmask_client->async_send_request(segmask_request,
         std::bind(&PerceptionManager::_seg_mask_callback, this, std::placeholders::_1));
    
    while (rclcpp::ok() && !_segmask_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Waiting for segmentation mask service...");
    }

    // Call suc pose service
    auto sucpose_request = std::make_shared<perception_interfaces::srv::Sucpose::Request>();
    sucpose_request->color_image = color_image;
    sucpose_request->depth_image = depth_image;
    sucpose_request->segmask = _segmask;
    sucpose_request->camera_info = camera_info;

    while (!_sucpose_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto sucpose_result = _sucpose_client->async_send_request(sucpose_request,
         std::bind(&PerceptionManager::_suc_pose_callback, this, std::placeholders::_1));

    while (rclcpp::ok() && !_sucpose_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Waiting for suction pose service...");
    }

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
        tf2::doTransform(_sucpose, suction_pose_base_link, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }

    response->pose = suction_pose_base_link;

    _segmask_received = false;
    _sucpose_received = false;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto perception_manager = std::make_shared<PerceptionManager>();
    executor.add_node(perception_manager);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}