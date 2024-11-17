#include "perception_manager.h"

PerceptionManager::PerceptionManager() : 
    Node("perception_manager"),
    tfBuffer(this->get_clock()),
    tf_listener(tfBuffer),
    tf_broadcaster(this)
{
    // Declare parameters
    this->declare_parameter<std::string>("base_link_name", "torso_lift_link");
    this->declare_parameter<std::string>("camera_link_name", "head_camera_rgb_optical_frame");
    this->declare_parameter<std::string>("segmask_service_name", "/seg_mask");
    this->declare_parameter<std::string>("sucpose_service_name", "/sucpose_service");

    // Retrieve parameter values
    this->get_parameter("base_link_name", base_link_name);
    this->get_parameter("camera_link_name", camera_link_name);
    this->get_parameter("segmask_service_name", segmask_service_name);
    this->get_parameter("sucpose_service_name", sucpose_service_name);

    // Create clients
    _client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    _sucpose_client = this->create_client<perception_interfaces::srv::Sucpose>(sucpose_service_name,
        rmw_qos_profile_services_default, _client_callback_group);
    _segmask_client = this->create_client<perception_interfaces::srv::Segmask>(segmask_service_name,
        rmw_qos_profile_services_default, _client_callback_group);
    _droppose_client = this->create_client<perception_interfaces::srv::Droppose>("/droppose_service",
        rmw_qos_profile_services_default, _client_callback_group);

    _find_aruco_in_frame_client = this->create_client<perception_interfaces::srv::FindObjInFrame>("/find_aruco_in_frame",
        rmw_qos_profile_services_default, _client_callback_group);
    
    _find_box_in_frame_client = this->create_client<perception_interfaces::srv::FindObjInFrame>("/find_box_in_frame",
        rmw_qos_profile_services_default, _client_callback_group);

    // Create subscriptions
    _subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> sub_options;
    sub_options.callback_group = _subscription_callback_group;
    _color_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/head_camera/rgb/image_raw", 5, 
        std::bind(&PerceptionManager::_color_image_callback, this, std::placeholders::_1), sub_options);
    _depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/head_camera/depth/image_rect_raw", 5,
        std::bind(&PerceptionManager::_depth_image_callback, this, std::placeholders::_1), sub_options);
    _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/head_camera/camera_info", 5,
        std::bind(&PerceptionManager::_camera_info_callback, this, std::placeholders::_1), sub_options);

    // Create publisher
    _pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/suction_pose_pm", 5);
    
    _service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Create service
    _get_suc_pose_service = this->create_service<orbiter_bt::srv::GetSucPose>("get_suc_pose",
        std::bind(&PerceptionManager::_get_suc_pose, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, _service_callback_group);
    _get_drop_pose_service = this->create_service<orbiter_bt::srv::GetDropPose>("get_drop_pose",
        std::bind(&PerceptionManager::_get_drop_pose, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, _service_callback_group);
    _find_x_service = this->create_service<perception_interfaces::srv::FindX>("find_x",
        std::bind(&PerceptionManager::_find_x, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, _service_callback_group);
}

void PerceptionManager::_get_suc_pose(const std::shared_ptr<orbiter_bt::srv::GetSucPose::Request> request,
                           std::shared_ptr<orbiter_bt::srv::GetSucPose::Response> response)
{   
    _service_count++;
    // Retrieve request information
    std::string item = request->item.data;
    RCLCPP_INFO(this->get_logger(), "Perception Manager Got Request GetSucPose for %s", item.c_str());
    // Call segmentation mask service
    auto segmask_request = std::make_shared<perception_interfaces::srv::Segmask::Request>();
    segmask_request->object_of_interest.data = item;
    if (_color_image.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Color image is empty while calling segmentation mask service");
    }
    else if (_color_image.encoding != "rgb8") {
        RCLCPP_ERROR(this->get_logger(), "Color image encoding is not rgb8 while calling segmentation mask service, is ", _color_image.encoding);
    }
    segmask_request->color_image = _color_image;
    segmask_request->color_image.encoding = "rgb8";
    segmask_request->camera_info = _camera_info;

    while (!_segmask_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto segmask_result = _segmask_client->async_send_request(segmask_request,
         std::bind(&PerceptionManager::_seg_mask_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Segmentation mask service called");
    _client_count++;
    
    while (rclcpp::ok() && !_segmask_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Waiting for segmentation mask service...");
    }
    // segmask_result.wait_for(std::chrono::seconds(5));
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Call suc pose service
    auto sucpose_request = std::make_shared<perception_interfaces::srv::Sucpose::Request>();
    if (_color_image.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Color image is empty while calling suc pose service");
        return;
    }
    else if (_color_image.encoding != "rgb8") {
        RCLCPP_ERROR(this->get_logger(), "Color image encoding is not rgb8 while calling suc pose service");
        return;
    }
    if (_depth_image.data.empty() || _depth_image.width == 0 || _depth_image.height == 0) {
        RCLCPP_ERROR(this->get_logger(), "Depth image is empty while calling suc pose service");
        return;
    }
    else if (_depth_image.encoding != "16UC1") {
        RCLCPP_ERROR(this->get_logger(), "Depth image encoding is not 16UC1 while calling suc pose service");
        return;
    }
    if (_segmask.data.empty() || _segmask.width == 0 || _segmask.height == 0) {
        RCLCPP_ERROR(this->get_logger(), "Segmentation mask is empty while calling suc pose service");
        return;
    }
    sucpose_request->color_image = _color_image;
    sucpose_request->color_image.encoding = "rgb8";
    sucpose_request->depth_image = _depth_image;
    sucpose_request->depth_image.encoding = "16UC1";
    sucpose_request->segmask = _segmask;
    sucpose_request->camera_info = _camera_info;

    while (!_sucpose_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto sucpose_result = _sucpose_client->async_send_request(sucpose_request,
         std::bind(&PerceptionManager::_suc_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Suction pose service called");
    _client_count++;
    while (rclcpp::ok() && !_sucpose_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "Waiting for suction pose service...");
    }
    // sucpose_result.wait();

    if (std::abs(_sucpose.position.x) < 0.01 && std::abs(_sucpose.position.y) < 0.01 && std::abs(_sucpose.position.z) < 0.01) {
        RCLCPP_ERROR(this->get_logger(), "Received suction pose at origin");
        return;
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
    RCLCPP_INFO(this->get_logger(), "Suction pose at camera link: %f %f %f", _sucpose.position.x, _sucpose.position.y, _sucpose.position.z);
    RCLCPP_INFO(this->get_logger(), "Suction pose at base link: %f %f %f", suction_pose_base_link.position.x, suction_pose_base_link.position.y, suction_pose_base_link.position.z);
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = base_link_name;
    pose_msg.pose = suction_pose_base_link;
    _pose_pub->publish(pose_msg);
    
    _segmask_received = false;
    _sucpose_received = false;
    _service_count--;
    RCLCPP_INFO(this->get_logger(), "Service count After _get_suc_pose: %d", _service_count);
}

void PerceptionManager::_get_drop_pose(const std::shared_ptr<orbiter_bt::srv::GetDropPose::Request> request,
                           std::shared_ptr<orbiter_bt::srv::GetDropPose::Response> response){
    RCLCPP_INFO(this->get_logger(), "Perception Manager Got Request for Drop Pose");
    _service_count++;
    auto droppose_request = std::make_shared<perception_interfaces::srv::Droppose::Request>();
    droppose_request->color_image = _color_image;
    droppose_request->aruco_id = request->aruco_id;

    while (!_droppose_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto droppose_result = _droppose_client->async_send_request(droppose_request,
         std::bind(&PerceptionManager::_drop_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Drop pose service called");
    _client_count++;
    droppose_result.wait_for(std::chrono::seconds(5));

    response->pose = _droppose;

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = base_link_name;
    pose_msg.pose = _droppose;
    RCLCPP_INFO(this->get_logger(), "Drop pose published");
    _pose_pub->publish(pose_msg);
    _service_count--;
    RCLCPP_INFO(this->get_logger(), "Service count After _get_drop_pose: %d", _service_count);
}

void PerceptionManager::_find_x(const std::shared_ptr<perception_interfaces::srv::FindX::Request> request,
                           std::shared_ptr<perception_interfaces::srv::FindX::Response> response){

    RCLCPP_INFO(this->get_logger(), "Perception Manager Got Request Find X...");
    _service_count++;
    if (request->object == "aruco"){
        RCLCPP_INFO(this->get_logger(), "Finding Aruco in Frame...");
        auto find_aruco_in_frame_request = std::make_shared<perception_interfaces::srv::FindObjInFrame::Request>();
        if (_color_image.data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Color image is empty while calling find aruco in frame service");
            sleep(1);
        }
        else if (_color_image.encoding != "rgb8") {
            RCLCPP_ERROR(this->get_logger(), "Color image encoding is not rgb8 while calling find aruco in frame service");
            sleep(1);
        }
        
        find_aruco_in_frame_request->image = _color_image;
        find_aruco_in_frame_request->id = request->id;
        RCLCPP_INFO(this->get_logger(), "Waiting for find aruco in frame service...");

        auto future = _find_aruco_in_frame_client->async_send_request(find_aruco_in_frame_request,
            std::bind(&PerceptionManager::_find_aruco_in_frame_callback, this, std::placeholders::_1));
        _client_count++;
        future.wait_for(std::chrono::seconds(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "Aruco sending Find X at %d %d", _obj_frame_x, _obj_frame_y);
        response->x = _obj_frame_x;
        response->y = _obj_frame_y;
    }
    else if (request->object == "box"){
        RCLCPP_INFO(this->get_logger(), "Finding Box in Frame...");
        auto find_box_in_frame_request = std::make_shared<perception_interfaces::srv::FindObjInFrame::Request>();
        if (_color_image.data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Color image is empty while calling find box in frame service");
            sleep(1);
        }
        else if (_color_image.encoding != "rgb8") {
            RCLCPP_ERROR(this->get_logger(), "Color image encoding is not rgb8 while calling find box in frame service");
            sleep(1);
        }
        
        find_box_in_frame_request->image = _color_image;
        RCLCPP_INFO(this->get_logger(), "Waiting for find box in frame service...");

        auto future = _find_box_in_frame_client->async_send_request(find_box_in_frame_request,
            std::bind(&PerceptionManager::_find_box_in_frame_callback, this, std::placeholders::_1));
        _client_count++;
        future.wait_for(std::chrono::seconds(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        RCLCPP_INFO(this->get_logger(), "Box sending Find X at %d %d", _obj_frame_x, _obj_frame_y);
        response->x = _obj_frame_x;
        response->y = _obj_frame_y;
    }

    else{
        RCLCPP_ERROR(this->get_logger(), "Unknown object type");
    }
    _service_count--;
    return;
}

static bool g_shutdown = false;

// Signal handler function
void signalHandler(int signum) {
    if (signum == SIGINT) {
        g_shutdown = true;
    }
}

int main(int argc, char * argv[])
{
    // Register signal handler
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto perception_manager = std::make_shared<PerceptionManager>();
    executor.add_node(perception_manager);
    
    // Spin until shutdown is requested
    executor.spin();
    
    std::cout << "Perception Manager shutting down!!!!" << std::endl;
    rclcpp::shutdown();
    executor.cancel();
    executor.remove_node(perception_manager);
    std::cout << "Perception Manager shutdown complete" << std::endl;
    return 0;
}