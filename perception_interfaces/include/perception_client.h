#include "rclcpp/rclcpp.hpp"
#include "perception_interfaces/srv/segmask.hpp"
#include "perception_interfaces/srv/sucpose.hpp"
#include "orbiter_bt/srv/get_suc_pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class PerceptionClient : public rclcpp::Node
{   
    private:
        rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedPtr _get_suc_pose_client;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _rgb_subscription;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depth_subscription;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_subscription;

        sensor_msgs::msg::Image rgb_image;
        sensor_msgs::msg::Image depth_image;
        sensor_msgs::msg::CameraInfo camera_info;

        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            rgb_image = *msg;
        };
        void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            depth_image = *msg;
        };
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            camera_info = *msg;
        };
    public:
        void send_request();
        PerceptionClient();

};