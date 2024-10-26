#include "rclcpp/rclcpp.hpp"
#include "perception_interfaces/srv/segmask.hpp"
#include "perception_interfaces/srv/sucpose.hpp"
#include "perception_interfaces/srv/find_x.hpp"
#include "perception_interfaces/srv/find_obj_in_frame.hpp"
#include "orbiter_bt/srv/get_suc_pose.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PerceptionManager : public rclcpp::Node
{   
    private:
        rclcpp::Service<orbiter_bt::srv::GetSucPose>::SharedPtr _get_suc_pose_service;
        rclcpp::Service<perception_interfaces::srv::FindX>::SharedPtr _find_x_service;

        rclcpp::Client<perception_interfaces::srv::Segmask>::SharedPtr _segmask_client;
        rclcpp::Client<perception_interfaces::srv::Sucpose>::SharedPtr _sucpose_client;
        rclcpp::Client<perception_interfaces::srv::FindObjInFrame>::SharedPtr _find_aruco_in_frame_client;
        rclcpp::Client<perception_interfaces::srv::FindObjInFrame>::SharedPtr _find_box_in_frame_client;
        rclcpp::CallbackGroup::SharedPtr _client_callback_group;
        rclcpp::CallbackGroup::SharedPtr _service_callback_group;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _color_image_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depth_image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _suc_pose_pub;
        rclcpp::CallbackGroup::SharedPtr _subscription_callback_group;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf_listener;
        tf2_ros::TransformBroadcaster tf_broadcaster;

        std::string base_link_name;
        std::string camera_link_name;
        std::string segmask_service_name;
        std::string sucpose_service_name;

        void _seg_mask_callback(rclcpp::Client<perception_interfaces::srv::Segmask>::SharedFuture response){
            RCLCPP_INFO(this->get_logger(), "Segmentation mask service received");
            _segmask = response.get()->segmask;
            _segmask_received = true;
        }
        
        void _suc_pose_callback(rclcpp::Client<perception_interfaces::srv::Sucpose>::SharedFuture response){
            RCLCPP_INFO(this->get_logger(), "Suction pose service received");
            _sucpose = response.get()->pose;
            _sucpose_received = true;
        }

        void _find_aruco_in_frame_callback(rclcpp::Client<perception_interfaces::srv::FindObjInFrame>::SharedFuture response){
            RCLCPP_INFO(this->get_logger(), "Find Aruco in frame service received");
            _obj_frame_x = response.get()->x;
            _obj_frame_y = response.get()->y;
        }

        void _find_box_in_frame_callback(rclcpp::Client<perception_interfaces::srv::FindObjInFrame>::SharedFuture response){
            RCLCPP_INFO(this->get_logger(), "Find Box in frame service received");
            _obj_frame_x = response.get()->x;
            _obj_frame_y = response.get()->y;
        }

        void _color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            if (msg->data.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received blank color image");
            } else {
                RCLCPP_INFO(this->get_logger(), "Received img");
                _color_image = *msg;
            }
        }

        void _depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            if (msg->data.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received blank depth image");
            } else {
                _depth_image = *msg;
            }
        }

        void _camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            if (msg->d.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received blank camera info");
            } else {
                _camera_info = *msg;
            }
        }

        sensor_msgs::msg::Image _segmask;
        sensor_msgs::msg::Image _color_image;
        sensor_msgs::msg::Image _depth_image;
        sensor_msgs::msg::CameraInfo _camera_info;
        bool _segmask_received = false;
        geometry_msgs::msg::Pose _sucpose;
        bool _sucpose_received = false;
        int _obj_frame_x = -1;
        int _obj_frame_y = -1;
        
        void _get_suc_pose(const std::shared_ptr<orbiter_bt::srv::GetSucPose::Request> request,
                           std::shared_ptr<orbiter_bt::srv::GetSucPose::Response> response);

        void _find_x(const std::shared_ptr<perception_interfaces::srv::FindX::Request> request,
                           std::shared_ptr<perception_interfaces::srv::FindX::Response> response);
    public:
        PerceptionManager();

};