#include "rclcpp/rclcpp.hpp"
#include "perception_interfaces/srv/segmask.hpp"
#include "perception_interfaces/srv/sucpose.hpp"
#include "orbiter_bt/srv/get_suc_pose.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PerceptionManager : public rclcpp::Node
{   
    private:
        rclcpp::Service<orbiter_bt::srv::GetSucPose>::SharedPtr _get_suc_pose_service;
        rclcpp::Client<perception_interfaces::srv::Segmask>::SharedPtr _segmask_client;
        rclcpp::Client<perception_interfaces::srv::Sucpose>::SharedPtr _sucpose_client;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf_listener;
        tf2_ros::TransformBroadcaster tf_broadcaster;

        std::string base_link_name;
        std::string camera_link_name;
        std::string segmask_service_name;
        std::string sucpose_service_name;
        
        void _get_suc_pose(const std::shared_ptr<orbiter_bt::srv::GetSucPose::Request> request,
                           std::shared_ptr<orbiter_bt::srv::GetSucPose::Response> response);
    public:
        PerceptionManager();

};