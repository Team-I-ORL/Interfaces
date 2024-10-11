#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "target_client/srv/target_pose.hpp"
#include <bt_string_serialize.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveArm_CuRobo : public BT::StatefulActionNode // using async action
{
    public:
        MoveArm_CuRobo(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<target_client::srv::TargetPose>::SharedPtr client;
        bool finished = false;
        bool move_arm_result = false;
        void result_callback(rclcpp::Client<target_client::srv::TargetPose>::SharedFuture result);

        // overides for the BT::StatefulActionNode
        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override{};
};