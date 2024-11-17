#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "orbiter_bt/srv/get_suc_pose.hpp"
#include "orbiter_bt/srv/get_drop_pose.hpp"
#include "bt_string_serialize.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>
#include "std_msgs/msg/bool.hpp"

class GetItemPose : public BT::StatefulActionNode{
public:
    GetItemPose(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedPtr suc_client;
    rclcpp::Client<orbiter_bt::srv::GetDropPose>::SharedPtr drop_client;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_scene_pub;
    std::string type;
    bool suc_finished = false;
    bool suc_success = false;
    bool drop_finished = false;
    bool drop_success = false;

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    void suc_result_callback(rclcpp::Client<orbiter_bt::srv::GetSucPose>::SharedFuture result);
    void drop_result_callback(rclcpp::Client<orbiter_bt::srv::GetDropPose>::SharedFuture result);
};

