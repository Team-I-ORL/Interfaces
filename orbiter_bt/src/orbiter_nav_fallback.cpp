#include "orbiter_nav_fallback.h"

NavFallback::NavFallback(const std::string &name,
                     const BT::NodeConfiguration &config,
                     rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    client(node->create_client<orbiter_bt::srv::CmdVelDist>("cmd_vel_dist"))
{
    RCLCPP_INFO(node_->get_logger(), "NavFallback has been created");
    moved_pub = node_->create_publisher<std_msgs::msg::Bool>("moved", 5);
}

BT::PortsList NavFallback::providedPorts()
{
    return {
        BT::InputPort<std::string>("type"),
        BT::InputPort<std::string>("pose"),
        BT::InputPort<std::string>("fwd_dist"),
        BT::InputPort<std::string>("spin_dist")
    };
}

BT::NodeStatus NavFallback::onStart()
{   
    finished = false;
    success = false;

    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available!");
        return BT::NodeStatus::FAILURE;
    }

    std::string type;
    if (!getInput<std::string>("type", type)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [type]");
        return BT::NodeStatus::FAILURE;
    }
    
    std::string fwd_dist;
    std::string spin_dist;
    std::string posestr;
    std_msgs::msg::Bool msg;
    msg.data = true;
    moved_pub->publish(msg);
    if (type == "open_loop"){
        if (!getInput<std::string>("fwd_dist", fwd_dist)) {
            fwd_dist = "0.0";
        }
        if (!getInput<std::string>("spin_dist", spin_dist)) {
            spin_dist = "0.0";
        }
        float fwd = std::stof(fwd_dist);
        float spin = std::stof(spin_dist);
        auto request = std::make_shared<orbiter_bt::srv::CmdVelDist::Request>();
        request->fwd_dist = fwd;
        request->spin_dist = spin;
        auto result = client->async_send_request(request, std::bind(&NavFallback::result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for open loop nav fallback, with fwd: %f, spin: %f", fwd, spin);
    }
    else if (type == "aruco"){
        if (!getInput<std::string>("pose", posestr)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input [pose]");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<std::string>("fwd_dist", fwd_dist)) {
            fwd_dist = "0.8";
            RCLCPP_WARN(node_->get_logger(), "Using default fwd_dist of 0.8 for aruco fallback");
        }
        auto request = std::make_shared<orbiter_bt::srv::CmdVelDist::Request>();

        auto pose = bt_string_serialize::stringToPose(posestr);
        float x = pose.position.x;
        float y = pose.position.y;

        bool has_spin = fwd_dist.back() == 's';
        float fwd_dist_val = std::stof(has_spin ? fwd_dist.substr(0, fwd_dist.length()-1) : fwd_dist);
        float fwd = has_spin ? sqrt(x*x + y*y) - fwd_dist_val : x - fwd_dist_val;
        float spin = has_spin ? std::atan2(y, pose.position.x) : 0.0f;
        request->fwd_dist = fwd;
        request->spin_dist = spin;
        auto result = client->async_send_request(request, std::bind(&NavFallback::result_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Request sent for aruco fallback, with fwd: %f, spin: %f", fwd, spin);
    }

    else{
        RCLCPP_ERROR(node_->get_logger(), "Invalid type input, must be 'open_loop' or 'aruco'");
        return BT::NodeStatus::FAILURE;
        
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavFallback::onRunning()
{
    if (finished) {
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "NavFallback finished successfully");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "NavFallback failed");
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void NavFallback::result_callback(rclcpp::Client<orbiter_bt::srv::CmdVelDist>::SharedFuture result)
{
    auto response = result.get();
    if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "NavFallback command successful");
        finished = true;
        success = true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "NavFallback command failed");
        finished = true;
        success = false;
    }
}