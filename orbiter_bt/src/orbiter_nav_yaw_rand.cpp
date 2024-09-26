#include "orbiter_nav_yaw_rand.h"

RandomizeYaw::RandomizeYaw(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "RandomizeYaw has been created.");
}

BT::PortsList RandomizeYaw::providedPorts()
{
    return {
        BT::BidirectionalPort<std::string>("yaw"),
        BT::InputPort<std::string>("randomize_range")
    };
}

BT::NodeStatus RandomizeYaw::tick()
{
    // Randomize yaw
    std::random_device rd;
    std::mt19937 gen(rd());
    int range = bt_string_serialize::stringToInt(getInput<std::string>("randomize_range").value());
    std::uniform_real_distribution<> dis(0, range);
    int yaw_offset = dis(gen);
    if (leftAlready)
    {
        yaw_offset = -yaw_offset;
        leftAlready = false;
    }
    int yaw = bt_string_serialize::stringToInt(getInput<std::string>("yaw").value());
    yaw += yaw_offset;
    setOutput("yaw", std::to_string(yaw));
    RCLCPP_INFO(node_->get_logger(), "Randomized yaw: %d", yaw);
    leftAlready = true;
    return BT::NodeStatus::SUCCESS;
}