#include "orbiter_repetitionManager.h"

repetition_manager::repetition_manager(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "Creating repetition_manager node");
}

BT::PortsList repetition_manager::providedPorts()
{
    return {
        BT::InputPort<std::string>("ids"),
        BT::OutputPort<std::string>("id"),
        BT::OutputPort<int>("num_repeats"),
        BT::BidirectionalPort<std::string>("cur_repeat")
    };
}

BT::NodeStatus repetition_manager::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Repetition manager tick");
    RCLCPP_INFO(node_->get_logger(), "IDs are: %s", getInput<std::string>("ids").value().c_str());
    std::vector<int> ids = bt_string_serialize::stringToVectorInt(getInput<std::string>("ids").value());
    int num_repeat = ids.size();
    setOutput("num_repeats",num_repeat);
    // RCLCPP_INFO(node_->get_logger(), "Number of repeats: %d", num_repeat);
    int cur_repeat = bt_string_serialize::stringToInt(getInput<std::string>("cur_repeat").value());
    if (cur_repeat < num_repeat)
    {
        RCLCPP_INFO(node_->get_logger(), "Repeating %d out of %d", cur_repeat, num_repeat);
        setOutput("id", std::to_string(ids[cur_repeat]));
        setOutput("cur_repeat", std::to_string(cur_repeat+1));

        return BT::NodeStatus::FAILURE;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Repetition complete");
        return BT::NodeStatus::SUCCESS;
    }
}