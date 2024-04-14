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
        BT::BidirectionalPort<std::string>("cur_repeat")
    };
}

BT::NodeStatus repetition_manager::tick()
{
    std::vector<int> ids = bt_string_serialize::stringToVectorInt(getInput<std::string>("ids").value());
    int num_repeats = ids.size();
    int cur_repeat = bt_string_serialize::stringToInt(getInput<std::string>("cur_repeat").value());
    if (cur_repeat < num_repeats)
    {
        RCLCPP_INFO(node_->get_logger(), "Repeating %d out of %d", cur_repeat, num_repeats);
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