#include "orbiter_clearInputs.h"
clearInputs::clearInputs(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node){
    RCLCPP_INFO(node_->get_logger(), "clearInputs has been created.");
}

BT::PortsList clearInputs::providedPorts(){
    return{
        BT::OutputPort<std::string>("item_name"),
        BT::OutputPort<std::string>("arm_goal"),
        BT::OutputPort<std::string>("nav_goal"),
        BT::OutputPort<std::string>("yaw"),
        BT::OutputPort<std::string>("quantity"),
        BT::OutputPort<std::string>("vending_machine_id"),
        BT::OutputPort<std::string>("id"),
        BT::OutputPort<std::string>("ids"),
        BT::OutputPort<std::string>("cur_repeat")
    };
}

BT::NodeStatus clearInputs::tick(){
    RCLCPP_INFO(node_->get_logger(), "Clearing inputs.");
    setOutput<std::string>("item_name", "");
    setOutput<std::string>("arm_goal", "");
    setOutput<std::string>("nav_goal", "");
    setOutput<std::string>("yaw", "");
    setOutput<std::string>("quantity", "");
    setOutput<std::string>("vending_machine_id", "");
    setOutput<std::string>("id", "");
    setOutput<std::string>("ids", "");
    setOutput<std::string>("cur_repeat", "");
    return BT::NodeStatus::SUCCESS;
}