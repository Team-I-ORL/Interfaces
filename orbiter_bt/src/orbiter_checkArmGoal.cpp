#include "orbiter_checkArmGoal.h"
checkArmGoal::checkArmGoal(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
 : BT::ConditionNode(name, config), node_(node){
    RCLCPP_INFO(node_->get_logger(), "checkArmGoal has been created.");
}

BT::PortsList checkArmGoal::providedPorts(){
    return{BT::InputPort<std::string>("arm_goal")};
}
BT::NodeStatus checkArmGoal::tick(){
    RCLCPP_INFO(node_->get_logger(), "Checking arm goal.");
    BT::Optional<std::string> arm_goal = getInput<std::string>("arm_goal");
    if (!arm_goal){
        RCLCPP_WARN(node_->get_logger(), "Missing required input [arm_goal]");
        return BT::NodeStatus::FAILURE;
    }

    auto arm_goal_vector = bt_string_serialize::stringToVector(arm_goal.value());
    if (arm_goal_vector == std::vector<double>{0.0, 0.0, 0.0}){
        RCLCPP_INFO(node_->get_logger(), "No arm goal detected !!");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Arm goal detected !!");
    return BT::NodeStatus::SUCCESS;
}