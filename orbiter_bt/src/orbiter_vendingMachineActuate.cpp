#include "orbiter_vendingMachineActuate.h"

ActuateVendingMachine::ActuateVendingMachine(const std::string &name,
                                             const BT::NodeConfiguration &config,
                                             rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "ActuateVendingMachine has been created.");
    // vending_machine_client = node_->create_client<ims_interfaces::srv::VendingMachine>("vending_machine_service");
    vending_machine_publisher = node_->create_publisher<std_msgs::msg::String>("ping/primitive", 10);
}

BT::PortsList ActuateVendingMachine::providedPorts()
{
    return {
        BT::InputPort<std::string>("vending_machine_id"),
        BT::InputPort<std::string>("quantity_dispense"),
    };
}

BT::NodeStatus ActuateVendingMachine::onStart() {
    // if (!vending_machine_client->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_ERROR(node_->get_logger(), "Vending machine service not available after waiting");
    //     return BT::NodeStatus::FAILURE;
    // }
    // finished = false;
    BT::Optional<std::string> vending_machine_id = getInput<std::string>("vending_machine_id");
    int vendor_id = bt_string_serialize::stringToInt(vending_machine_id.value());
    int quantity_requested = bt_string_serialize::stringToInt(getInput<std::string>("quantity_dispense").value());

    auto message = std_msgs::msg::String();
    int action = 1; int requestor = 69;
    std::string dat = "<" + std::to_string(vendor_id) + ":" + std::to_string(action) + ":" + std::to_string(requestor) + ">";
    message.data = dat;
    vending_machine_publisher->publish(message);
    RCLCPP_INFO(node_->get_logger(), "Vending Machine Request sent :-)");
    // finished = true;
    end_time = std::chrono::system_clock::now() + std::chrono::seconds(7);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActuateVendingMachine::onRunning(){
    if (std::chrono::system_clock::now() > end_time){
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::RUNNING;
    }
}

// void ActuateVendingMachine::result_callback(rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedFuture result){
//     RCLCPP_INFO(node_->get_logger(), "Response received");
//     auto response = result.get();
//     if (response->success){
//         RCLCPP_INFO(node_->get_logger(), "Vending Machine Actuation successful");
//         finished = true;
//         successful = true;
//         return;
//     }
//     else{
//         RCLCPP_ERROR(node_->get_logger(), "Failed to dispense items from vending machine");
//         finished = true;
//         successful = false;
//     }
// }