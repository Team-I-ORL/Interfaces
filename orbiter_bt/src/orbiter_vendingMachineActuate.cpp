#include "orbiter_vendingMachineActuate.h"

ActuateVendingMachine::ActuateVendingMachine(const std::string &name,
                                             const BT::NodeConfiguration &config,
                                             rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "ActuateVendingMachine has been created.");
    vending_machine_client = node_->create_client<ims_interfaces::srv::VendingMachine>("vending_machine_service");
}

BT::PortsList ActuateVendingMachine::providedPorts()
{
    return {
        BT::InputPort<std::string>("vending_machine_id"),
        BT::InputPort<std::string>("quantity_dispense"),
    };
}

BT::NodeStatus ActuateVendingMachine::onStart(){

    if (!vending_machine_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Vending machine service not available after waiting");
        return BT::NodeStatus::FAILURE;
    }
    finished = false;
    BT::Optional<std::string> vending_machine_id = getInput<std::string>("vending_machine_id");
    int id_request = bt_string_serialize::stringToInt(vending_machine_id.value());
    auto request = std::make_shared<ims_interfaces::srv::VendingMachine::Request>();
    request->vending_machine_id = id_request;
    request->quantity = bt_string_serialize::stringToInt(getInput<std::string>("quantity_dispense").value());
    auto result = vending_machine_client->async_send_request(request, std::bind(&ActuateVendingMachine::result_callback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Vending Machine Request sent, waiting for response.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActuateVendingMachine::onRunning(){
    if (finished){
        if (successful){
            return BT::NodeStatus::SUCCESS;
        }
        else{
            return BT::NodeStatus::FAILURE;
        }
    }
    else{
        return BT::NodeStatus::RUNNING;
    }
}

void ActuateVendingMachine::result_callback(rclcpp::Client<ims_interfaces::srv::VendingMachine>::SharedFuture result){
    RCLCPP_INFO(node_->get_logger(), "Response received");
    auto response = result.get();
    if (response->success){
        RCLCPP_INFO(node_->get_logger(), "Vending Machine Actuation successful");
        finished = true;
        successful = true;
        return;
    }
    else{
        RCLCPP_ERROR(node_->get_logger(), "Failed to dispense items from vending machine");
        finished = true;
        successful = false;
    }
}