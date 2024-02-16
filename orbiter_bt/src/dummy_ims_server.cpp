#include "dummy_ims_server.h"

outputItemInfo::outputItemInfo(const std::string &name) : Node(name)
{
    server_ = this->create_service<orbiter_bt::srv::Ims>("ims_service",
        [this](const std::shared_ptr<orbiter_bt::srv::Ims::Request> request,
               std::shared_ptr<orbiter_bt::srv::Ims::Response> response) {
            this->itemInfoCallback(request, response);
        });
    RCLCPP_INFO(this->get_logger(), "IMS server has been created.");
}

void outputItemInfo::itemInfoCallback(const std::shared_ptr<orbiter_bt::srv::Ims::Request> request,
                                      std::shared_ptr<orbiter_bt::srv::Ims::Response> response)
{   
    RCLCPP_INFO(this->get_logger(), "Item Name: %s", request->itemname.c_str());


    response->iteminfo = "Item Info";
    response->loc = "Location";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<outputItemInfo>("ims_server_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
