#include "rclcpp/rclcpp.hpp"
#include "orbiter_bt/srv/ims.hpp"
#include <memory>
#include <string>

class outputItemInfo : public rclcpp::Node
{
public:
    explicit outputItemInfo(const std::string &name);
    void itemInfoCallback(const std::shared_ptr<orbiter_bt::srv::Ims::Request> request,
                          std::shared_ptr<orbiter_bt::srv::Ims::Response> response);
    rclcpp::Service<orbiter_bt::srv::Ims>::SharedPtr server_;
};
