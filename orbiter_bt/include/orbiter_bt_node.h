#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "orbiter_nav_behaviors.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class OrbiterBTNode : public rclcpp::Node
{
private:
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    explicit OrbiterBTNode(const std::string &name);
    void setup();
    void creatBT();
    void updateBT();
};
