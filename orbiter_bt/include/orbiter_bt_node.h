#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "orbiter_nav_behaviors.h"
#include "orbiter_moveit_behaviors.h"
#include "orbiter_moveit_behaviors_wrapper.h"
#include "orbiter_vendingMachineActuate.h"
#include "orbiter_getItemInfo.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
// #include "behaviortree_cpp/bt_factory.h"

class OrbiterBTNode : public rclcpp::Node
{
private:
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    ~OrbiterBTNode();
    explicit OrbiterBTNode(const std::string &name);
    void setup();
    void creatBT();
    void updateBT();
};
