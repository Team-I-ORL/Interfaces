#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "orbiter_nav_behaviors.h"
// #include "orbiter_moveit_behaviors.h"
// #include "orbiter_moveit_behaviors_wrapper.h"
#include "orbiter_moveArmCuRobo.h"
// #include "orbiter_getSucPose.h"
// #include "orbiter_getDropPose.h"
#include "orbiter_getItemPose.h"
#include "orbiter_waitUntilActivate.h"
// #include "orbiter_vendingMachineActuate.h"
#include "orbiter_getNextAction.h"
#include "orbiter_signalFinish.h"
// #include "orbiter_getItemInfo.h"
// #include "orbiter_getVendingLoc.h"
// #include "orbiter_checkArmGoal.h"
// #include "orbiter_clearInputs.h"
// #include "orbiter_repetitionManager.h"
#include "orbiter_suck.h"
#include "orbiter_moveHead.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
// #include "behaviortree_cpp/bt_factory.h"
// #include "orbiter_nav_yaw_rand.h"
class OrbiterBTNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
public:
    BT::Tree tree_;
    ~OrbiterBTNode();
    explicit OrbiterBTNode(const std::string &name);
    void setup();
    void creatBT();
    void updateBT();
};
