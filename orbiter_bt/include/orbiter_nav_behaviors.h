#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
// sudo apt-get install ros-humble-tf2-geometry-msgs
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

class GoToPose : public BT::StatefulActionNode // using async action
{
public:
    GoToPose(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_; 

    // overides for the BT::StatefulActionNode
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};
    // std::vector<double> getGoal();

    bool nav_done_flag;

    // getting results
    void navigate_to_pose_callback(const GoalHandleNav::WrappedResult &result);

    std::vector<double> stringToVector(const std::string& s) {
        std::string numbers = s;
        numbers.erase(std::remove(numbers.begin(), numbers.end(), '['), numbers.end());
        numbers.erase(std::remove(numbers.begin(), numbers.end(), ']'), numbers.end());

        std::vector<double> result;
        std::stringstream ss(numbers);

        for (double i; ss >> i;) {
            result.push_back(i);
            if (ss.peek() == ',')
                ss.ignore();
        }

        return result;
    }
};