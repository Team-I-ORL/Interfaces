#include "orbiter_moveit_behaviors.h"


MoveArm::MoveArm(const std::string &name,
                 const BT::NodeConfiguration &config,
                 const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
    node_(node),
    move_group_interface(node, PLANNING_GROUP),
    tfBuffer(node->get_clock()),
    tf_listener(tfBuffer)
{
    RCLCPP_INFO(node_->get_logger(), "MoveArm has been created.");
    RCLCPP_INFO(node_->get_logger(), "MoveArm onStart");
    EE_LINK = move_group_interface.getEndEffectorLink();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", EE_LINK.c_str());

    // std::cout << "MoveGroup pointer: " << move_group_interface.getNodeHandle() << std::endl;
}

BT::PortsList MoveArm::providedPorts()
{
    return {
        // BT::InputPort<std::string>("goal"),
        // BT::OutputPort<std::string>("result")
        };
}

bool MoveArm::goalChecker(){
    auto curPose = tfBuffer.lookupTransform(BASE_LINK, EE_LINK,tf2::TimePointZero);
    double dx = target_pose.pose.position.x - curPose.transform.translation.x;
    double dy = target_pose.pose.position.y - curPose.transform.translation.y;
    double dz = target_pose.pose.position.z - curPose.transform.translation.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    double toleance = move_group_interface.getGoalPositionTolerance();
    if (dist < toleance){
        RCLCPP_INFO(node_->get_logger(), "Goal Reached !!!");
        return true;
    }

    return false;
}

BT::NodeStatus MoveArm::onStart()
{   
    // // Get goal from input port
    // BT::Optional<std::string> goal = getInput<std::string>("goal");
    // if (!goal)
    // {
    //     throw BT::RuntimeError("missing required input [goal]");
    // }
    // RCLCPP_INFO(node_->get_logger(), "Moving to %s", goal.value().c_str());    
    // Send goal to action server
    // std::cout << "Node pointer: " << node_.get() << std::endl;
    // std::cout << "Node Use Count: " << node_.use_count() << std::endl;

    target_pose.header.frame_id = "base_link";  // Set the frame ID
    target_pose.pose.position.x = 0.3;          // Set desired X position
    target_pose.pose.position.y = 0.03;          // Set desired Y position
    target_pose.pose.position.z = 0.93;          // Set desired Z position
    target_pose.pose.orientation.w = 1.0;
    move_group_interface.setPoseTarget(target_pose);  
    RCLCPP_INFO(node_->get_logger(), "Goal created at: x=%f, y=%f, z=%f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode planResult = move_group_interface.plan(my_plan);
    if (planResult == moveit::core::MoveItErrorCode::SUCCESS){
        RCLCPP_INFO(node_->get_logger(), "Planning Succeeded !!!");
        auto executionStarted = move_group_interface.asyncExecute(my_plan);
        if (executionStarted == moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO(node_->get_logger(), "Execution Started !!!");
        }
        else {
            RCLCPP_WARN(node_->get_logger(), "Execution FAILED !!!!");
            return BT::NodeStatus::FAILURE;
        }
    }
    else {
        RCLCPP_WARN(node_->get_logger(), "Planning FAILED !!!!");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArm::onRunning()
{   
    bool checkPoseResult = goalChecker();
    if (checkPoseResult){
        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::RUNNING;
    }
}


