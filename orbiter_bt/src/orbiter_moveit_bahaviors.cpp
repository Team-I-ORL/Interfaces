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
    std::cout << "MoveArm node creating" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "MoveArm has been created.");
    RCLCPP_INFO(node_->get_logger(), "MoveArm onStart");
    EE_LINK = move_group_interface.getEndEffectorLink();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", EE_LINK.c_str());

}

BT::PortsList MoveArm::providedPorts()
{
    return {
        BT::InputPort<std::string>("arm_goal"),
        // BT::OutputPort<std::string>("result")
        };
}

bool MoveArm::goalChecker(){
    // rclcpp::Time now = node_->get_clock()->now();
    // auto curPose = tf_buffer_->lookupTransform(BASE_LINK, EE_LINK, now, 100);
    if (!tfBuffer.canTransform(BASE_LINK, EE_LINK, tf2::TimePointZero)) {
        RCLCPP_ERROR(node_->get_logger(), "Transform not available !!!");
        return false;
    }

    auto curPose = tfBuffer.lookupTransform(BASE_LINK, EE_LINK,tf2::TimePointZero);
    double dx = target_pose.pose.position.x - curPose.transform.translation.x;
    double dy = target_pose.pose.position.y - curPose.transform.translation.y;
    double dz = target_pose.pose.position.z - curPose.transform.translation.z;

    double quatx = target_pose.pose.orientation.x - curPose.transform.rotation.x;
    double quaty = target_pose.pose.orientation.y - curPose.transform.rotation.y;   
    double quatz = target_pose.pose.orientation.z - curPose.transform.rotation.z;
    double quatw = target_pose.pose.orientation.w - curPose.transform.rotation.w;

    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    double toleance = move_group_interface.getGoalPositionTolerance();
    double rotTolerance = move_group_interface.getGoalOrientationTolerance();
    
    // std::cout << "ROT tolerance is " << rotTolerance << std::endl;
    // std::cout << "Quaternion error is " << quatx << " " << quaty << " " << quatz << " " << quatw << std::endl;
    // std::cout << "RPY Error is" << roll << " " << pitch << " " << yaw << std::endl;
    
    if (dist < toleance && quatx < rotTolerance && quaty < rotTolerance && quatz < rotTolerance && quatw < rotTolerance){
        move_group_interface.stop();
        move_group_interface.clearPoseTargets();
        // DONT EVER DELETE THE DELAY !!!!! MOVE GROUP NEED TIME TO STOP !!!!!
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(node_->get_logger(), "Goal Reached !!!");
        
        return true;
    }

    return false;
}

BT::NodeStatus MoveArm::onStart()
{   
    // // Get goal from input port
    BT::Optional<std::string> goal = getInput<std::string>("arm_goal");
    if (!goal)
    {
        throw BT::RuntimeError("missing required input [arm_goal]");
    }

    std::vector<double> goalVec = bt_string_serialize::stringToVector(goal.value());
    double x = goalVec[0];
    double y = goalVec[1];
    double z = goalVec[2];

    // Adding orientation constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "grippper_link";
    ocm.header.frame_id = "map";
    ocm.orientation.x = 0.000;
    ocm.orientation.y = 0.707;
    ocm.orientation.z = 0.000;
    ocm.orientation.w = 0.707;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);


    geometry_msgs::msg::PoseStamped target_map;
    target_map.header.frame_id = "map";  // Set the frame ID
    target_map.pose.position.x = x;          // Set desired X position
    target_map.pose.position.y = y;          // Set desired Y position
    target_map.pose.position.z = z;          // Set desired Z position
    target_map.pose.orientation.w = 1.0;
    if (!tfBuffer.canTransform("map", BASE_LINK, tf2::TimePointZero)) {
        RCLCPP_ERROR(node_->get_logger(), "Transform not available !!!");
        return BT::NodeStatus::FAILURE;
    }
    tfBuffer.transform(target_map, target_pose, BASE_LINK);

    move_group_interface.setPoseTarget(target_pose);  
    RCLCPP_INFO(node_->get_logger(), "Goal created at: x=%f, y=%f, z=%f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    if (goalChecker()){
        RCLCPP_INFO(node_->get_logger(), "Already at Pose Not Planning");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "Goal not reached");
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
        // std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5 second delay
        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::RUNNING;
    }
}


