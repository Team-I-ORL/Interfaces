#include "orbiter_bt_node.h"

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("orbiter_bt") + "/bt_xml";

OrbiterBTNode::OrbiterBTNode(const std::string &name) : Node(name)
{
    RCLCPP_INFO(get_logger(), "OrbiterBTNode has been created.");
    this->declare_parameter("inventory_file","none");
}

OrbiterBTNode::~OrbiterBTNode()
{
    delete this;
    std::cout << "OrbiterBTNode is being destroyed" << std::endl;
}

void OrbiterBTNode::setup()
{
    // initialize
    creatBT();
    const auto period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(period, std::bind(&OrbiterBTNode::updateBT, this));
    std::cout << "OrbiterBTNode setup complete" << std::endl;
}

void OrbiterBTNode::creatBT()
{
    // create the behavior tree
    BT::BehaviorTreeFactory factory;
    BT::NodeBuilder builder;
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToPose>("GoToPose", builder);
    
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<GetItemInfo>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<GetItemInfo>("getItemInfo", builder);

    // RCLCPP_INFO(get_logger(), "MoveArm_Wrapper creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<MoveArm_Wrapper>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<MoveArm_Wrapper>("MoveArm_Wrapper", builder);

    RCLCPP_INFO(get_logger(), "Move Arm CuRobo creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<MoveArm_CuRobo>(name, config, shared_from_this());
    };
    factory.registerBuilder<MoveArm_CuRobo>("MoveArm_CuRobo", builder);

    RCLCPP_INFO(get_logger(), "getItemPose creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<GetItemPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GetItemPose>("getItemPose", builder);

    // RCLCPP_INFO(get_logger(), "getDropPose creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<GetDropPose>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<GetDropPose>("getDropPose", builder);
   
    // RCLCPP_INFO(get_logger(), "vendingMachine creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<ActuateVendingMachine>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<ActuateVendingMachine>("vendingMachineActuate", builder);

    RCLCPP_INFO(get_logger(), "waitUntilActivate creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<wait_until_activate>(name, config, shared_from_this());
    };
    factory.registerBuilder<wait_until_activate>("waitUntilActivate", builder);

    // RCLCPP_INFO(get_logger(), "arucoArmPose creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<arucoArmPose>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<arucoArmPose>("arucoArmPose", builder);

    RCLCPP_INFO(get_logger(), "checkArmGoal creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<checkArmGoal>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<checkArmGoal>("armGoalIsEmpty", builder);

    // RCLCPP_INFO(get_logger(), "clearInputs creating");
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<clearInputs>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<clearInputs>("clearInputs", builder);

    // RCLCPP_INFO(get_logger(), "repetitionManager creating"); 
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<repetition_manager>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<repetition_manager>("repetitionManager", builder);

    // RCLCPP_INFO(get_logger(), "randAng creating"); 
    // builder = 
    //     [=](const std::string &name, const BT::NodeConfiguration &config) {
    //     return std::make_unique<RandomizeYaw>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<RandomizeYaw>("randAng", builder);

    RCLCPP_INFO(get_logger(), "SuctionCmd creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<SuctionCmd>(name, config, shared_from_this());
    };
    factory.registerBuilder<SuctionCmd>("suctionCmd", builder);

    RCLCPP_INFO(get_logger(), "MoveHead creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<MoveHead>(name, config, shared_from_this());
    };
    factory.registerBuilder<MoveHead>("moveHead", builder);

    RCLCPP_INFO(get_logger(), "GetNextAction creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<GetNextAction>(name, config, shared_from_this());
    };
    factory.registerBuilder<GetNextAction>("getNextAction", builder);

    RCLCPP_INFO(get_logger(), "SingFinished creating");
    builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<SingFinished>(name, config, shared_from_this());
    };
    factory.registerBuilder<SingFinished>("signalRestockingFinished", builder);

    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/testing_sequencial.xml");
    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/testing_full_fallback.xml");
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/getsuc_then_move.xml");
    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/fall.xml");


    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/testing_nonbot.xml");

    printTreeRecursively(tree_.rootNode());

    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/testing_nonbot.xml");
    std::cout << "Behavior tree created" << std::endl;
}

void OrbiterBTNode::updateBT()
{
    // tick the behavior tree when asked
    BT::NodeStatus tree_status = tree_.tickRoot();
    // BT::NodeStatus tree_status = tree_.tickOnce(); // V4
    if (tree_status == BT::NodeStatus::SUCCESS)
    {   
        RCLCPP_INFO(get_logger(), "Behavior tree executed successfully");
    }
    else if (tree_status == BT::NodeStatus::RUNNING)
    {
        // RCLCPP_INFO(get_logger(), "Behavior tree is running");
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(get_logger(), "Behavior tree failed");
        timer_->cancel();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<OrbiterBTNode>("orbiter_bt_node");

    // std::cout << "Node pointer 1: " << node.get() << std::endl;
    // BT::PublisherZMQ publisher_zmq(node->tree_);

    node->setup();
    // BT::PublisherZMQ publisher_zmq(node->tree_);
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}