#include "orbiter_bt_node.h"

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("orbiter_bt") + "/bt_xml";

OrbiterBTNode::OrbiterBTNode(const std::string &name) : Node(name)
{
    RCLCPP_INFO(get_logger(), "OrbiterBTNode has been created.");
    this->declare_parameter("inventory_file","none");
}

void OrbiterBTNode::setup()
{
    // initialize
    creatBT();
    const auto period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(period, std::bind(&OrbiterBTNode::updateBT, this));
}

void OrbiterBTNode::creatBT()
{
    // create the behavior tree
    BT::BehaviorTreeFactory factory;
    BT::NodeBuilder builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToPose>("GoToPose", builder);
    // tree_ = factory.createTreeFromFile(bt_xml_dir + "/orbiter_bt.xml");
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/orbiter_patrol.xml");
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
    auto node = std::make_shared<OrbiterBTNode>("orbiter_bt_node");
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}