cmake_minimum_required(VERSION 3.8)
project(orbiter_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(moveit_msgs REQUIRED)
# find_package(moveit REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ims_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED) 
# find_package(target_client REQUIRED)

# find_package(moveit_ros_planning_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Fetch.msg"
  "srv/Ims.srv"
  "srv/MoveArm.srv"
  "srv/GetSucPose.srv"
  "srv/MoveHead.srv"
  "srv/Suck.srv"
  "srv/GetDropPose.srv"
  "srv/NextAction.srv"
  "srv/MoveCam.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Install directories
install(DIRECTORY
bt_xml launch dummy_database
DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOR_SOURCES
src/orbiter_nav_behaviors.cpp
# src/orbiter_getItemInfo.cpp
# src/orbiter_vendingMachineActuate.cpp
# src/orbiter_moveit_bahaviors_wrapper.cpp
src/orbiter_waitUntilActivate.cpp
# src/orbiter_getVendingLoc.cpp
# src/orbiter_checkArmGoal.cpp
# src/orbiter_clearInputs.cpp
# src/orbiter_repetitionManager.cpp
# src/orbiter_nav_yaw_rand.cpp
src/orbiter_moveArmCuRobo.cpp
src/orbiter_getItemPose.cpp
src/orbiter_moveHead.cpp
src/orbiter_suck.cpp
src/orbiter_getNextAction.cpp
src/orbiter_signalFinish.cpp
)

set(TARGET_DEPENDS
    rclcpp
    yaml-cpp
    behaviortree_cpp_v3
    rclcpp_action
    # moveit_msgs
    # moveit_ros_planning_interface
    geometry_msgs
    std_msgs
    ims_interfaces
    urdf
    visualization_msgs
    ros2_aruco_interfaces
    nav2_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    # target_client
)

include_directories(
  include
)

add_executable(orbiter_bt_node src/orbiter_bt_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(orbiter_bt_node ${TARGET_DEPENDS})
rosidl_target_interfaces(orbiter_bt_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(orbiter_bt_node
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES})
target_link_libraries(orbiter_bt_node ${YAML_CPP_LIBRARIES})
# target_link_libraries(orbiter_bt_node ${moveit_msgs_LIBRARIES})
# target_link_libraries(orbiter_bt_node ${moveit_ros_planning_interface_LIBRARIES})
ament_target_dependencies(orbiter_bt_node ims_interfaces)


add_executable(dummy_ims_server src/dummy_ims_server.cpp)
rosidl_target_interfaces(dummy_ims_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(dummy_ims_server rclcpp)

install(TARGETS
orbiter_bt_node
dummy_ims_server
DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
