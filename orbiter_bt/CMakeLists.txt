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
find_package(behaviortree_cpp_v3 REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

# Install directories
install(DIRECTORY
bt_xml launch config
DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOR_SOURCES
src/orbiter_behaviors.cpp 
)

set(TARGET_DEPENDS
    rclcpp
    behaviortree_cpp_v3
    rclcpp_action
    nav2_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(include)
add_executable(orbiter_bt src/orbiter_bt_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(orbiter_bt ${TARGET_DEPENDS})

install(TARGETS
orbiter_bt_node
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
