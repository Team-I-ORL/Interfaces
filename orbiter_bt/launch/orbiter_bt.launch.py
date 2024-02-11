import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  inventory_file = os.path.join(get_package_share_directory('navs'), "dummy_database", "inv_locs.yaml")
  print("inventory_file: ", inventory_file)
  autonomy_node_cmd = Node(
      package="orbiter_bt",
      executable="orbiter_bt",
      name="orbiter_bt",
      parameters=[{
        "inventory_file": inventory_file
        }],
  )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(autonomy_node_cmd)

  return ld