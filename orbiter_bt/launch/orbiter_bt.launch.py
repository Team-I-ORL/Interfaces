import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

  autonomy_node_cmd = Node(
      package="orbiter_bt",
      executable="orbiter_bt_node",
      name="autonomy_node",
  )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(autonomy_node_cmd)

  return ld