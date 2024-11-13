import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='aruco_pose_server',
            name='aruco_pose_server'
        ),
        Node(
            package='ros2_aruco',
            executable='get_drop_pose',
            name='get_drop_pose'
        ),
        # Node(
        #     package='suctionnet_ros2',
        #     executable='suctionnet',
        #     name='suctionnet',
        #     output='screen',
        #     arguments=[]  # Explicitly setting no arguments

        # ),
        Node(
            package='perception_interfaces',
            executable='perception_manager',
            name='perception_manager'
        ),
        Node(
            package='seg_mask',
            executable='yolo_sam2',
            name='yolo_sam2'
        )
    ])
