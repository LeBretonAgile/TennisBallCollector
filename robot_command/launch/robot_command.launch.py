import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("robot_command").find("robot_command")

    cam_top_publisher_node = Node(
        package="robot_command",    
        executable="camera_top"
    )

    waypoint_publisher_node = Node(
        package="robot_command",
        executable="waypoint"
    )

    # command_publisher_node = Node(
    #     package="robot_command",
    #     executable="command"
    # )

    return LaunchDescription([
        cam_top_publisher_node,
        waypoint_publisher_node,
        #command_publisher_node
    ])